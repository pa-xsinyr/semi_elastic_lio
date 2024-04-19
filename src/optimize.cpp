// c++
#include <iostream>
#include <math.h>
#include <vector>

// eigen 
#include <Eigen/Core>

#include "cloudMap.h"

// utility
#include "utility.h"
#include "parameters.h"

// optimize factor
#include "imuFactor.h"
#include "lidarFactor.h"
#include "poseParameterization.h"
#include "lioOptimization.h"

optimizeSummary lioOptimization::optimizeByAnalyticLidar(const icpOptions &cur_icp_options, const voxelHashMap &voxel_map_temp, std::vector<point3D> &keypoints, cloudFrame *p_frame)//当前帧
{//命名空间或类包含与优化相关的函数或数据结构，负责处理激光雷达和IMU数据优化。类的成员函数，函数是用于优化的，使用解析法处理lidar数据。lioOptimization::optimizeByAnalyticLidar 是 LIO-SAM 中用于激光雷达数据优化的一个函数，它使用解析法来改善SLAM算法的性能

    const short nb_voxels_visited = p_frame->frame_id < cur_icp_options.init_num_frames ? 2 : cur_icp_options.voxel_neighborhood;//lidar扫描的体素数量，当前帧的帧id，ICP选项中的初始帧数、体素邻域大小
    const int kMinNumNeighbors = cur_icp_options.min_number_neighbors;//最小邻居数量
    const int kThresholdCapacity = p_frame->frame_id < cur_icp_options.init_num_frames ? 1 : cur_icp_options.threshold_voxel_occupancy;//体素占用阈值

    state *previous_state = nullptr;//是指向（前者）对象的指针
    Eigen::Vector3d previous_translation = Eigen::Vector3d::Zero();
    Eigen::Vector3d previous_velocity = Eigen::Vector3d::Zero();
    Eigen::Quaterniond previous_orientation = Eigen::Quaterniond::Identity();
//变量初始化
    if (p_frame->frame_id > 1) {//根据当前帧的帧ID判断是否为第一帧，如果不是第一帧，则获取前一帧的状态信息，包括平移、速度和方向。
        previous_state = all_cloud_frame[p_frame->id - 1]->p_state;//数组或列表，存储帧。当前帧的帧ID，帧的索引。访问该帧的状态信息
        previous_translation = previous_state->translation;
        previous_velocity = previous_state->translation - previous_state->translation_begin;//previous_velocity 的值等于前一状态的平移减去前一状态的初始平移。previous_state 是一个状态对象，其中包含了两个属性：translation 和 translation_begin
        previous_orientation = Eigen::Quaterniond(previous_state->rotation);
    }

    state *current_state = p_frame->p_state;
    Eigen::Quaterniond begin_quat = Eigen::Quaterniond(current_state->rotation_begin);
    Eigen::Quaterniond end_quat = Eigen::Quaterniond(current_state->rotation);
    Eigen::Vector3d begin_t = current_state->translation_begin;
    Eigen::Vector3d end_t = current_state->translation;//获取当前帧的状态信息，包括起始和结束时刻的平移、旋转

    int num_iter_icp = p_frame->frame_id < cur_icp_options.init_num_frames ? std::max(15, cur_icp_options.num_iters_icp) :
                       cur_icp_options.num_iters_icp;// 根据当前帧的帧ID选择迭代次数

    auto transformKeypoints = [&]()
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        for (auto &keypoint: keypoints) {// 对每个关键点进行变换
            if (cur_icp_options.point_to_plane_with_distortion || cur_icp_options.distance == CT_POINT_TO_PLANE) {
                double alpha_time = keypoint.alpha_time;
                Eigen::Quaterniond q = begin_quat.slerp(alpha_time, end_quat);
                q.normalize();
                R = q.toRotationMatrix();
                t = (1.0 - alpha_time) * begin_t + alpha_time * end_t;
            } else {
                R = end_quat.normalized().toRotationMatrix();
                t = end_t;
            }//根据当前帧的状态信息和激光雷达关键点的信息，对关键点进行变换。如果选项为“点对平面”，则对关键点进行时间插值变换；否则，直接将关键点变换到当前帧的位置

            keypoint.point = R * (R_imu_lidar * keypoint.raw_point + t_imu_lidar) + t;
        }// 对关键点进行变换
    };

    auto estimatePointNeighborhood = [&](std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &vector_neighbors,
                                           Eigen::Vector3d &location, double &planarity_weight)
    {//估计点邻域特征的lambda函数

        auto neighborhood = computeNeighborhoodDistribution(vector_neighbors);//计算邻域分布
        planarity_weight = std::pow(neighborhood.a2D, cur_icp_options.power_planarity);//计算平面性权重

        if (neighborhood.normal.dot(p_frame->p_state->translation_begin - location) < 0) {
            neighborhood.normal = -1.0 * neighborhood.normal;
        }
        return neighborhood;
    };//这段代码的主要作用是对激光雷达数据进行变换和特征估计，为后续的优化过程做准备。

    double lambda_weight = std::abs(cur_icp_options.weight_alpha);//absolute value。定义变量，调用abs函数，传入参数。初始化
    double lambda_neighborhood = std::abs(cur_icp_options.weight_neighborhood);
    const double kMaxPointToPlane = cur_icp_options.max_dist_to_plane_icp;//the constant kMaxPointToPlane is defined and initialized with the value of cur_icp_options.max_dist_to_plane_icp
    const double sum = lambda_weight + lambda_neighborhood;

    lambda_weight /= sum;
    lambda_neighborhood /= sum;

    int number_of_residuals = 0;//记录残差数量

    for (int iter(0); iter < num_iter_icp; iter++) {
        transformKeypoints();//函数，对关键点进行变换

        ceres::Problem problem;//创建ceres求解问题
        ceres::LossFunction *loss_function;

        switch (cur_icp_options.loss_function)//根据不同的损失函数类型，我们选择相应的损失函数。这些损失函数用于衡量模型拟合数据的好坏
        {
            case LeastSquares::STANDARD:
                break;
            case LeastSquares::CAUCHY:
                loss_function = new ceres::CauchyLoss(cur_icp_options.ls_sigma);
                break;
            case LeastSquares::HUBER:
                loss_function = new ceres::HuberLoss(cur_icp_options.ls_sigma);
                break;
            case LeastSquares::TOLERANT:
                loss_function = new ceres::TolerantLoss(cur_icp_options.ls_tolerant_min_threshold, cur_icp_options.ls_sigma);
                break;
            case LeastSquares::TRUNCATED:
                loss_function = new TruncatedLoss(cur_icp_options.ls_sigma);
                break;
        }

        ceres::LocalParameterization *parameterization = new RotationParameterization();// 创建一个旋转参数化对象，用于描述旋转参数
//在 new ()可以传递参数来初始化对象。new{}可以使用列表初始化来初始化对象。new Type[n]：分配一个包含 n 个 Type 类型对象的数组。
        switch (cur_icp_options.distance) {
            case CT_POINT_TO_PLANE://是 Ceres的成员函数，向优化问题中添加一个参数块。参数块是希望优化的变量，在这个函数中可以指定参数块的初始值、大小和参数化方式。
                problem.AddParameterBlock(&begin_quat.x(), 4, parameterization);//起始姿态的四元数。&begin_quat.x()一个四元数的参数块。begin_quat 一个四元数对象，.x() 获取四元数的 x 分量（实部）。&取地址操作将四元数的x分量传递给 problem.AddParameterBlock() 函数。参数块的大小，表示四元数有 4 个分量（x、y、z、w）。parameterization一个 LocalParameterization 对象，定义四元数的参数化方式
                problem.AddParameterBlock(&end_quat.x(), 4, parameterization);//结束姿态的四元数
                problem.AddParameterBlock(&begin_t.x(), 3);//起始位移
                problem.AddParameterBlock(&end_t.x(), 3);//结束位移
                break;
            case POINT_TO_PLANE:
                problem.AddParameterBlock(&end_quat.x(), 4, parameterization);
                problem.AddParameterBlock(&end_t.x(), 3);
                break;
        }

        int num_residuals = 0;// 初始化残差数量

        int num_keypoints = keypoints.size();// 获取关键点的数量
        int num_threads = cur_icp_options.ls_num_threads;// 获取线程数目

        for (int k = 0; k < num_keypoints; ++k) {// 遍历所有关键点
            auto &keypoint = keypoints[k]; // 获取当前关键点的引用
            auto &raw_point = keypoint.raw_point;// 获取当前关键点的原始点

            std::vector<voxel> voxels; // 创建一个体素向量
            auto vector_neighbors = searchNeighbors(voxel_map_temp, keypoint.point,//函数，搜索邻居点。体素地图（可能是一个数据结构，用于存储点云数据）。关键点的位置。
                                                     nb_voxels_visited, cur_icp_options.size_voxel_map,//已访问的体素数量。体素地图的大小，最大邻居数量
                                                     cur_icp_options.max_number_neighbors, kThresholdCapacity,//阈值容量
                                                     cur_icp_options.estimate_normal_from_neighborhood ? nullptr : &voxels);//条件表达式，用于选择不同的值。为真（非零），则整个表达式的值为 nullptr。为假（零），值为 &voxels。nullptr表示空指针，&voxels表示指向voxels的指针
//? :条件运算符（三元运算符），用于根据条件选择不同的值
// 使用搜索函数找到当前关键点附近的邻域点，并估计法线
// 通过设置参数确定是否从邻域中估计法线
            if (vector_neighbors.size() < kMinNumNeighbors)
                continue; // 如果邻域点数量小于最小邻域点数量要求，则跳过当前关键点

            double weight;//权重变量

            Eigen::Vector3d location = R_imu_lidar * raw_point + t_imu_lidar; // 计算当前关键点在IMU到激光雷达坐标系下的位置

            auto neighborhood = estimatePointNeighborhood(vector_neighbors, location/*raw_point*/, weight);// 估计当前关键点的邻域，计算权重

            weight = lambda_weight * weight + lambda_neighborhood * std::exp(-(vector_neighbors[0] -//中括号访问C++中vector容器中特定索引位置的元素。vector_neighbors是vector 对象，访问该向量中的第一个元素（索引为0的元素）
                     keypoint.point).norm() / (kMaxPointToPlane * kMinNumNeighbors));
// 计算最终的权重，结合 lambda_weight 和 lambda_neighborhood
            double point_to_plane_dist;//点到平面距离变量
            std::set<voxel> neighbor_voxels;//创建体素集合
            for (int i(0); i < cur_icp_options.num_closest_neighbors; ++i) {//技巧：for循环中i<  .A那么就遍历A
                point_to_plane_dist = std::abs((keypoint.point - vector_neighbors[i]).transpose() * neighborhood.normal);//计算当前邻域点到平面得距离

                if (point_to_plane_dist < cur_icp_options.max_dist_to_plane_icp) {
// 如果点到平面距离小于最大距离阈值
                    num_residuals++;// 残差数量增加1

                    Eigen::Vector3d norm_vector = neighborhood.normal;
                    norm_vector.normalize();// 将法线向量归一化
                    double norm_offset = - norm_vector.dot(vector_neighbors[i]);// 计算法线偏移

                    switch (cur_icp_options.distance) {//距离度量方法
                        case CT_POINT_TO_PLANE:
                        {
                            CTLidarPlaneNormFactor *cost_function = new CTLidarPlaneNormFactor(keypoints[k].raw_point, norm_vector, norm_offset, keypoints[k].alpha_time, weight);// 创建点到平面残差函数对象
                            problem.AddResidualBlock(cost_function, loss_function, &begin_t.x(), &begin_quat.x(), &end_t.x(), &end_quat.x());// 向优化问题中添加残差项
                            break;
                        }
                        case POINT_TO_PLANE:
                        {
                            Eigen::Vector3d point_end = end_quat.inverse() * keypoints[k].point - end_quat.inverse() * end_t;//计算结束时刻的点。四元数表示旋转。三维向量，关键点的位置。平移。先转到世界坐标系，再减去平移
                            LidarPlaneNormFactor *cost_function = new LidarPlaneNormFactor(point_end, norm_vector, norm_offset, weight);// 创建点到平面残差函数对象
                            problem.AddResidualBlock(cost_function, loss_function, &end_t.x(), &end_quat.x());// 向优化问题中添加残差项
                            break;
                        }
                    }
                }
            }

            if(num_residuals >= cur_icp_options.max_num_residuals) break;
        }// 如果残差数量达到最大值，则提前终止循环

        if (p_frame->frame_id > 1) {// 如果当前帧ID大于1
            if (cur_icp_options.distance == CT_POINT_TO_PLANE)
            {
                LocationConsistencyFactor *cost_location_consistency = new LocationConsistencyFactor(previous_translation, sqrt(num_residuals * cur_icp_options.beta_location_consistency * laser_point_cov));
                problem.AddResidualBlock(cost_location_consistency, nullptr, &begin_t.x());
//创建位置一致性因子对象。向优化问题中添加位置一致性残差项
                RotationConsistencyFactor *cost_rotation_consistency = new RotationConsistencyFactor(previous_orientation, sqrt(num_residuals * cur_icp_options.beta_orientation_consistency * laser_point_cov));
                problem.AddResidualBlock(cost_rotation_consistency, nullptr, &begin_quat.x());

                SmallVelocityFactor *cost_small_velocity = new SmallVelocityFactor(sqrt(num_residuals * cur_icp_options.beta_small_velocity * laser_point_cov));
                problem.AddResidualBlock(cost_small_velocity, nullptr, &begin_t.x(), &end_t.x());
            }
        }
        if (num_residuals < cur_icp_options.min_number_neighbors)
        {

            std::stringstream ss_out;
            ss_out << "[Optimization] Error : not enough keypoints selected in ct-icp !" << std::endl;
            ss_out << "[Optimization] number_of_residuals : " << num_residuals << std::endl;
            optimizeSummary summary;
            summary.success = false;
            summary.num_residuals_used = num_residuals;
            summary.error_log = ss_out.str();
            if (cur_icp_options.debug_print) {
                std::cout << summary.error_log;
            }
            return summary;
        }

        ceres::Solver::Options ceres_options;
        ceres_options.max_num_iterations = cur_icp_options.ls_max_num_iters;
        ceres_options.num_threads = cur_icp_options.ls_num_threads;
        ceres_options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
        ceres_options.minimizer_progress_to_stdout = false;

        ceres::Solver::Summary summary_ceres;
        ceres::Solve(ceres_options, &problem, &summary_ceres);

        if (!summary_ceres.IsSolutionUsable()) {
            std::cout << summary_ceres.FullReport() << std::endl;
            throw std::runtime_error("Error During Optimization");
        }
        if (cur_icp_options.debug_print) {
            std::cout << summary_ceres.BriefReport() << std::endl;
        }


        begin_quat.normalize();
        end_quat.normalize();

        double diff_trans = 0, diff_rot = 0;

        diff_trans += (current_state->translation_begin - begin_t).norm();
        diff_rot += AngularDistance(current_state->rotation_begin, begin_quat);

        diff_trans += (current_state->translation - end_t).norm();
        diff_rot += AngularDistance(current_state->rotation, end_quat);

        switch (cur_icp_options.distance) {
            case CT_POINT_TO_PLANE:
                current_state->translation_begin = begin_t;
                current_state->translation = end_t;
                current_state->rotation_begin = begin_quat;
                current_state->rotation = end_quat;
                break;
            case POINT_TO_PLANE:
                current_state->translation = end_t;
                current_state->rotation = end_quat;
                break;
        }

        if ((p_frame->frame_id > 1) &&
            (diff_rot < cur_icp_options.threshold_orientation_norm &&
             diff_trans < cur_icp_options.threshold_translation_norm)) {

            if (cur_icp_options.debug_print) {
                std::cout << "Optimization: Finished with N=" << iter << " ICP iterations" << std::endl;

            }
            break;
        }
    }
    transformKeypoints();

    optimizeSummary summary;
    summary.success = true;
    summary.num_residuals_used = number_of_residuals;
    return summary;
}

optimizeSummary lioOptimization::optimizeByAnalyticLio(const icpOptions &cur_icp_options, const voxelHashMap &voxel_map_temp, std::vector<point3D> &keypoints, cloudFrame *p_frame)
{

    const short nb_voxels_visited = p_frame->frame_id < cur_icp_options.init_num_frames ? 2 : cur_icp_options.voxel_neighborhood;
    const int kMinNumNeighbors = cur_icp_options.min_number_neighbors;
    const int kThresholdCapacity = p_frame->frame_id < cur_icp_options.init_num_frames ? 1 : cur_icp_options.threshold_voxel_occupancy;

    state *previous_state = nullptr;
    Eigen::Vector3d previous_translation = Eigen::Vector3d::Zero();
    Eigen::Vector3d previous_velocity = Eigen::Vector3d::Zero();
    Eigen::Quaterniond previous_orientation = Eigen::Quaterniond::Identity();

    if (p_frame->frame_id > 1) {
        previous_state = all_cloud_frame[p_frame->id - 1]->p_state;
        previous_translation = previous_state->translation;
        previous_velocity = previous_state->translation - previous_state->translation_begin;
        previous_orientation = Eigen::Quaterniond(previous_state->rotation);
    }

    state *current_state = p_frame->p_state;
    Eigen::Quaterniond begin_quat = Eigen::Quaterniond(current_state->rotation_begin);
    Eigen::Quaterniond end_quat = Eigen::Quaterniond(current_state->rotation);
    Eigen::Vector3d begin_t = current_state->translation_begin;
    Eigen::Vector3d end_t = current_state->translation;

    Eigen::Matrix<double, 9 ,1> begin_velocity_bias;
    begin_velocity_bias.segment<3>(0) = current_state->velocity_begin;
    begin_velocity_bias.segment<3>(3) = current_state->ba_begin;
    begin_velocity_bias.segment<3>(6) = current_state->bg_begin;
    Eigen::Matrix<double, 9 ,1> end_velocity_bias;
    end_velocity_bias.segment<3>(0) = current_state->velocity;
    end_velocity_bias.segment<3>(3) = current_state->ba;
    end_velocity_bias.segment<3>(6) = current_state->bg;

    int num_iter_icp = p_frame->frame_id < cur_icp_options.init_num_frames ? std::max(15, cur_icp_options.num_iters_icp) :
                       cur_icp_options.num_iters_icp;

    auto transformKeypoints = [&]()
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        for (auto &keypoint: keypoints) {
            if (cur_icp_options.point_to_plane_with_distortion || cur_icp_options.distance == CT_POINT_TO_PLANE) {
                double alpha_time = keypoint.alpha_time;
                Eigen::Quaterniond q = begin_quat.slerp(alpha_time, end_quat);
                q.normalize();
                R = q.toRotationMatrix();
                t = (1.0 - alpha_time) * begin_t + alpha_time * end_t;
            } else {
                R = end_quat.normalized().toRotationMatrix();
                t = end_t;
            }

            keypoint.point = R * (R_imu_lidar * keypoint.raw_point + t_imu_lidar) + t;
        }
    };

    auto estimatePointNeighborhood = [&](std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &vector_neighbors,
                                           Eigen::Vector3d &location, double &planarity_weight)
    {

        auto neighborhood = computeNeighborhoodDistribution(vector_neighbors);
        planarity_weight = std::pow(neighborhood.a2D, cur_icp_options.power_planarity);

        if (neighborhood.normal.dot(p_frame->p_state->translation_begin - location) < 0) {
            neighborhood.normal = -1.0 * neighborhood.normal;
        }
        return neighborhood;
    };

    double lambda_weight = std::abs(cur_icp_options.weight_alpha);
    double lambda_neighborhood = std::abs(cur_icp_options.weight_neighborhood);
    const double kMaxPointToPlane = cur_icp_options.max_dist_to_plane_icp;
    const double sum = lambda_weight + lambda_neighborhood;

    lambda_weight /= sum;
    lambda_neighborhood /= sum;

    int number_of_residuals = 0;

    for (int iter(0); iter < num_iter_icp; iter++) {
        transformKeypoints();

        ceres::Problem problem;
        ceres::LossFunction *loss_function;

        switch (cur_icp_options.loss_function)
        {
            case LeastSquares::STANDARD:
                break;
            case LeastSquares::CAUCHY:
                loss_function = new ceres::CauchyLoss(cur_icp_options.ls_sigma);
                break;
            case LeastSquares::HUBER:
                loss_function = new ceres::HuberLoss(cur_icp_options.ls_sigma);
                break;
            case LeastSquares::TOLERANT:
                loss_function = new ceres::TolerantLoss(cur_icp_options.ls_tolerant_min_threshold, cur_icp_options.ls_sigma);
                break;
            case LeastSquares::TRUNCATED:
                loss_function = new TruncatedLoss(cur_icp_options.ls_sigma);
                break;
        }

        ceres::LocalParameterization *parameterization = new RotationParameterization();

        switch (cur_icp_options.distance) {
            case CT_POINT_TO_PLANE:
                problem.AddParameterBlock(&begin_quat.x(), 4, parameterization);
                problem.AddParameterBlock(&end_quat.x(), 4, parameterization);
                problem.AddParameterBlock(&begin_t.x(), 3);
                problem.AddParameterBlock(&end_t.x(), 3);
                problem.AddParameterBlock(&begin_velocity_bias[0], 9);
                problem.AddParameterBlock(&end_velocity_bias[0], 9);
                break;
            case POINT_TO_PLANE:
                problem.AddParameterBlock(&end_quat.x(), 4, parameterization);
                problem.AddParameterBlock(&end_t.x(), 3);
                problem.AddParameterBlock(&end_velocity_bias[0], 9);
                problem.AddParameterBlock(&begin_quat.x(), 4, parameterization);
                problem.AddParameterBlock(&begin_t.x(), 3);
                problem.AddParameterBlock(&begin_velocity_bias[0], 9);
                break;
        }

        int num_residuals = 0;
        int num_keypoints = keypoints.size();
        int num_threads = cur_icp_options.ls_num_threads;

        for (int k = 0; k < num_keypoints; ++k) {
            auto &keypoint = keypoints[k];
            auto &raw_point = keypoint.raw_point;

            std::vector<voxel> voxels;
            auto vector_neighbors = searchNeighbors(voxel_map_temp, keypoint.point,
                                                     nb_voxels_visited, cur_icp_options.size_voxel_map,
                                                     cur_icp_options.max_number_neighbors, kThresholdCapacity,
                                                     cur_icp_options.estimate_normal_from_neighborhood ? nullptr : &voxels);

            if (vector_neighbors.size() < kMinNumNeighbors)
                continue;

            double weight;

            Eigen::Vector3d location = R_imu_lidar * raw_point + t_imu_lidar;

            auto neighborhood = estimatePointNeighborhood(vector_neighbors, location, weight);

            weight = lambda_weight * weight + lambda_neighborhood * std::exp(-(vector_neighbors[0] -
                     keypoint.point).norm() / (kMaxPointToPlane * kMinNumNeighbors));

            double point_to_plane_dist;
            std::set<voxel> neighbor_voxels;
            for (int i(0); i < cur_icp_options.num_closest_neighbors; ++i) {
                point_to_plane_dist = std::abs((keypoint.point - vector_neighbors[i]).transpose() * neighborhood.normal);

                if (point_to_plane_dist < cur_icp_options.max_dist_to_plane_icp) {

                    num_residuals++;

                    Eigen::Vector3d norm_vector = neighborhood.normal;
                    norm_vector.normalize();
                    double norm_offset = - norm_vector.dot(vector_neighbors[i]);

                    switch (cur_icp_options.distance) {
                        case CT_POINT_TO_PLANE:
                        {
                            CTLidarPlaneNormFactor *cost_function = new CTLidarPlaneNormFactor(keypoints[k].raw_point, norm_vector, norm_offset, keypoints[k].alpha_time, weight);
                            problem.AddResidualBlock(cost_function, loss_function, &begin_t.x(), &begin_quat.x(), &end_t.x(), &end_quat.x());
                            break;
                        }
                        case POINT_TO_PLANE:
                        {
                            Eigen::Vector3d point_end = end_quat.inverse() * keypoints[k].point - end_quat.inverse() * end_t;
                            LidarPlaneNormFactor *cost_function = new LidarPlaneNormFactor(point_end, norm_vector, norm_offset, weight);
                            problem.AddResidualBlock(cost_function, loss_function, &end_t.x(), &end_quat.x());
                            break;
                        }
                    }
                }
            }

            if(num_residuals >= cur_icp_options.max_num_residuals) break;
        }

        if (p_frame->frame_id > 1) {

            switch (cur_icp_options.distance) {
                case CT_POINT_TO_PLANE:
                {
                    LocationConsistencyFactor *cost_location_consistency = new LocationConsistencyFactor(previous_translation, sqrt(num_residuals * cur_icp_options.beta_location_consistency * laser_point_cov));
                    problem.AddResidualBlock(cost_location_consistency, nullptr, &begin_t.x());

                    RotationConsistencyFactor *cost_rotation_consistency = new RotationConsistencyFactor(previous_orientation, sqrt(num_residuals * cur_icp_options.beta_orientation_consistency * laser_point_cov));
                    problem.AddResidualBlock(cost_rotation_consistency, nullptr, &begin_quat.x());

                    SmallVelocityFactor *cost_small_velocity = new SmallVelocityFactor(sqrt(num_residuals * cur_icp_options.beta_small_velocity * laser_point_cov));
                    problem.AddResidualBlock(cost_small_velocity, nullptr, &begin_t.x(), &end_t.x());

                    if (p_frame->p_state->pre_integration->sum_dt < 10.0)
                    {
                        CTImuFactor* imu_factor = new CTImuFactor(all_cloud_frame[p_frame->id]->p_state->pre_integration, 1);
                        problem.AddResidualBlock(imu_factor, loss_function, &begin_t.x(), &begin_quat.x(), &begin_velocity_bias[0],  &end_t.x(), &end_quat.x(), &end_velocity_bias[0]);

                        VelocityConsistencyFactor *cost_velocity_consistency = new VelocityConsistencyFactor(all_cloud_frame[p_frame->id - 1]->p_state, sqrt(num_residuals * cur_icp_options.beta_constant_velocity * laser_point_cov));
                        problem.AddResidualBlock(cost_velocity_consistency, nullptr, &begin_velocity_bias[0]);
                    }
                    break;
                }
                case POINT_TO_PLANE:
                {
                    CTImuFactor* imu_factor = new CTImuFactor(p_frame->p_state->pre_integration, 1);
                    problem.AddResidualBlock(imu_factor, loss_function, &begin_t.x(), &begin_quat.x(), &begin_velocity_bias[0],  &end_t.x(), &end_quat.x(), &end_velocity_bias[0]);

                    LocationConsistencyFactor *cost_location_consistency = new LocationConsistencyFactor(previous_translation, sqrt(num_residuals * cur_icp_options.beta_location_consistency * laser_point_cov));
                    problem.AddResidualBlock(cost_location_consistency, nullptr, &begin_t.x());

                    RotationConsistencyFactor *cost_rotation_consistency = new RotationConsistencyFactor(previous_orientation, sqrt(num_residuals * cur_icp_options.beta_orientation_consistency * laser_point_cov));
                    problem.AddResidualBlock(cost_rotation_consistency, nullptr, &begin_quat.x());

                    VelocityConsistencyFactor *cost_velocity_consistency = new VelocityConsistencyFactor(all_cloud_frame[p_frame->id - 1]->p_state, sqrt(num_residuals * cur_icp_options.beta_constant_velocity * laser_point_cov));
                    problem.AddResidualBlock(cost_velocity_consistency, nullptr, &begin_velocity_bias[0]);

                    break;
                }
            }
        }
        if (num_residuals < cur_icp_options.min_number_neighbors)
        {
            std::cout << "not enough ICP residuals!" << std::endl;
            std::stringstream ss_out;
            ss_out << "[Optimization] Error : not enough keypoints selected in ct-icp !" << std::endl;
            ss_out << "[Optimization] number_of_residuals : " << num_residuals << std::endl;
            optimizeSummary summary;
            summary.success = false;
            summary.num_residuals_used = num_residuals;
            summary.error_log = ss_out.str();
            if (cur_icp_options.debug_print) {
                std::cout << summary.error_log;
            }
            return summary;
        }

        ceres::Solver::Options ceres_options;
        ceres_options.max_num_iterations = cur_icp_options.ls_max_num_iters;
        ceres_options.num_threads = cur_icp_options.ls_num_threads;
        ceres_options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
        ceres_options.minimizer_progress_to_stdout = false;

        ceres::Solver::Summary summary_ceres;
        ceres::Solve(ceres_options, &problem, &summary_ceres);

        if (!summary_ceres.IsSolutionUsable()) {
            std::cout << summary_ceres.FullReport() << std::endl;
            throw std::runtime_error("Error During Optimization");
        }
        if (cur_icp_options.debug_print) {
            std::cout << summary_ceres.BriefReport() << std::endl;
        }


        begin_quat.normalize();
        end_quat.normalize();

        double diff_trans = 0, diff_rot = 0, diff_velocity = 0;

        diff_trans += (current_state->translation_begin - begin_t).norm();
        diff_rot += AngularDistance(current_state->rotation_begin, begin_quat);
        diff_velocity += (current_state->velocity_begin - begin_velocity_bias.segment<3>(0)).norm();

        diff_trans += (current_state->translation - end_t).norm();
        diff_rot += AngularDistance(current_state->rotation, end_quat);
        diff_velocity += (current_state->velocity - end_velocity_bias.segment<3>(0)).norm();

        switch (cur_icp_options.distance) {
            case CT_POINT_TO_PLANE:
                current_state->translation_begin = begin_t;
                current_state->rotation_begin = begin_quat;
                current_state->velocity_begin = begin_velocity_bias.segment<3>(0);
                current_state->ba_begin = begin_velocity_bias.segment<3>(3);
                current_state->bg_begin = begin_velocity_bias.segment<3>(6);

                current_state->translation = end_t;
                current_state->rotation = end_quat;
                current_state->velocity = end_velocity_bias.segment<3>(0);
                current_state->ba = end_velocity_bias.segment<3>(3);
                current_state->bg = end_velocity_bias.segment<3>(6);
                break;
            case POINT_TO_PLANE:
                current_state->translation_begin = begin_t;
                current_state->rotation_begin = begin_quat;
                current_state->velocity_begin = begin_velocity_bias.segment<3>(0);
                current_state->ba_begin = begin_velocity_bias.segment<3>(3);
                current_state->bg_begin = begin_velocity_bias.segment<3>(6);

                current_state->translation = end_t;
                current_state->rotation = end_quat;
                current_state->velocity = end_velocity_bias.segment<3>(0);
                current_state->ba = end_velocity_bias.segment<3>(3);
                current_state->bg = end_velocity_bias.segment<3>(6);
                break;
        }

        if ((p_frame->frame_id > 1) &&
            (diff_rot < cur_icp_options.threshold_orientation_norm &&
             diff_trans < cur_icp_options.threshold_translation_norm && 
             diff_velocity < cur_icp_options.threshold_translation_norm)) {

            if (cur_icp_options.debug_print) {
                std::cout << "Optimization: Finished with N=" << iter << " ICP iterations" << std::endl;

            }
            break;
        }
    }
    transformKeypoints();

    optimizeSummary summary;
    summary.success = true;
    summary.num_residuals_used = number_of_residuals;
    return summary;
}

Neighborhood lioOptimization::computeNeighborhoodDistribution(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points)
{
    Neighborhood neighborhood;
    // Compute the normals
    Eigen::Vector3d barycenter(Eigen::Vector3d(0, 0, 0));
    for (auto &point: points) {
        barycenter += point;
    }

    barycenter /= (double) points.size();
    neighborhood.center = barycenter;//计算点集重心作为邻域中心点

    Eigen::Matrix3d covariance_Matrix(Eigen::Matrix3d::Zero());
    for (auto &point: points) {
        for (int k = 0; k < 3; ++k)
            for (int l = k; l < 3; ++l)
                covariance_Matrix(k, l) += (point(k) - barycenter(k)) *
                                           (point(l) - barycenter(l));
    }
    covariance_Matrix(1, 0) = covariance_Matrix(0, 1);
    covariance_Matrix(2, 0) = covariance_Matrix(0, 2);
    covariance_Matrix(2, 1) = covariance_Matrix(1, 2);
    neighborhood.covariance = covariance_Matrix;//计算点集协方差矩阵，获得点云分布形状的信息
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance_Matrix);
    Eigen::Vector3d normal(es.eigenvectors().col(0).normalized());
    neighborhood.normal = normal;
//使用协方差矩阵的特征值和特征向量计算邻域的法线向量。
    double sigma_1 = sqrt(std::abs(es.eigenvalues()[2]));
    double sigma_2 = sqrt(std::abs(es.eigenvalues()[1]));
    double sigma_3 = sqrt(std::abs(es.eigenvalues()[0]));
    neighborhood.a2D = (sigma_2 - sigma_3) / sigma_1;//计算和返回邻域的二维延展性（a2D），用于描述邻域的平坦度

    if (neighborhood.a2D != neighborhood.a2D) {
        throw std::runtime_error("error");
    }

    return neighborhood;
}//通过计算点集的邻域分布，提供了关于局部结构的重要信息，如中心位置、主要方向、协方差等。这些信息可用于点云处理中的特征提取、点云分割、物体识别等。

using pair_distance_t = std::tuple<double, Eigen::Vector3d, voxel>;

struct comparator {
    bool operator()(const pair_distance_t &left, const pair_distance_t &right) const {
        return std::get<0>(left) < std::get<0>(right);
    }
};

using priority_queue_t = std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, comparator>;

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> lioOptimization::searchNeighbors(const voxelHashMap &map, const Eigen::Vector3d &point,
        int nb_voxels_visited, double size_voxel_map, int max_num_neighbors, int threshold_voxel_capacity, std::vector<voxel> *voxels)
{

    if (voxels != nullptr)
        voxels->reserve(max_num_neighbors);

    short kx = static_cast<short>(point[0] / size_voxel_map);
    short ky = static_cast<short>(point[1] / size_voxel_map);
    short kz = static_cast<short>(point[2] / size_voxel_map);

    priority_queue_t priority_queue;

    voxel voxel_temp(kx, ky, kz);
    for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx) {
        for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy) {
            for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz) {
                voxel_temp.x = kxx;
                voxel_temp.y = kyy;
                voxel_temp.z = kzz;

                auto search = map.find(voxel_temp);
                if (search != map.end()) {
                    const auto &voxel_block = search.value();
                    if (voxel_block.NumPoints() < threshold_voxel_capacity)
                        continue;
                    for (int i(0); i < voxel_block.NumPoints(); ++i) {
                        auto &neighbor = voxel_block.points[i];
                        double distance = (neighbor - point).norm();
                        if (priority_queue.size() == max_num_neighbors) {
                            if (distance < std::get<0>(priority_queue.top())) {
                                priority_queue.pop();
                                priority_queue.emplace(distance, neighbor, voxel_temp);
                            }
                        } else
                            priority_queue.emplace(distance, neighbor, voxel_temp);
                    }
                }
            }
        }
    }

    auto size = priority_queue.size();
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> closest_neighbors(size);
    if (voxels != nullptr) {
        voxels->resize(size);
    }
    for (auto i = 0; i < size; ++i) {
        closest_neighbors[size - 1 - i] = std::get<1>(priority_queue.top());
        if (voxels != nullptr)
            (*voxels)[size - 1 - i] = std::get<2>(priority_queue.top());
        priority_queue.pop();
    }


    return closest_neighbors;
}

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> lioOptimization::selectClosestNeighbors(
        const std::vector<std::vector<Eigen::Vector3d> const *> &neighbors_ptr, const Eigen::Vector3d &pt_keypoint, int num_neighbors, int max_num_neighbors)
{
    std::vector<std::pair<double, Eigen::Vector3d>> distance_neighbors;
    distance_neighbors.reserve(neighbors_ptr.size());
    for (auto &it_ptr: neighbors_ptr) {
        for (auto &it: *it_ptr) {
            double sq_dist = (pt_keypoint - it).squaredNorm();
            distance_neighbors.emplace_back(sq_dist, it);
        }
    }

    int real_number_neighbors = std::min(max_num_neighbors, (int) distance_neighbors.size());
    std::partial_sort(distance_neighbors.begin(),
                      distance_neighbors.begin() + real_number_neighbors,
                      distance_neighbors.end(),
                      [](const std::pair<double, Eigen::Vector3d> &left,
                         const std::pair<double, Eigen::Vector3d> &right) {
                          return left.first < right.first;
                      });

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> neighbors(real_number_neighbors);
    for (auto i(0); i < real_number_neighbors; ++i)
        neighbors[i] = distance_neighbors[i].second;
    return neighbors;
}

estimationSummary lioOptimization::optimize(cloudFrame *p_frame, const icpOptions &cur_icp_options, estimationSummary &summary, double sample_voxel_size)
{
    std::vector<point3D> keypoints;
    gridSampling(p_frame->point_frame, keypoints, sample_voxel_size);

    auto num_keypoints = (int) keypoints.size();
    summary.sample_size = num_keypoints;

    {
        optimizeSummary optimize_summary;
        if (options.optimize_options.solver == LIO && initial_flag) {
            optimize_summary = optimizeByAnalyticLio(cur_icp_options, voxel_map, keypoints, p_frame);
        } else {
            optimize_summary = optimizeByAnalyticLidar(cur_icp_options, voxel_map, keypoints, p_frame);
        }
        summary.success = optimize_summary.success;
        summary.number_of_residuals = optimize_summary.num_residuals_used;

        if (!summary.success) {
            summary.success = false;
            return summary;
        }

        Eigen::Quaterniond q_begin = p_frame->p_state->rotation_begin;
        Eigen::Quaterniond q_end = p_frame->p_state->rotation;
        Eigen::Vector3d t_begin = p_frame->p_state->translation_begin;
        Eigen::Vector3d t_end = p_frame->p_state->translation;
        for (auto &point_temp: p_frame->point_frame) {
            transformPoint(options.motion_compensation, point_temp, q_begin, q_end, t_begin, t_end, R_imu_lidar, t_imu_lidar);
        }
    }
    std::vector<point3D>().swap(summary.keypoints);
    summary.keypoints = keypoints;
    summary.state_frame->release();
    summary.state_frame = new state(p_frame->p_state, true);

    return summary;
}