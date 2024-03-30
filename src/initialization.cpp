#include "lioOptimization.h"

void lioOptimization::motionInitialization()//成员函数，运动初始化过程
{
    bool result = false;

    if(all_cloud_frame.size() >= options.num_for_initialization)//检查存储点云帧的向量是否包含足够的帧数，以满足初始化的要求
    {
        Eigen::Vector3d g_sum = Eigen::Vector3d::Zero();//初始化，积累IMU加速度变化

        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> v_lidar_pose;//存储LIDAR位姿。是 Eigen 库中的一个内存分配器，用于确保矩阵在内存中的对齐。
        v_lidar_pose.push_back(Eigen::MatrixXd::Identity(4, 4));

        Eigen::Matrix4d initial_pose = Eigen::MatrixXd::Identity(4, 4);//初始化为单位矩阵
        initial_pose.block<3, 3>(0, 0) = all_cloud_frame[0]->p_state->rotation.toRotationMatrix();//提取3x3矩阵，起始位置行0列0。从第一帧点云中提取姿态信息，填充初始位姿矩阵的旋转部分
        initial_pose.block<3, 1>(0, 3) = all_cloud_frame[0]->p_state->translation;//从第一帧点云中提取平移信息，填充初始位姿矩阵的平移部分

        for (int i = 1; i < all_cloud_frame.size(); i++)//遍历剩余的点云帧
        {
            double dt = all_cloud_frame[i]->p_state->pre_integration->sum_dt;//获取当前点云帧的IMU数据的时间间隔
            Eigen::Vector3d g_temp = all_cloud_frame[i]->p_state->pre_integration->delta_v / dt;//计算当前帧的IMU加速度变化
            g_sum += g_temp;//累积IMU加速度变化

            Eigen::Matrix4d current_pose = Eigen::MatrixXd::Identity(4, 4);//初始化当前帧的位姿矩阵为单位矩阵。
            current_pose.block<3, 3>(0, 0) = all_cloud_frame[i]->p_state->rotation.toRotationMatrix();
            current_pose.block<3, 1>(0, 3) = all_cloud_frame[i]->p_state->translation;

            Eigen::Matrix4d pose_temp = initial_pose.inverse() * current_pose;//计算当前帧相对于初始帧的位姿变换。
            current_pose.block<3, 3>(0, 0) = pose_temp.block<3, 3>(0, 0) * R_imu_lidar;//通过IMU和LiDAR之间的外部姿态关系，调整当前帧的旋转部分。
            current_pose.block<3, 1>(0, 3) = R_imu_lidar.transpose() * (pose_temp.block<3, 3>(0, 0) * t_imu_lidar + pose_temp.block<3, 1>(0, 3) - t_imu_lidar);//调整当前帧的平移部分

            v_lidar_pose.push_back(current_pose);//将调整后的LiDAR位姿存储起来。
        }//这段代码将每个激光雷达时间点的位姿从初始时间点转移到了各自的时间点。

        Eigen::Vector3d g_average;
        g_average = g_sum * 1.0 / ((int)all_cloud_frame.size() - 1);//IMU加速度变化的平均值

        double variance = 0;//IMU加速度变化的方差

        for (int i = 1; i < all_cloud_frame.size(); i++)
        {
            double dt = all_cloud_frame[i]->p_state->pre_integration->sum_dt;
            Eigen::Vector3d g_temp = all_cloud_frame[i]->p_state->pre_integration->delta_v / dt;
            variance += (g_temp - g_average).transpose() * (g_temp - g_average);
        }

        variance = sqrt(variance / ((int)all_cloud_frame.size() - 1));//计算IMU加速度变化的方差

        if(variance < 0.25)//根据IMU加速度变化的平均值和方差，判断是否具有足够的IMU激励来进行初始化。若方差小于某个阈值，则认为IMU激励不足，初始化失败，这个阈值可以根据实际情况进行调整。
        {
            ROS_INFO("IMU excitation not enouth!");//如果IMU的激励不足，则通过 ROS 的 INFO 级别打印消息，提示用户IMU的激励不足。
            return;
        }

        if(initialLidarStructure(v_lidar_pose))//调用 initialLidarStructure() 函数来进行后续的LiDAR结构初始化，返回一个布尔值，表示初始化是否成功。
            result = true;
        else
            ROS_INFO("misalign lidar structure with IMU");//如果初始化LiDAR结构失败，则通过 ROS 的 INFO 级别打印消息，提示用户LiDAR的结构与IMU不匹配

        if(result == true)
            initial_flag = true;    
    }
}

void lioOptimization::staticInitialization(cloudFrame *p_frame)//向 cloudFrame 对象的指针
{
    if(p_frame->frame_id == 1) time_init = p_frame->time_sweep_begin;

    if(p_frame->time_sweep_end - time_init > 3.0)
    {
        Eigen::Vector3d avg_velocity = p_frame->p_state->translation / (p_frame->time_sweep_end - time_init);//通过将 p_frame 的平移除以时间差来计算平均速度

        if(G.norm() < 0.1)//加速度G的范数（模，大小）
        {
            Eigen::Vector3d ba_sum = Eigen::Vector3d::Zero();//::Zero() 是 Eigen::Vector3d 类的一个静态成员函数，用于创建一个零向量，即所有分量均为零的向量。
            Eigen::Vector3d bg_sum = Eigen::Vector3d::Zero();

            assert(v_acc_static.size() == v_gyr_static.size());//assert 是 C++ 中的一个宏，用于在运行时检查条件是否为真.如果条件为假，assert 会中止程序执行，并在标准错误流中输出一条错误消息

            for (int i = 0; i < v_acc_static.size(); i++)
            {
                ba_sum += v_acc_static[i];
                bg_sum += v_gyr_static[i];//从存储的数据（v_acc_static 和 v_gyr_static）中累积加速度计偏置（ba_sum）和陀螺仪偏置（bg_sum）向量
            }

            Eigen::Vector3d ba_avg = ba_sum / v_acc_static.size();
            Eigen::Vector3d bg_avg = bg_sum / v_gyr_static.size();//通过将累积的和除以数据点的数量，计算出平均加速度计偏置（ba_avg）和陀螺仪偏置（bg_avg）

            for (int i = 0; i < all_cloud_frame.size(); i++)//对于 all_cloud_frame 中的每个 cloudFrame，它更新各种状态变量
            {
                all_cloud_frame[i]->p_state->velocity = Eigen::Vector3d::Zero();//将速度设置为零
                all_cloud_frame[i]->p_state->ba = ba_avg;//将加速度计偏置（ba）和陀螺仪偏置（bg）设置为计算出的平均值
                all_cloud_frame[i]->p_state->bg = bg_avg;
                all_cloud_frame[i]->p_state->velocity_begin = Eigen::Vector3d::Zero();//更新初始速度、加速度计偏置和陀螺仪偏置
                all_cloud_frame[i]->p_state->ba_begin = ba_avg;
                all_cloud_frame[i]->p_state->bg_begin = bg_avg;
                all_cloud_frame[i]->p_state->pre_integration->repropagate(all_cloud_frame[i]->p_state->ba, all_cloud_frame[i]->p_state->bg);//在预积分对象上调用 repropagate 函数。
            }

            initial_flag = true;

            std::cout << "gravity vector g = " << G.transpose() << std::endl;
            std::cout << "gyr bias bg = " << all_cloud_frame.back()->p_state->bg.transpose() << std::endl;
            std::cout << "gyr bias ba = " << all_cloud_frame.back()->p_state->ba.transpose() << std::endl;

            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>().swap(v_acc_static);//清空 v_acc_static 向量
            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>().swap(v_gyr_static);
        }
        else if(avg_velocity.norm() < 0.1)
        {
            Eigen::Vector3d g_sum = Eigen::Vector3d::Zero();
            Eigen::Vector3d bg_sum = Eigen::Vector3d::Zero();

            assert(v_acc_static.size() == v_gyr_static.size());

            for (int i = 0; i < v_acc_static.size(); i++)
            {
                g_sum += v_acc_static[i];
                bg_sum += v_gyr_static[i];//累积了一组加速度计偏置（v_acc_static）和陀螺仪偏置（v_gyr_static）的和
            }

            Eigen::Vector3d g_avg = g_sum / v_acc_static.size();
            Eigen::Vector3d bg_avg = bg_sum / v_gyr_static.size();//计算了平均加速度计偏置（ba_avg）和陀螺仪偏置（bg_avg）

            Eigen::Vector3d z_axis = g_avg / g_avg.norm();//z_axis 是 g_avg 的单位向量。
            Eigen::Vector3d e_1(1.0, 0.0, 0.0);
            Eigen::Vector3d x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
            x_axis = x_axis / x_axis.norm();
            Eigen::Vector3d y_axis = numType::skewSymmetric(z_axis) * x_axis;//x_axis 和 y_axis 是与 z_axis 正交的单位向量
            Eigen::Matrix3d Ro;
            Ro.block<3, 1>(0, 0) = x_axis;
            Ro.block<3, 1>(0, 1) = y_axis;
            Ro.block<3, 1>(0, 2) = z_axis;//通过计算单位重力向量 g_avg，构建了一个旋转矩阵 Ro

            Eigen::Vector3d ba_avg = g_avg - Ro * G;
            G = Ro * G;

            for (int i = 0; i < all_cloud_frame.size(); i++)
            {
                all_cloud_frame[i]->p_state->velocity = Eigen::Vector3d::Zero();
                all_cloud_frame[i]->p_state->ba = ba_avg;
                all_cloud_frame[i]->p_state->bg = bg_avg;
                all_cloud_frame[i]->p_state->velocity_begin = Eigen::Vector3d::Zero();
                all_cloud_frame[i]->p_state->ba_begin = ba_avg;
                all_cloud_frame[i]->p_state->bg_begin = bg_avg;
                all_cloud_frame[i]->p_state->pre_integration->repropagate(all_cloud_frame[i]->p_state->ba, all_cloud_frame[i]->p_state->bg);
            }

            initial_flag = true;

            std::cout << "gravity vector g = " << G.transpose() << std::endl;
            std::cout << "gyr bias bg = " << all_cloud_frame.back()->p_state->bg.transpose() << std::endl;
            std::cout << "gyr bias ba = " << all_cloud_frame.back()->p_state->ba.transpose() << std::endl;

            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>().swap(v_acc_static);
            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>().swap(v_gyr_static);
        }
        else
        {
            std::cout << "the hardware platform has moved, static init failed!" << std::endl;
        }
    }
    else
    {
        std::cout << "wait more IMU measurements ..." << std::endl;
    }
}

bool lioOptimization::initialLidarStructure(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v_lidar_pose)
{
    Eigen::Vector3d g;
    Eigen::VectorXd x;
    
    bool result = lidarImuAlignment(v_lidar_pose, g, x);
    if(!result)
    {
        ROS_INFO("solve g failed!");
        return false;
    }

    for (int i = 0; i < all_cloud_frame.size(); i++)
        all_cloud_frame[i]->p_state->velocity = all_cloud_frame[i]->p_state->rotation * x.segment<3>(3 * i);

    g = all_cloud_frame[0]->p_state->rotation.toRotationMatrix() * R_imu_lidar * g;

    G = g;

    std::cout << "after transformation g = " << g.transpose() << std::endl;

    return true;
}

bool lioOptimization::lidarImuAlignment(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v_lidar_pose,
     Eigen::Vector3d &g, Eigen::VectorXd &x)
{
    solveGyroscopeBias(v_lidar_pose);

    bool sucess = linearAlignment(v_lidar_pose, g, x);

    if(sucess)
        return true;
    else 
        return false;
}

void lioOptimization::solveGyroscopeBias(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v_lidar_pose)
{
    Eigen::Matrix3d A;
    Eigen::Vector3d b;
    Eigen::Vector3d delta_bg;
    A.setZero();
    b.setZero();

    assert(v_lidar_pose.size() == all_cloud_frame.size());

    for (int i = 0; i < v_lidar_pose.size() - 1; i++)
    {
        Eigen::Matrix3d temp_A = Eigen::Matrix3d::Zero();
        Eigen::Vector3d temp_b = Eigen::Vector3d::Zero();

        Eigen::Matrix3d current_rot = v_lidar_pose[i].block<3, 3>(0, 0);
        Eigen::Matrix3d next_rot = v_lidar_pose[i + 1].block<3, 3>(0, 0);

        Eigen::Quaterniond quat(current_rot.transpose() * next_rot);
        temp_A = all_cloud_frame[i + 1]->p_state->pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
        temp_b = 2 * (all_cloud_frame[i + 1]->p_state->pre_integration->delta_q.inverse() * quat).vec();

        A += temp_A.transpose() * temp_A;
        b += temp_A.transpose() * temp_b;
    }

    delta_bg = A.ldlt().solve(b);
    std::cout << "gyroscope bias initial calibration " << delta_bg.transpose() << std::endl;
    
    for (int i = 0; i < all_cloud_frame.size(); i++)
    {
        all_cloud_frame[i]->p_state->bg += delta_bg;
        all_cloud_frame[i]->p_state->pre_integration->repropagate(all_cloud_frame[i]->p_state->ba, all_cloud_frame[i]->p_state->bg);
    }       
}

bool lioOptimization::linearAlignment(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v_lidar_pose, 
        Eigen::Vector3d &g, Eigen::VectorXd &x)
{
    assert(v_lidar_pose.size() == all_cloud_frame.size());
// 确保 LiDAR 位姿和点云帧数量相匹配
    int n_state = all_cloud_frame.size() * 3 + 3;
// 计算状态变量的数量
    Eigen::MatrixXd A{n_state, n_state};
    Eigen::VectorXd b{n_state};
    A.setZero();
    b.setZero();// 初始化线性方程组的系数矩阵和常数向量，并初始化为零

    for (int i = 0; i < all_cloud_frame.size() - 1; i++)// 遍历所有 LiDAR 时间点，对于每对相邻的点执行以下操作
    {
        Eigen::Matrix<double, 6, 9> temp_A = Eigen::MatrixXd::Zero(6, 9);
        Eigen::Matrix<double, 6, 1> temp_b = Eigen::MatrixXd::Zero(6, 1);// 初始化临时系数矩阵和常数向量，并初始化为零

        double dt = all_cloud_frame[i + 1]->p_state->pre_integration->sum_dt;// 计算两帧之间的时间间隔

        temp_A.block<3, 3>(0, 0) = - dt * Eigen::Matrix3d::Identity();
        temp_A.block<3, 3>(0, 6) = v_lidar_pose[i].block<3, 3>(0, 0).transpose() * dt * dt / 2;
        temp_b.block<3, 1>(0, 0) = all_cloud_frame[i + 1]->p_state->pre_integration->delta_p + v_lidar_pose[i].block<3, 3>(0, 0).transpose() * v_lidar_pose[i + 1].block<3, 3>(0, 0) * t_imu_lidar - t_imu_lidar
                                   - v_lidar_pose[i].block<3, 3>(0, 0).transpose() * (v_lidar_pose[i + 1].block<3, 1>(0, 3) - v_lidar_pose[i].block<3, 1>(0, 3));
 
        temp_A.block<3, 3>(3, 0) = - Eigen::Matrix3d::Identity();
        temp_A.block<3, 3>(3, 3) = v_lidar_pose[i].block<3, 3>(0, 0).transpose() * v_lidar_pose[i + 1].block<3, 3>(0, 0);
        temp_A.block<3, 3>(3, 6) = v_lidar_pose[i].block<3, 3>(0, 0).transpose() * dt;
        temp_b.block<3, 1>(3, 0) = all_cloud_frame[i + 1]->p_state->pre_integration->delta_v;
// 计算位置和速度变化的线性关系，构建线性方程组的系数矩阵和常数向量
        Eigen::Matrix<double, 6, 6> con_inv = Eigen::MatrixXd::Identity(6, 6);

        Eigen::MatrixXd r_A = temp_A.transpose() * con_inv * temp_A;
        Eigen::VectorXd r_b = temp_A.transpose() * con_inv * temp_b;

        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
        b.segment<6>(i * 3) += r_b.head<6>();

        A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
        b.tail<3>() += r_b.tail<3>();

        A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
        A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
    } // 使用加权的最小二乘方法计算线性方程组的解，并将结果累积到全局的系数矩阵和常数向量中

    A = A * 1000.0;
    b = b * 1000.0;// 放大系数矩阵和常数向量以确保数值稳定性
    x = A.ldlt().solve(b);
    g = x.segment<3>(n_state - 3);// 使用 LDLT 分解方法求解线性方程组，得到重力校准的更新量

    if(fabs(g.norm() - G.norm()) > 1.0)
    {
        return false;//如果更新后的重力向量与预期的重力向量差异超过一定阈值，则返回 false

    }//这段代码实现了线性对齐的过程

    refineGravity(v_lidar_pose, g, x);
    std::cout << "refine " << g.norm() << " " << g.transpose() << std::endl;

    return true;
}// 对重力向量进行进一步优化

Eigen::Matrix<double, 3, 2> lioOptimization::tangentBasis(Eigen::Vector3d &g0)
{
    Eigen::Vector3d b, c;
    Eigen::Vector3d a = g0.normalized();
    Eigen::Vector3d temp(0, 0, 1);
    if(a == temp)
        temp << 1, 0, 0;
    b = (temp - a * (a.transpose() * temp)).normalized();
    c = a.cross(b);
    Eigen::Matrix<double, 3, 2> bc;
    bc.block<3, 1>(0, 0) = b;
    bc.block<3, 1>(0, 1) = c;
    return bc;
}// 计算重力向量的切空间基向量

void lioOptimization::refineGravity(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v_lidar_pose, 
        Eigen::Vector3d &g, Eigen::VectorXd &x)// LiDAR 位姿,重力向量，存储结果
{
    Eigen::Vector3d g0 = g.normalized() * G.norm();//重力g归一化，乘以重力常数
    Eigen::Vector3d lx, ly;//存储切空间的两个基向量

    int n_state = all_cloud_frame.size() * 3 + 2;//计算状态变量的数量，这个数量包括每个 LiDAR 时间点的位姿变量（旋转和平移，共 6 个自由度）和两个额外的重力校准参数

    Eigen::MatrixXd A{n_state, n_state};
    Eigen::VectorXd b{n_state};
    A.setZero();
    b.setZero();//初始化为0

    for (int k = 0; k < 4; k++)//执行四次迭代以优化重力向量
    {
        Eigen::Matrix<double, 3, 2> lxly = tangentBasis(g0);//计算切空间的两个基向量，并将结果存储

        for (int i = 0; i < all_cloud_frame.size() - 1; i++)//遍历 LiDAR 时间点，对于每对相邻的时间点执行下面的操作
        {
            Eigen::Matrix<double, 6, 8> temp_A = Eigen::MatrixXd::Zero(6, 8);//6x8矩阵，元素类型double，初始化为0.通常用于存储一个线性方程组的系数矩阵，其中每行代表一个方程，每列代表一个未知数的系数。在这段代码中，该矩阵用于存储求解重力校准的线性方程组的系数
            Eigen::Matrix<double, 6, 1> temp_b = Eigen::MatrixXd::Zero(6, 1);//用于存储线性方程组的系数和常数项。

            double dt = all_cloud_frame[i + 1]->p_state->pre_integration->sum_dt;//提取两个时间点之间的时间间隔。访问下一个点云帧状态信息中的预积分信息中的时间间隔

            temp_A.block<3, 3>(0, 0) = - dt * Eigen::Matrix3d::Identity();
            temp_A.block<3, 2>(0, 6) = v_lidar_pose[i].block<3, 3>(0, 0).transpose() * dt * dt / 2 * lxly;//第一部分，位置变化的线性关系。当前点云帧的旋转矩阵，转置，乘以时间间隔的平方，除以2

            temp_b.block<3, 1>(0, 0) = all_cloud_frame[i + 1]->p_state->pre_integration->delta_p + v_lidar_pose[i].block<3, 3>(0, 0).transpose() * v_lidar_pose[i + 1].block<3, 3>(0, 0) * t_imu_lidar - t_imu_lidar 
                                     - v_lidar_pose[i].block<3, 3>(0, 0).transpose() * dt * dt / 2 * g0 - v_lidar_pose[i].block<3, 3>(0, 0).transpose() * (v_lidar_pose[i + 1].block<3, 1>(0, 3) - v_lidar_pose[i].block<3, 1>(0, 3));//计算位置变化的常数项
//temp_b(0:2, 0) = delta_p + R_i^T * R_{i+1} * t_imu_lidar - t_imu_lidar - R_i^T * (Delta_t^2 / 2) * g_0 - R_i^T * (p_{i+1} - p_i)。delta_p 表示下一个点云帧中的预积分对象的位置增量。R_i 表示当前点云帧的旋转矩阵。R_{i+1} 表示下一个点云帧的旋转矩阵。t_imu_lidar 表示 IMU 和 LiDAR 之间的平移变换矢量。Delta_t 表示当前点云帧和下一个点云帧之间的时间间隔。g_0 表示归一化的重力向量。p_i 表示当前点云帧的平移向量。p_{i+1} 表示下一个点云帧的平移向量
//这个表达式包括了预积分位置增量、两帧之间的相对位姿变换、重力对位移的影响以及两帧之间的平移变化，用于优化过程中的残差计算。           
            temp_A.block<3, 3>(3, 0) = - Eigen::Matrix3d::Identity();
            temp_A.block<3, 3>(3, 3) = v_lidar_pose[i].block<3, 3>(0, 0).transpose() * v_lidar_pose[i + 1].block<3, 3>(0, 0);
            temp_A.block<3, 2>(3, 6) = v_lidar_pose[i].block<3, 3>(0, 0).transpose() * dt * lxly;//第二部分，表示速度变化的线性关系
            temp_b.block<3, 1>(3, 0) = all_cloud_frame[i + 1]->p_state->pre_integration->delta_v - v_lidar_pose[i].block<3, 3>(0, 0).transpose() * dt * g0;

            Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Identity();

            Eigen::Matrix<double, 8, 8> r_A = temp_A.transpose() * cov_inv * temp_A;
            Eigen::Matrix<double, 8, 1> r_b = temp_A.transpose() * cov_inv * temp_b;//计算加权的最小二乘解

            A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();//左上角6x6
            b.segment<6>(i * 3) += r_b.head<6>();前6个元素

            A.bottomRightCorner<2, 2>() += r_A.bottomRightCorner<2, 2>();//右下角2x2加到A右下角
            b.tail<2>() += r_b.tail<2>();//最后2个元素

            A.block<6, 2>(i * 3, n_state - 2) += r_A.topRightCorner<6, 2>();
            A.block<2, 6>(n_state - 2, i * 3) += r_A.bottomLeftCorner<2, 6>();//将加权的最小二乘解添加到A和b
        }

        A = A * 1000.0;
        b = b * 1000.0;//放大数值以确保数值稳定性
        x = A.ldlt().solve(b);//使用 LDLT 分解求解线性方程组
        Eigen::VectorXd dg = x.segment<2>(n_state - 2);
        g0 = (g0 + lxly * dg).normalized() * G.norm();
    }
    g = g0;
}//这段代码的目的是通过最小化重力校准参数，以最佳化 LiDAR 的位置和速度变化，从而提高定位系统的精度和稳定性。