#include "utility.h"
#include "imuProcessing.h"

bool debug_output = false;
std::string output_path = "";//字符串变量，存储输出文件或目录的路径

bool time_diff_enable = false;//时间差功能
double time_diff = 0.0;

float mov_threshold = 1.5f;//移动阈值

bool initial_flag = false;//某个过程是否已经初始化

Eigen::Vector3d G;//三维向量

bool time_list(point3D &point_1, point3D &point_2)
{
	return (point_1.relative_time < point_2.relative_time);
};

bool time_list_velodyne(velodyne_ros::Point &point_1, velodyne_ros::Point &point_2)
{
    return (point_1.time < point_2.time);
}

bool time_list_robosense(robosense_ros::Point &point_1, robosense_ros::Point &point_2)
{
     return (point_1.timestamp < point_2.timestamp);
}

bool time_list_ouster(ouster_ros::Point &point_1, ouster_ros::Point &point_2)
{
     return (point_1.t < point_2.t);
}

void point3DtoPCL(std::vector<point3D> &v_point_temp, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &p_cloud_temp)//智能指针
{
    for(int i = 0; i < v_point_temp.size(); i++)
    {
        pcl::PointXYZINormal cloud_temp;
        cloud_temp.x = v_point_temp[i].raw_point.x();
        cloud_temp.y = v_point_temp[i].raw_point.y();
        cloud_temp.z = v_point_temp[i].raw_point.z();
        cloud_temp.normal_x = 0;
        cloud_temp.normal_y = 0;
        cloud_temp.normal_z = 0;
        cloud_temp.intensity = v_point_temp[i].alpha_time;
        cloud_temp.curvature = v_point_temp[i].relative_time;

        p_cloud_temp->points.push_back(cloud_temp);
    }
}//用于将point3D类型的点云转换为pcl::PointCloud<pcl::PointXYZINormal>类型的点云

Eigen::Matrix3d mat33FromArray(std::vector<double> &array)
{
	assert(array.size() == 9);
	Eigen::Matrix3d mat33;
	mat33(0, 0) = array[0]; mat33(0, 1) = array[1]; mat33(0, 2) = array[2];
	mat33(1, 0) = array[3]; mat33(1, 1) = array[4]; mat33(1, 2) = array[5];
	mat33(2, 0) = array[6]; mat33(2, 1) = array[7]; mat33(2, 2) = array[8];

	return mat33;
}将包含9个元素的向量转换为一个33矩阵

Eigen::Vector3d vec3FromArray(std::vector<double> &array)//类，三维向量，动态数组
{
	assert(array.size() == 3);
	Eigen::Vector3d vec3;
	vec3(0, 0) = array[0]; vec3(1, 0) = array[1]; vec3(2, 0) = array[2];

	return vec3;
}

void pointBodyToWorld(pcl::PointXYZINormal const * const pi, pcl::PointXYZINormal * const po, Eigen::Matrix3d &R_world_cur, 
	Eigen::Vector3d &t_world_cur, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar)
{
    Eigen::Vector3d point_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_global(R_world_cur * (R_imu_lidar * point_body + t_imu_lidar) + t_world_cur);
//将 point_body 从 IMU/Lidar 坐标系转换到世界坐标系
    po->x = point_global(0);
    po->y = point_global(1);
    po->z = point_global(2);
    po->intensity = pi->intensity;
}//用于将输入点从车体坐标系转换到世界坐标系，并将结果存储在输出点中。非常重要！！！！！！

void RGBpointLidarToIMU(pcl::PointXYZINormal const * const pi, pcl::PointXYZINormal * const po, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar)//imu到lidar的矩阵和平移向量
{
    Eigen::Vector3d pt_lidar(pi->x, pi->y, pi->z);
    Eigen::Vector3d pt_imu = R_imu_lidar * pt_lidar + t_imu_lidar;

    po->x = pt_imu(0, 0);
    po->y = pt_imu(1, 0);
    po->z = pt_imu(2, 0);
    po->intensity = pi->intensity;// pi->intensity：这是一个指针 pi 所指向的对象的成员访问操作。pi 是一个指向 pcl::PointXYZINormal 类型的常量指针。 intensity 是 pcl::PointXYZINormal 类型对象的一个成员变量。通过 pi->intensity，我们可以获取 pi 指向的对象的 intensity 成员的值。总之，箭头符号 -> 用于通过指针访问对象的成员
}//将输入点从激光雷达坐标系转换到 IMU 坐标系

bool planeFitting(Eigen::Matrix<double, 4, 1> &plane_parameter, const std::vector<pcl::PointXYZINormal, Eigen::aligned_allocator<pcl::PointXYZINormal>> &point, const double &threshold)
{
    Eigen::Matrix<double, NUM_MATCH_POINTS, 3> A;//行，3列，一组三维点坐标
    Eigen::Matrix<double, NUM_MATCH_POINTS, 1> b;//行，1列，一组标量值
    A.setZero();
    b.setOnes();
    b *= -1.0f;//乘以

    for (int i = 0; i < NUM_MATCH_POINTS; i++)
    {
        A(i, 0) = point[i].x;
        A(i, 1) = point[i].y;
        A(i, 2) = point[i].z;
    }

    Eigen::Matrix<double, 3, 1> norm_vector = A.colPivHouseholderQr().solve(b);

    double n = norm_vector.norm();
    plane_parameter(0) = norm_vector(0, 0) / n;
    plane_parameter(1) = norm_vector(1, 0) / n;
    plane_parameter(2) = norm_vector(2, 0) / n;
    plane_parameter(3) = 1.0 / n;

    for (int i = 0; i < NUM_MATCH_POINTS; i++)
    {
        if (fabs(plane_parameter(0, 0) * point[i].x + plane_parameter(1, 0) * point[i].y + plane_parameter(2, 0) * point[i].z + plane_parameter(3, 0)) > threshold)
            return false;
    }

    return true;
}

double AngularDistance(const Eigen::Matrix3d &rota, const Eigen::Matrix3d &rotb)
{
    double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
    norm = std::acos(norm) * 180 / M_PI;
    return norm;
}

double AngularDistance(const Eigen::Quaterniond &q_a, const Eigen::Quaterniond &q_b)
{
    Eigen::Matrix3d rota = q_a.toRotationMatrix();
    Eigen::Matrix3d rotb = q_b.toRotationMatrix();
    
    double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
    norm = std::acos(norm) * 180 / M_PI;
    return norm;
}

float calculateDist2(pcl::PointXYZINormal point1, pcl::PointXYZINormal point2)
{
    float d = (point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y) + (point1.z - point2.z) * (point1.z - point2.z);
    return d;
}

void saveCutCloud(std::string &str, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &p_cloud_temp)
{
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(str, *p_cloud_temp);
}//点到面ICP核心内容。非常重要

void subSampleFrame(std::vector<point3D> &frame, double size_voxel)//网格采样的体素大小
{
    std::tr1::unordered_map<voxel, std::vector<point3D>, std::hash<voxel>> grid;//键的类型网格单元的索引，值的类型在网格中存储的点的集合，哈希函数计算键的哈希值。创建一个哈希表 grid，用于存储网格中的点。哈希表的键是网格单元的索引，值是在该网格单元中的点的集合。
    for (int i = 0; i < (int) frame.size(); i++) {
        auto kx = static_cast<short>(frame[i].point[0] / size_voxel);//x坐标除以体素大小，显式类型转换。关键字，在不指定数据类型情况下生命变量
        auto ky = static_cast<short>(frame[i].point[1] / size_voxel);
        auto kz = static_cast<short>(frame[i].point[2] / size_voxel);
        grid[voxel(kx, ky, kz)].push_back(frame[i]);//网格单元的索引，键来访问网格单元。将点 frame[i] 添加到哈希表 grid 中的特定网格单元
    }
    frame.resize(0);
    int step = 0;
    for(const auto &n: grid)//用于遍历哈希表 grid 中的每个元素，并将每个元素的引用存储在循环变量 n 中。在循环体内，我们可以使用 n 来访问哈希表中的元素
    {
        if(n.second.size() > 0)//n 是哈希表中的一个元素，n.second 表示该元素的值，即存储在网格单元中的点的集合
        {
            frame.push_back(n.second[0]);
            step++;
        }//对输入的三维点云进行网格采样，以获取稀疏的关键点
    }
}

void gridSampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling)//采样后的关键点，网格采样的体素大小
{
    keypoints.resize(0);
    std::vector<point3D> frame_sub;//临时点云
    frame_sub.resize(frame.size());
    for (int i = 0; i < (int) frame_sub.size(); i++) {
        frame_sub[i] = frame[i];
    }
    subSampleFrame(frame_sub, size_voxel_subsampling);
    keypoints.reserve(frame_sub.size());
    for (int i = 0; i < (int) frame_sub.size(); i++) {
        keypoints.push_back(frame_sub[i]);
    }
}

void distortFrameUsingConstant(std::vector<point3D> &points, Eigen::Quaterniond &q_begin, Eigen::Quaterniond &q_end, Eigen::Vector3d &t_begin, Eigen::Vector3d &t_end, 
    Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar)
{
    Eigen::Quaterniond q_end_inv = q_end.inverse(); // Rotation of the inverse pose
    Eigen::Vector3d t_end_inv = -1.0 * (q_end_inv * t_end); // Translation of the inverse pose
    for (auto &point_temp: points)
    {
        double alpha_time = point_temp.alpha_time;
        Eigen::Quaterniond q_alpha = q_begin.slerp(alpha_time, q_end);
        q_alpha.normalize();
        Eigen::Vector3d t_alpha = (1.0 - alpha_time) * t_begin + alpha_time * t_end;

        point_temp.raw_point = R_imu_lidar.transpose() * (q_end_inv * (q_alpha * (R_imu_lidar * point_temp.raw_point + t_imu_lidar) + t_alpha) + t_end_inv) - R_imu_lidar.transpose() * t_imu_lidar;
    }
}

void distortFrameUsingImu(std::vector<point3D> &points, state* cur_state, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar)
{
    int n = 0;
    double time_imu_begin;
    double time_imu_end;
    Eigen::Quaterniond q_imu_begin;
    Eigen::Quaterniond q_imu_end;
    Eigen::Vector3d t_imu_begin;
    Eigen::Vector3d t_imu_end;

    Eigen::Quaterniond q_end_inv = cur_state->rotation.inverse();
    Eigen::Vector3d t_end_inv = -1.0 * (q_end_inv * cur_state->translation);

    std::vector<point3D>::iterator iter = points.begin();

    while (iter != points.end())
    {
        for (int n = 0; n + 1 < cur_state->dt_buf.size(); n++)
        {
            time_imu_begin = cur_state->dt_buf[n] * 1000.0;
            time_imu_end = cur_state->dt_buf[n + 1] * 1000.0;

            if ((*iter).relative_time > time_imu_begin && (*iter).relative_time < time_imu_end) 
            {
                q_imu_begin = cur_state->rot_buf[n];
                q_imu_end = cur_state->rot_buf[n + 1];
                t_imu_begin = cur_state->trans_buf[n];
                t_imu_end = cur_state->trans_buf[n + 1];
                break;
            }
            else n++;
        }

        double alpha_time = ((*iter).relative_time - time_imu_begin) / (time_imu_end - time_imu_begin);
        Eigen::Quaterniond q_alpha = q_imu_begin.slerp(alpha_time, q_imu_end);
        Eigen::Vector3d t_alpha = (1.0 - alpha_time) * t_imu_begin + alpha_time * t_imu_end;

        (*iter).raw_point = R_imu_lidar.transpose() * (q_end_inv * (q_alpha * (R_imu_lidar * (*iter).raw_point + t_imu_lidar) + t_alpha) + t_end_inv) - R_imu_lidar.transpose() * t_imu_lidar;

        iter++;
    }
}

void transformPoint(MotionCompensation compensation, point3D &point_temp, Eigen::Quaterniond &q_begin, Eigen::Quaterniond &q_end, 
    Eigen::Vector3d &t_begin, Eigen::Vector3d &t_end, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar)
{
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    double alpha_time = point_temp.alpha_time;
    switch (compensation) {
        case MotionCompensation::NONE:
        case MotionCompensation::CONSTANT_VELOCITY:
            R = q_end.toRotationMatrix();
            t = t_end;
            break;
        case MotionCompensation::CONTINUOUS:
        case MotionCompensation::ITERATIVE:
            R = q_begin.slerp(alpha_time, q_end).normalized().toRotationMatrix();
            t = (1.0 - alpha_time) * t_begin + alpha_time * t_end;
            break;
        case MotionCompensation::IMU:
            R = q_end.toRotationMatrix();
            t = t_end;
            break;
    }
    point_temp.point = R * (R_imu_lidar * point_temp.raw_point + t_imu_lidar) + t;
}
