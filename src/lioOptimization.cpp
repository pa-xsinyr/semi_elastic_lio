#include "lioOptimization.h"

cloudFrame::cloudFrame(std::vector<point3D> &point_frame_, std::vector<point3D> &const_frame_, state *p_state_)//指向state对象的指针
{//cloudFrame 类的对象通常用于存储点云数据和状态信息。point_frame 和 const_frame 是存储点云数据的向量。p_state 是指向状态信息的指针。
    point_frame.insert(point_frame.end(), point_frame_.begin(), point_frame_.end());//将 point_frame_ 中的元素插入到 point_frame 向量的末尾
    const_frame.insert(const_frame.end(), const_frame_.begin(), const_frame_.end());

    p_state = p_state_;//将 p_state_ 指针赋值给 p_state 成员变量

    success = true;
}//请注意，这段代码只是构造函数的定义，实际使用时需要在其他地方创建 cloudFrame 对象并传递相应的参数

cloudFrame::cloudFrame(cloudFrame *p_cloud_frame)//类的构造函数。构造函数是特殊的成员函数，用于初始化类的对象。接受一个指向 cloudFrame 对象的指针作为参数。
{
    time_sweep_begin = p_cloud_frame->time_sweep_begin;//成员变量赋值，复制时间范围的起始和结束时间
    time_sweep_end = p_cloud_frame->time_sweep_end;

    id = p_cloud_frame->id;
    frame_id = p_cloud_frame->frame_id;// 复制帧的ID和frame_id

    p_state = p_cloud_frame->p_state;// 复制指向点云帧状态的指针

    point_frame.insert(point_frame.end(), p_cloud_frame->point_frame.begin(), p_cloud_frame->point_frame.end());//将点云数据从输入点云帧插入到当前点云帧中。insert函数
    const_frame.insert(const_frame.end(), p_cloud_frame->const_frame.begin(), p_cloud_frame->const_frame.end());

    offset_begin = p_cloud_frame->offset_begin;
    offset_end = p_cloud_frame->offset_end;
    dt_offset = p_cloud_frame->dt_offset;// 复制偏移量和时间偏移量

    success = p_cloud_frame->success;// 复制帧是否成功的标志
}//定义了类的拷贝构造函数，创建新的对象

void cloudFrame::release()//类的release()方法，用于释放该点云帧对象所占用的内存
{
    std::vector<point3D>().swap(point_frame);
    std::vector<point3D>().swap(const_frame);// 释放 point_frame 和 const_frame 中的内存

    if(p_state != nullptr)
        p_state->release();// 如果 p_state 指针不为空，则释放其内存

    delete p_state;// 删除 p_state 指针所指向的内存

    p_state = nullptr;// 将 p_state 指针设置为 nullptr，确保不再指向任何内存
}

estimationSummary::estimationSummary()
{

}

void estimationSummary::release()//定义了类的方法，用于释放估计摘要对象所占用的内存
{
    if(!state_frame) state_frame->release();// 检查 state_frame 指针是否为空，如果不为空，则调用其 release() 方法释放内存

    std::vector<point3D>().swap(corrected_points);// 释放 corrected_points 向量的内存，并将其容量置为零

    std::vector<point3D>().swap(all_corrected_points);// 释放 all_corrected_points 向量的内存，并将其容量置为零

    std::vector<point3D>().swap(keypoints);// 释放 keypoints 向量的内存，并将其容量置为零
}

lioOptimization::lioOptimization()
{
	allocateMemory();//分配内存

    readParameters();//读取参数

    initialValue();//初始化数值

    pub_cloud_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_current", 2);//在 ROS（机器人操作系统）中创建一个发布者，将 PointCloud2 数据发布到主题 “/cloud_registered_current” 上。这个发布者可以用于向其他对此数据感兴趣的 ROS 节点或订阅者发送 PointCloud2 消息
//nh 可能是 ROS 的 ros::NodeHandle 类的一个实例，它提供了与 ROS 功能的接口。 advertise 是 ros::NodeHandle 类提供的一个方法/函数，用于在特定主题上创建一个发布者。<sensor_msgs::PointCloud2> 指定将在该主题上发布的消息类型。在这种情况下，它是一个 PointCloud2 消息，表示 3D 点云数据
//发布者的主题名称是 “/cloud_registered_current”。这意味着 PointCloud2 数据将发布在名为 “/cloud_registered_current” 的主题上。 
//advertise 的第二个参数是队列大小（在这里是 2）。它决定了可以排队多少条消息，超过这个数量的消息将被丢弃。如果队列已满，新消息将替换最旧的消息
    pub_cloud_world = nh.advertise<sensor_msgs::PointCloud2>("/cloud_global_map", 2);
    pub_odom = nh.advertise<nav_msgs::Odometry>("/Odometry_after_opt", 5);
    pub_path = nh.advertise<nav_msgs::Path>("/path", 5);

    sub_cloud_ori = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 20, &lioOptimization::standardCloudHandler, this);
    sub_imu_ori = nh.subscribe<sensor_msgs::Imu>(imu_topic, 500, &lioOptimization::imuHandler, this);
//初始化ROS订阅者
    path.header.stamp = ros::Time::now();//path 是一个变量，可能表示一个 ROS 消息，其类型为 nav_msgs::Path。通常，它包含一系列 geometry_msgs::PoseStamped 消息，这些消息表示路径上的位姿（位置和方向）。
    //header 是 path 消息的一个成员，它表示消息的头部信息，包括时间戳、坐标系 ID 和其他元数据，header 的类型是 std_msgs::Header
    //stamp 是 header 的一个成员，它表示与消息关联的时间戳，在这行代码中，ros::Time::now() 用于将时间戳设置为当前时间，返回当前时间作为 ros::Time 对象
    path.header.frame_id ="camera_init";//这行代码将 path 消息的坐标系 ID（frame ID）设置为 “camera_init”。坐标系 ID 对于在特定坐标系中正确解释路径数据非常重要
 //初始化路径消息
    points_world.reset(new pcl::PointCloud<pcl::PointXYZI>());
//创建一个新的点云对象，表示世界坐标系下的点云
    //options.recordParameters();
}

void lioOptimization::readParameters()//定义类的一个方法。从ros参数服务器中读取参数
{	
    int para_int;
    double para_double;
    bool para_bool;
    std::string str_temp;

    // common
    nh.param<std::string>("common/lidar_topic", lidar_topic, "/points_raw");//激光雷达话题名称
	nh.param<std::string>("common/imu_topic", imu_topic, "/imu_raw");
    nh.param<int>("common/point_filter_num", para_int, 1);  cloud_pro->setPointFilterNum(para_int);//设置点云过滤器的参数
    nh.param<std::vector<double>>("common/gravity_acc", v_G, std::vector<double>());//重力加速度的参数（gravity_acc）
    nh.param<bool>("debug_output", debug_output, false);//调试输出的开关（debug_output）
    nh.param<std::string>("output_path", output_path, "");//输出路径（output_path）

    // LiDAR parameter
    nh.param<int>("lidar_parameter/lidar_type", para_int, AVIA);  cloud_pro->setLidarType(para_int);// 从 ROS 参数服务器中读取激光雷达类型参数，如果没有找到，则使用默认值 AVIA，并将读取的值存储在 para_int 中。// 调用 cloud_pro 对象的 setLidarType() 方法，将之前读取的激光雷达类型参数值设置到相应的成员变量中
    nh.param<int>("lidar_parameter/N_SCANS", para_int, 16);  cloud_pro->setNumScans(para_int);//扫描线数参数
    nh.param<int>("lidar_parameter/SCAN_RATE", para_int, 10);  cloud_pro->setScanRate(para_int);//扫描速率参数
    nh.param<int>("lidar_parameter/time_unit", para_int, US);  cloud_pro->setTimeUnit(para_int);//时间单位参数
    nh.param<double>("lidar_parameter/blind", para_double, 0.01);  cloud_pro->setBlind(para_double);//盲区大小参数
    nh.param<float>("lidar_parameter/det_range", det_range, 300.f);//激光雷达探测范围参数
    nh.param<double>("lidar_parameter/fov_degree", fov_deg, 180);//激光雷达视场角参数

    // IMU parameter
    nh.param<double>("imu_parameter/acc_cov", para_double, 0.1);  imu_pro->setAccCov(para_double);//加速度测量噪声方差
    nh.param<double>("imu_parameter/gyr_cov", para_double, 0.1);  imu_pro->setGyrCov(para_double);//陀螺仪测量噪声方差
    nh.param<double>("imu_parameter/b_acc_cov", para_double, 0.0001);  imu_pro->setBiasAccCov(para_double);//加速度偏置测量噪声方差
    nh.param<double>("imu_parameter/b_gyr_cov", para_double, 0.0001);  imu_pro->setBiasGyrCov(para_double);//陀螺仪偏置测量噪声
    nh.param<bool>("imu_parameter/time_diff_enable", time_diff_enable, false);//时间差分数据是否启用

    // extrinsic parameter
    nh.param<bool>("extrinsic_parameter/extrinsic_enable", extrin_enable, true);//外部参数是否启用
    nh.param<std::vector<double>>("extrinsic_parameter/extrinsic_t", v_extrin_t, std::vector<double>());//外部参数的平移向量参数
    nh.param<std::vector<double>>("extrinsic_parameter/extrinsic_R", v_extrin_R, std::vector<double>());//外部参数的旋转矩阵参数

    // new ct-icp
    nh.param<double>("odometry_options/init_voxel_size", options.init_voxel_size, 0.2);//初始化体素
    nh.param<double>("odometry_options/init_sample_voxel_size", options.init_sample_voxel_size, 1.0)//初采样体素大小;
    nh.param<int>("odometry_options/init_num_frames", options.init_num_frames, 20);//初始帧数
    nh.param<double>("odometry_options/voxel_size", options.voxel_size, 0.5);//
    nh.param<double>("odometry_options/sample_voxel_size", options.sample_voxel_size, 1.5);
    nh.param<double>("odometry_options/max_distance", options.max_distance, 100.0);
    nh.param<int>("odometry_options/max_num_points_in_voxel", options.max_num_points_in_voxel, 20);
    nh.param<double>("odometry_options/min_distance_points", options.min_distance_points, 0.1);
    nh.param<double>("odometry_options/distance_error_threshold", options.distance_error_threshold, 5.0);
    nh.param<int>("odometry_options/robust_minimal_level", options.robust_minimal_level, 0);//控制鲁棒里程计算法的最低级别，影响算法的精度或计算速度
    nh.param<bool>("odometry_options/robust_registration", options.robust_registration, false);//布尔参数，启动或禁用鲁棒的配准
    nh.param<double>("odometry_options/robust_full_voxel_threshold", options.robust_full_voxel_threshold, 0.7);//设置鲁棒体素阈值。体素是三维空间中的小立方体单元
    nh.param<double>("odometry_options/robust_empty_voxel_threshold", options.robust_empty_voxel_threshold, 0.1);//空体素的阈值
    nh.param<double>("odometry_options/robust_neighborhood_min_dist", options.robust_neighborhood_min_dist, 0.10);//控制着鲁棒的领域最小距离。在点云匹配中，领域通常用于搜索最近邻点
    nh.param<double>("odometry_options/robust_neighborhood_min_orientation", options.robust_neighborhood_min_orientation, 0.1);//领域的最小方向
    nh.param<double>("odometry_options/robust_relative_trans_threshold", options.robust_relative_trans_threshold, 1.0);//这个参数可能与相对位姿变换的阈值有关。在机器人定位中，相对位姿变换通常表示两个连续帧之间的相对运动
    nh.param<bool>("odometry_options/robust_fail_early", options.robust_fail_early, false);//这是一个布尔参数，如果设置为 true，则表示在某些条件下，鲁棒算法会尽早失败。
    nh.param<int>("odometry_options/robust_num_attempts", options.robust_num_attempts, 6);//：这个参数控制着鲁棒算法的尝试次数。在某些情况下，算法可能会多次尝试以获得更好的结果。
    nh.param<int>("odometry_options/robust_num_attempts_when_rotation", options.robust_num_attempts_when_rotation, 2);//类似于上一个参数，但仅在旋转情况下使用。
    nh.param<int>("odometry_options/robust_max_voxel_neighborhood", options.robust_max_voxel_neighborhood, 3);
    nh.param<double>("odometry_options/robust_threshold_ego_orientation", options.robust_threshold_ego_orientation, 3);//可能与机器人自身方向的阈值有关。
    nh.param<double>("odometry_options/robust_threshold_relative_orientation", options.robust_threshold_relative_orientation, 3);//类似于上一个参数，但与相对方向有关。

    nh.param<std::string>("odometry_options/method_system_init", str_temp, "MOTION_INIT");
    if(str_temp == "MOTION_INIT") options.method_system_init = MOTION_INIT;
    else if(str_temp == "STATIC_INIT") options.method_system_init = STATIC_INIT;
    else std::cout << "The `initialization_method` " << str_temp << " is not supported." << std::endl;//该参数可能控制着某个咯i成绩算法或系统的初始化方法

    nh.param<std::string>("odometry_options/motion_compensation", str_temp, "NONE");//与运动补偿有关
    if(str_temp == "NONE") options.motion_compensation = NONE;
    else if(str_temp == "CONSTANT_VELOCITY") options.motion_compensation = CONSTANT_VELOCITY;
    else if(str_temp == "ITERATIVE") options.motion_compensation = ITERATIVE;
    else if(str_temp == "CONTINUOUS") options.motion_compensation = CONTINUOUS;
    else if(str_temp == "IMU") options.motion_compensation = IMU;
    else std::cout << "The `motion_compensation` " << str_temp << " is not supported." << std::endl;

    nh.param<std::string>("odometry_options/initialization", str_temp, "INIT_NONE");//运动初始化
    if(str_temp == "INIT_NONE") options.initialization = INIT_NONE;
    else if(str_temp == "INIT_CONSTANT_VELOCITY") options.initialization = INIT_CONSTANT_VELOCITY;
    else if(str_temp == "INIT_IMU") options.initialization = INIT_IMU;
    else std::cout << "The `state_initialization` " << str_temp << " is not supported." << std::endl;


    icpOptions optimize_options;//这些参数可能控制着在点云处理或传感器融合任务中使用的优化算法的行为。
    nh.param<int>("icp_options/threshold_voxel_occupancy", options.optimize_options.threshold_voxel_occupancy, 1);//体素占用阈值
    nh.param<double>("icp_options/size_voxel_map", options.optimize_options.size_voxel_map, 1.0);//体素地图大小
    nh.param<int>("icp_options/num_iters_icp", options.optimize_options.num_iters_icp, 5);//icp迭代过程的迭代次数
    nh.param<int>("icp_options/min_number_neighbors", options.optimize_options.min_number_neighbors, 20);//icp优化过程中考虑的最小邻居数
    nh.param<int>("icp_options/voxel_neighborhood", options.optimize_options.voxel_neighborhood, 1);//基于体素的操作的邻域大小
    nh.param<double>("icp_options/power_planarity", options.optimize_options.power_planarity, 2.0);//与平面性估计相关的参数。它可能影响在 ICP 过程中如何处理平面表面
    nh.param<bool>("icp_options/estimate_normal_from_neighborhood", options.optimize_options.estimate_normal_from_neighborhood, true);//是否从邻域中估计法线。法线对于点到平面 ICP 非常重要。
    nh.param<int>("icp_options/max_number_neighbors", options.optimize_options.max_number_neighbors, 20);//在 ICP 优化过程中考虑的最大邻居数。
    nh.param<double>("icp_options/max_dist_to_plane_icp", options.optimize_options.max_dist_to_plane_icp, 0.3);//点到平面 ICP 中的最大平面距离
    nh.param<double>("icp_options/threshold_orientation_norm", options.optimize_options.threshold_orientation_norm, 0.0001);//旋转（方向）归一化的阈值。
    nh.param<double>("icp_options/threshold_translation_norm", options.optimize_options.threshold_translation_norm, 0.001);//平移归一化的阈值。
    nh.param<bool>("icp_options/point_to_plane_with_distortion", options.optimize_options.point_to_plane_with_distortion, true);//是否使用带有畸变校正的点到平面 ICP
    nh.param<int>("icp_options/max_num_residuals", options.optimize_options.max_num_residuals, -1);//优化过程中考虑的最大残差（误差）数量。
    nh.param<int>("icp_options/min_num_residuals", options.optimize_options.min_num_residuals, 100);//优化所需的最小残差数量
    nh.param<int>("icp_options/num_closest_neighbors", options.optimize_options.num_closest_neighbors, 1);//在 ICP 中使用的最近邻居数。
    nh.param<double>("icp_options/beta_location_consistency", options.optimize_options.beta_location_consistency, 0.001);//位置一致性的参数
    nh.param<double>("icp_options/beta_constant_velocity", options.optimize_options.beta_constant_velocity, 0.001);//常速度的参数
    nh.param<double>("icp_options/beta_small_velocity", options.optimize_options.beta_small_velocity, 0.0);//小速度
    nh.param<double>("icp_options/beta_orientation_consistency", options.optimize_options.beta_orientation_consistency, 0.0);//方向一致性
    nh.param<double>("icp_options/weight_alpha", options.optimize_options.weight_alpha, 0.9);//用于优化的权重参数
    nh.param<double>("icp_options/weight_neighborhood", options.optimize_options.weight_neighborhood, 0.1);//另一个用于优化的权重参数
    nh.param<int>("icp_options/ls_max_num_iters", options.optimize_options.ls_max_num_iters, 1);//最小二乘优化步骤的最大迭代次数
    nh.param<int>("icp_options/ls_num_threads", options.optimize_options.ls_num_threads, 16);//在最小二乘优化过程中使用的线程数
    nh.param<double>("icp_options/ls_sigma", options.optimize_options.ls_sigma, 0.1);
    nh.param<double>("icp_options/ls_tolerant_min_threshold", options.optimize_options.ls_tolerant_min_threshold, 0.05);//最小二乘优化中的容差阈值。用于确定何时停止优化。
    nh.param<bool>("icp_options/debug_print", options.optimize_options.debug_print, true);//是否启用调试打印。如果设置为 true，执行过程中可能会打印附加信息
    nh.param<bool>("icp_options/debug_viz", options.optimize_options.debug_viz, false);//是否启用调试可视化。如果设置为 true，可能会生成附加的可视化数据

    nh.param<std::string>("icp_options/distance", str_temp, "CT_POINT_TO_PLANE");//ROS类的一个对象，处理与节点的通信。ros::NodeHandle 是ROS中的一个类，用于与ROS节点通信。它允许您访问ROS参数服务器、发布和订阅主题、调用服务等。
//这是一个参数读取操作，用于从ROS参数服务器中获取名为 "icp_options/distance" 的字符串参数的值。参数的类型是 std::string，因此我们将其存储在变量 str_temp 中。  
    if(str_temp == "POINT_TO_PLANE") options.optimize_options.distance = POINT_TO_PLANE;//这是一个结构体或类中的成员变量，用于存储距离计算方式的选项。在这段代码中，根据 str_temp 的值，将其设置为 POINT_TO_PLANE 或 CT_POINT_TO_PLANE。
    else if(str_temp == "CT_POINT_TO_PLANE") options.optimize_options.distance = CT_POINT_TO_PLANE;
    else std::cout << "The `icp_residual` " << str_temp << " is not supported." << std::endl;

    nh.param<std::string>("icp_options/weighting_scheme", str_temp, "ALL");//加权方式
    if(str_temp == "PLANARITY") options.optimize_options.weighting_scheme = PLANARITY;//平面化
    else if(str_temp == "NEIGHBORHOOD") options.optimize_options.weighting_scheme = NEIGHBORHOOD;//附近
    else if(str_temp == "ALL") options.optimize_options.weighting_scheme = ALL;
    else std::cout << "The `weighting_scheme` " << str_temp << " is not supported." << std::endl;

    nh.param<std::string>("icp_options/solver", str_temp, "LIO");
    if(str_temp == "LIO") options.optimize_options.solver = LIO;
    else if(str_temp == "LIDAR") options.optimize_options.solver = LIDAR;
    else std::cout << "The `solve_method` " << str_temp << " is not supported." << std::endl;

    nh.param<std::string>("icp_options/loss_function", str_temp, "CAUCHY");
    if(str_temp == "CAUCHY") options.optimize_options.loss_function = CAUCHY;
    else if(str_temp == "STANDARD") options.optimize_options.loss_function = STANDARD;
    else if(str_temp == "HUBER") options.optimize_options.loss_function = HUBER;
    else if(str_temp == "TOLERANT") options.optimize_options.loss_function = TOLERANT;
    else if(str_temp == "TRUNCATED") options.optimize_options.loss_function = TRUNCATED;
    else std::cout << "The `loss_function` " << str_temp << " is not supported." << std::endl;

    nh.param<std::string>("icp_options/viz_mode", str_temp, "TIMESTAMP");
    if(str_temp == "TIMESTAMP") options.optimize_options.viz_mode = TIMESTAMP;
    else if(str_temp == "WEIGHT") options.optimize_options.viz_mode = WEIGHT;
    else if(str_temp == "NORMAL") options.optimize_options.viz_mode = NORMAL;
    else std::cout << "The `solve_method` " << str_temp << " is not supported." << std::endl;
    // new ct-icp
}

void lioOptimization::allocateMemory()
{
    cloud_pro = new cloudProcessing();
    imu_pro = new imuProcessing();
}

void lioOptimization::initialValue()
{
    laser_point_cov = 0.001;

    G = vec3FromArray(v_G);
    R_imu_lidar = mat33FromArray(v_extrin_R);
    t_imu_lidar = vec3FromArray(v_extrin_t);

    cloud_pro->setExtrinR(R_imu_lidar);
    cloud_pro->setExtrinT(t_imu_lidar);

    last_time_lidar = -1.0;
    last_time_imu = -1.0;
    last_time_frame = -1.0;
    current_time = -1.0;

    index_frame = 1;

    fov_deg = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);

    LidarPlaneNormFactor::t_il = t_imu_lidar;
    LidarPlaneNormFactor::q_il = Eigen::Quaterniond(R_imu_lidar);
    LidarPlaneNormFactor::sqrt_info = sqrt(1 / laser_point_cov);

    CTLidarPlaneNormFactor::t_il = t_imu_lidar;
    CTLidarPlaneNormFactor::q_il = Eigen::Quaterniond(R_imu_lidar);
    CTLidarPlaneNormFactor::sqrt_info = sqrt(1 / laser_point_cov);

    registered_frames = 0;

    robust_num_consecutive_failures = 0;

    suspect_registration_error = false;

    options.optimize_options.init_num_frames = options.init_num_frames;

    switch(options.motion_compensation)
    {
        case NONE:
        case CONSTANT_VELOCITY:
            options.optimize_options.point_to_plane_with_distortion = false;
            options.optimize_options.distance = POINT_TO_PLANE;
            break;
        case ITERATIVE:
            options.optimize_options.point_to_plane_with_distortion = true;
            options.optimize_options.distance = POINT_TO_PLANE;
            break;
        case CONTINUOUS:
            options.optimize_options.point_to_plane_with_distortion = true;
            options.optimize_options.distance = CT_POINT_TO_PLANE;
            break;
        case IMU:
            options.optimize_options.point_to_plane_with_distortion = false;
            options.optimize_options.distance = POINT_TO_PLANE;
            break;
    }
    next_robust_level = options.robust_minimal_level;
}

void lioOptimization::addPointToMap(voxelHashMap &map, const Eigen::Vector3d &point, double voxel_size, int max_num_points_in_voxel, double min_distance_points, int min_num_points, cloudFrame* p_frame)
{
    short kx = static_cast<short>(point[0] / voxel_size);
    short ky = static_cast<short>(point[1] / voxel_size);
    short kz = static_cast<short>(point[2] / voxel_size);

    voxelHashMap::iterator search = map.find(voxel(kx, ky, kz));

    if(search != map.end())
    {
        auto &voxel_block = (search.value());

        if(!voxel_block.IsFull()) {
            double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
            for (int i(0); i < voxel_block.NumPoints(); ++i)
            {
                auto &_point = voxel_block.points[i];
                double sq_dist = (_point - point).squaredNorm();
                if (sq_dist < sq_dist_min_to_points)
                {
                    sq_dist_min_to_points = sq_dist;
                }
            }
            if(sq_dist_min_to_points > (min_distance_points * min_distance_points))
            {
                if(min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points)
                {
                    voxel_block.AddPoint(point);
                    addPointToPcl(points_world, point, p_frame);
                }
            }
        }
    }
    else
    {
        if(min_num_points <= 0){
            voxelBlock block(max_num_points_in_voxel);
            block.AddPoint(point);
            map[voxel(kx, ky, kz)] = std::move(block);
        }

    }
}

void lioOptimization::addPointsToMap(voxelHashMap &map, cloudFrame* p_frame, double voxel_size, int max_num_points_in_voxel, double min_distance_points, int min_num_points)
{
    for (const auto &point: p_frame->point_frame)//范围循环语句，自动推断类型，引用。p_frame->point_frame 是一个 指针成员访问表达式。point_frame 是该对象的一个成员，可能是一个数组、容器或其他数据结构。p_frame->point_frame 表示通过指针 p_frame 访问到的对象中的 point_frame 成员。
    {
        addPointToMap(map, point.point, voxel_size, max_num_points_in_voxel, min_distance_points, min_num_points, p_frame);
    }//point.point：这是一个点云数据的点。point 是 p_frame->point_frame 中的一个元素，而 point.point 是该点的具体坐标
    publishCLoudWorld(pub_cloud_world, points_world, p_frame);
    points_world->clear();//成员函数，用于清空points_world中的所有元素
}

void lioOptimization::removePointsFarFromLocation(voxelHashMap &map, const Eigen::Vector3d &location, double distance)
{
    std::vector<voxel> voxels_to_erase;
    for (auto &pair: map) {//范围循环，遍历map中的每个键值对。pair循环变量，代表键值对。auto自动推断pair类型。引用避免不必要的拷贝。map是一个关联容器，里面的每个元素都是一个键值对。
        Eigen::Vector3d pt = pair.second.points[0];//pair.first访问键，是pair对象的成员，可能是一个数组，容器或其他结构。.second访问值。假设 pair 是一个键值对，其中 second 是一个包含点云数据的对象。points[0] 表示访问点云数据中的第一个点。
        if ((pt - location).squaredNorm() > (distance * distance)) {
            voxels_to_erase.push_back(pair.first);
        }
    }
    for (auto &vox: voxels_to_erase)
        map.erase(vox);//有效地从地图中移除距离指定位置远于给定距离的体素
}

size_t lioOptimization::mapSize(const voxelHashMap &map)//计算给定voxelhashmap中所有体素的点数之和
{
    size_t map_size(0);//用于初始化一个名为 map_size 的变量。在这里，size_t 是一种无符号整数类型，通常用于表示集合的大小或索引
    for (auto &itr_voxel_map: map) {//遍历map中的每个键值对
        map_size += (itr_voxel_map.second).NumPoints();//对于每个体素，它调用 NumPoints 函数获取该体素中的点数，并将其累加到 map_size 中。
    }
    return map_size;
}

void lioOptimization::standardCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) //指向对象的常量指针。在ROS中是一种消息类型，表示点云数据
{
    double sample_size = index_frame < options.init_num_frames ? options.init_voxel_size : options.voxel_size;//真假

    assert(msg->header.stamp.toSec() > last_time_lidar);
    
    std::vector<point3D> v_point_cloud;
    double dt_offset;

    cloud_pro->process(msg, v_point_cloud, dt_offset);

    boost::mt19937_64 g;//boost::mt19937_64 是 Boost C++ 库 中的一个类型。64 位 Mersenne Twister 19937 伪随机数生成器。g 是一个变量，用于存储生成的随机数
    std::shuffle(v_point_cloud.begin(), v_point_cloud.end(), g);//std::shuffle 会随机重新排列 v_point_cloud 中的元素，使用了 g 作为均匀分布的随机数生成器。
//函数，打乱元素顺序。要打乱范围的起始迭代器，终止迭代器，随机数生成器，确定每个元素的新位置
    subSampleFrame(v_point_cloud, sample_size);//点云数据向量，在函数中进行子采样。子采样的大小

    std::shuffle(v_point_cloud.begin(), v_point_cloud.end(), g);

    lidar_buffer.push(v_point_cloud);
    time_buffer.push(std::make_pair(msg->header.stamp.toSec(), dt_offset / (double)1000.0));//函数，接受两个参数，返回std::pair对象。时间戳，从某个起始时间到当前时间的秒数。计算将毫秒转化为秒
//msg->header.stamp 表示 ROS 消息中的时间戳。toSec() 是一个成员函数，用于将时间戳转换为秒数。
    assert(msg->header.stamp.toSec() > last_time_lidar);
    last_time_lidar = msg->header.stamp.toSec();
}

void lioOptimization::imuHandler(const sensor_msgs::Imu::ConstPtr &msg)
{
    sensor_msgs::Imu::Ptr msg_temp(new sensor_msgs::Imu(*msg));//复制，副本。指向sensor_msgs::Imu对象的智能指针，一种特殊类型的指针，具有自动内存管理功能，使用了智能指针来管理新构造的IMU消息对象的生命周期，确保在不再需要时正确释放内存
//new创建一个新的对象的关键字，是一个运算符，用于在堆上动态分配内存并构造对象。用于创建一个新的 sensor_msgs::Imu 对象，并将其初始化为 msg 所指向的对象的副本。。ROS中用于表示IMU数据的消息类型。sensor_msgs::ImuConstPtr 是一个常见的 ROS 数据类型，用于表示 sensor_msgs::Imu 消息的常量指针。在 ROS 中，常量指针通常用于传递消息，以确保消息在传递过程中不被修改。
    if (abs(time_diff) > 0.1 && time_diff_enable)//时间差值的绝对值
    {
        msg_temp->header.stamp = ros::Time().fromSec(time_diff + msg->header.stamp.toSec());//实现对齐
    }

    assert(msg_temp->header.stamp.toSec() > last_time_imu);//最新时间戳，当前消息的时间戳晚于上一次记录的时间戳
//这有助于捕获潜在的错误，例如消息时间戳无序或不连续
    imu_buffer.push(msg_temp);//缓存队列

    assert(msg_temp->header.stamp.toSec() > last_time_imu);//确保在更新 last_time_imu 之前，新消息的时间戳仍然是有序的
    last_time_imu = msg_temp->header.stamp.toSec();
}//该段代码主要实现了对接收到的原始IMU数据进行预处理和校验，并将其添加到缓存队列中以供进一步使用。

std::vector<std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>, std::vector<point3D>>, std::pair<double, double>>> lioOptimization::getMeasurements()
{//代码中的循环用于处理缓冲区中的数据，并将满足特定条件的测量值添加到结果集合`measurements`
    std::vector<std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>, std::vector<point3D>>, std::pair<double, double>>> measurements;

    while (true)
    {
        if(imu_buffer.size() < 60 || lidar_buffer.size() < 2 || time_buffer.size() < 2)//或者。检查缓冲区中是否存在足够数量的IMU、Lidar和时间戳数据
            return measurements;//为了确保缓冲区中有足够的数据才能继续处理。

        if (!(imu_buffer.back()->header.stamp.toSec() > time_buffer.front().first + time_buffer.front().second))// 检查当前IMU消息时间戳是否晚于当前时间段内第一个时间戳加上偏移量。如果不满足条件，则将该时间段内所有缓存清除，并继续处理下一个时间段。
        {
            return measurements;
        }

        if (!(imu_buffer.front()->header.stamp.toSec() < time_buffer.front().first + time_buffer.front().second))//最早的 IMU 数据的时间戳不早于当前时间窗口的开始时间，
        {
            time_buffer.pop();//弹出 time_buffer 中的第一个时间窗口

            std::vector<point3D>().swap(lidar_buffer.front());//清空 lidar_buffer 中的第一个点云数据。
            assert(lidar_buffer.front().size() == 0);
            lidar_buffer.pop();//移除队列的第一个元素。清空队列中的旧数据。pop() 方法不返回被移除的元素，只是将其从队列中删除

            continue;//继续下一次循环
        }
//timestamp 是通过将 time_buffer 中第一个时间窗口的开始时间和偏移量相加而计算得出的。这个值表示当前时间窗口的时间戳。
        double timestamp = time_buffer.front().first + time_buffer.front().second;//根据时间窗口的开始时间和偏移量计算时间戳。。计算当前时间段内最后一个时间戳，以及该时间段开始时刻和偏移量，并从对应缓冲区中弹出相关数据。
        double timestamp_begin = time_buffer.front().first;
        double timestamp_offset = time_buffer.front().second;
        time_buffer.pop();

        if (fabs(timestamp_begin - time_buffer.front().first) > 1e-5)//0.00001.如果开始时间与下一个时间窗口的开始时间之间的差异超过 1e-5，则根据偏移量调整时间戳
        {//fabs 函数与 cmath 中的 abs 函数是相同的
            if (time_buffer.front().first - timestamp_begin - timestamp_offset > 1e-5)
            {
                timestamp_offset = time_buffer.front().first - timestamp_begin;
                timestamp = timestamp_begin + timestamp_offset;
            }
            else if (time_buffer.front().first - timestamp_begin - timestamp_offset < 1e-5)
            {
                timestamp_offset = time_buffer.front().first - timestamp_begin;
                timestamp = timestamp_begin + timestamp_offset;
            }
        }

        std::vector<point3D> v_point_cloud = lidar_buffer.front();//存储

        std::vector<point3D>().swap(lidar_buffer.front());
        assert(lidar_buffer.front().size() == 0);//返回的是激光雷达数据缓冲区中第一个点云数据的大小（即点的数量）。这个值表示当前时间窗口内激光雷达测量的数据点个数。
        lidar_buffer.pop();//清空第一个点云数据

        std::vector<sensor_msgs::ImuConstPtr> imu_measurements;
        while (imu_buffer.front()->header.stamp.toSec() < timestamp)
        {
            imu_measurements.emplace_back(imu_buffer.front());
            imu_buffer.pop();
        }

        imu_measurements.emplace_back(imu_buffer.front());

        if (imu_measurements.empty())
            ROS_WARN("no imu between two sweeps");

        measurements.emplace_back(std::make_pair(imu_measurements, v_point_cloud), std::make_pair(timestamp_begin, timestamp_offset));
        break;
    }
    return measurements;
}//这段代码用于从 IMU 和 Lidar 缓冲区获取一组测量值，并按照特定格式进行封装返回。

void lioOptimization::makePointTimestamp(std::vector<point3D> &sweep, double time_sweep_begin, double time_sweep_end)//扫描开始时刻（时间窗口的开始时间），结束时刻
{//函数的目的是给输入的点云数据添加时间戳信息
    if(cloud_pro->isPointTimeEnable())//检查是否启用了点云时间戳功能
    {
        return;
    }
    else
    {
        double delta_t = time_sweep_end - time_sweep_begin;//时间窗口的时间差

        std::vector<point3D>::iterator iter = sweep.begin();//迭代器对点云数据遍历

        while (iter != sweep.end())
        {
            if((*iter).timestamp > time_sweep_end) iter = sweep.erase(iter);
            else if((*iter).timestamp < time_sweep_begin) iter = sweep.erase(iter);//对于每个点，检查其时间戳是否超过扫描结束时刻或早于扫描开始时刻。如果是，则从向量中删除该点；
            else//否则，为该点计算相对时间和归一化相对时间，并将相对时间乘以1000转换为毫秒单位。
            {
                (*iter).relative_time = (*iter).timestamp - time_sweep_begin;
                (*iter).alpha_time = (*iter).relative_time / delta_t;//归一化相对时间
                (*iter).relative_time = (*iter).relative_time * 1000.0;
                iter++;
            }//这段代码用于给输入的点云数据添加相对时间、归一化相对时间和毫秒级别的绝对时间信息。确保它们在给定的时间窗口内
        }
    }
}

cloudFrame* lioOptimization::buildFrame(std::vector<point3D> &const_frame, state *cur_state, double timestamp_begin, double timestamp_offset)//指向点云数据向量的引用，当前状态指针，扫描开始时间，时间偏移值
{//函数构建一个 cloudFrame 对象，其中包含了点云数据、当前状态和时间戳信息。根据不同的情况，对点云数据进行处理和变换
    std::vector<point3D> frame(const_frame);//新点云数据向量，副本，复制存储

    double offset_begin = 0;//开始偏移时间
    double offset_end = timestamp_offset;//结束偏移时间差

    double dt_offset = 0;

    if(index_frame > 1)//在第二帧之后
        dt_offset -= timestamp_begin - all_cloud_frame.back()->time_sweep_end;//上一帧的结束时间戳

    makePointTimestamp(frame, timestamp_begin, timestamp_begin + timestamp_offset);
//给点云数据添加时间戳信息
    if (index_frame <= 2) {//前两帧
        for (auto &point_temp: frame) {
            point_temp.alpha_time = 1.0;
        }
    }

    if (index_frame > 2) {//第三帧及以后，则根据运动补偿模式进行畸变矫正和坐标变换操作。
        if (options.motion_compensation == CONSTANT_VELOCITY || (options.motion_compensation == IMU && !initial_flag)) {//运动补偿选项为恒定速度或 IMU（且不是初始状态）
            distortFrameUsingConstant(frame, cur_state->rotation_begin, cur_state->rotation, cur_state->translation_begin, cur_state->translation, R_imu_lidar, t_imu_lidar);
        }
        else if (options.motion_compensation == IMU && initial_flag) {
            distortFrameUsingImu(frame, cur_state, R_imu_lidar, t_imu_lidar);
        }

        for (auto &point_temp: frame) {//对点云数据进行变换
            transformPoint(options.motion_compensation, point_temp, cur_state->rotation_begin, cur_state->rotation, cur_state->translation_begin, cur_state->translation, R_imu_lidar, t_imu_lidar);
        }
    }
    else//否则，对点云数据进行初始变换。
    {
        for (auto &point_temp: frame) {
            Eigen::Quaterniond q_identity = Eigen::Quaterniond::Identity();
            Eigen::Vector3d t_zero = Eigen::Vector3d::Zero();
            transformPoint(options.motion_compensation, point_temp, q_identity, q_identity, t_zero, t_zero, R_imu_lidar, t_imu_lidar);
        }
    }

    cloudFrame *p_frame = new cloudFrame(frame, const_frame, cur_state);//通过创建新的 `cloudFrame` 对象并设置相关属性，将最终处理过的点云数据、原始点云数据和当前状态封装成帧对象返回
    p_frame->time_sweep_begin = timestamp_begin;
    p_frame->time_sweep_end = timestamp_begin + timestamp_offset;
    p_frame->offset_begin = offset_begin;
    p_frame->offset_end = offset_end;
    p_frame->dt_offset = dt_offset;
    p_frame->id = all_cloud_frame.size();
    p_frame->frame_id = index_frame;

    all_cloud_frame.push_back(p_frame);//在函数末尾还将新创建的帧对象指针添加到保存所有帧对象指针的向量中，并返回该指针。

    return p_frame;
}//这段代码用于构建一个表示扫描周期内点云数据的 `cloudFrame` 对象，并对输入的点云数据进行畸变校正和坐标变换操作。

void lioOptimization::stateInitialization(state *cur_state)//初始化当前状态
{
    registered_frames++;//对注册的帧数进行计数

    if (index_frame <= 2)//根据当前帧索引和初始化策略进行不同的处理
    {//将初始旋转和平移矩阵设置为单位矩阵和零向量
        cur_state->rotation_begin = Eigen::Quaterniond::Identity();
        cur_state->translation_begin = Eigen::Vector3d::Zero();
        cur_state->rotation = Eigen::Quaterniond::Identity();
        cur_state->translation = Eigen::Vector3d::Zero();
    }
    else if (index_frame == 3)
    {
        if (options.initialization == INIT_CONSTANT_VELOCITY)//常速度补偿
        {//计算下一帧结束时刻的旋转变换和平移变换。用上一帧结束时刻的旋转和平移作为初始值，并将计算得到的旋转和平移作为当前状态值。
            Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                    all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;

            Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation + 
                                         all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                                         all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * 
                                         (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation - 
                                         all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);

            cur_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
            cur_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
            cur_state->rotation = q_next_end;
            cur_state->translation = t_next_end;
        }
        else if (options.initialization == INIT_IMU)
        {
            if (initial_flag)
            {
                cur_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
                cur_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
            }
            else
            {
                Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                        all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;

                Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation + 
                                             all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                                             all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * 
                                             (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation - 
                                             all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);

                cur_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
                cur_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
                cur_state->rotation = q_next_end;
                cur_state->translation = t_next_end;
            }
        }
        else
        {
            cur_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation_begin;
            cur_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation_begin;
            cur_state->rotation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
            cur_state->translation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
        }
    }
    else
    {
        if (options.initialization == INIT_CONSTANT_VELOCITY)
        {
            if(options.motion_compensation == CONTINUOUS)
            {
                Eigen::Quaterniond q_next_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation_begin * 
                        all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation_begin.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation_begin;

                Eigen::Vector3d t_next_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation_begin + 
                                               all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation_begin * 
                                               all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation_begin.inverse() * 
                                               (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation_begin - 
                                               all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation_begin);

                cur_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
                cur_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
            }
            else
            {
                cur_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
                cur_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
            }

            Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                    all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;

            Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation + 
                                         all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                                         all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * 
                                         (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation - 
                                         all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);

            cur_state->rotation = q_next_end;
            cur_state->translation = t_next_end;
        }
        else if (options.initialization == INIT_IMU)
        {
            if (initial_flag)
            {
                cur_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
                cur_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
            }
            else
            {
                Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                        all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;

                Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation + 
                                             all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                                             all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * 
                                             (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation - 
                                             all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);

                cur_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
                cur_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
                cur_state->rotation = q_next_end;
                cur_state->translation = t_next_end;
            }
        }
        else
        {
            cur_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation_begin;
            cur_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation_begin;
            cur_state->rotation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
            cur_state->translation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
        }
    }
}

bool lioOptimization::assessRegistration(const cloudFrame *p_frame, estimationSummary &summary)//评估注册的质量，判断注册是否成功，指向...类型的指针，...类型的引用
{

    bool success = summary.success;//声明变量，赋值，跟踪评估的状态。复制当前的成功状态

    if(summary.robust_level == 0 && (summary.relative_orientation > options.robust_threshold_relative_orientation ||
         summary.ego_orientation > options.robust_threshold_ego_orientation))//ego自我。// 如果注册的稳健程度为0并且相对方向或者自我方向大于设定的阈值
    {
        if (summary.robust_level < options.robust_num_attempts_when_rotation) {//如果当前稳健程度小于指定的尝试旋转次数
            summary.error_message = "Large rotations require at a robust_level of at least 1 (got:" +//“大旋转需要至少1的鲁棒级
                                    std::to_string(summary.robust_level) + ").";// 设置错误信息
            return false;//返回失败
        }
    }

    if(summary.relative_distance > options.robust_relative_trans_threshold) {// 如果相对距离大于设定的阈值
        summary.error_message = "The relative distance is too important"; // 设置错误信息
        return false;//返回失败
    }

    bool do_neighbor_assessment = summary.distance_correction > 0.1;//邻域评估
    do_neighbor_assessment |= summary.relative_distance > options.robust_neighborhood_min_dist;
    do_neighbor_assessment |= summary.relative_orientation > options.robust_neighborhood_min_orientation;// 根据一些条件决定是否进行邻域评估

    if(do_neighbor_assessment && registered_frames > options.init_num_frames)//已注册的帧数超过了初始帧数
    {
        if (options.robust_registration)// 如果开启了稳健注册
        {
            const double kSizeVoxelMap = options.optimize_options.size_voxel_map;//定义常量，值被初始化。一个变量或结构体中的成员，表示体素地图的大小
            voxel voxel_temp;//声明变量
            double ratio_empty_voxel = 0;//声明变量，初始化，可能用于跟踪空的体素比例
            double ratio_half_full_voxel = 0;//计算一些体素指标，跟踪半满体素比例

            for (auto &point_temp: p_frame->point_frame) {//循环语句，遍历：后的每个元素。：前是临时变量，存储当前迭代的点
                voxel_temp = voxel::coordinates(point_temp.point, kSizeVoxelMap);// 计算空体素的比例和至少一半满的体素的比例。计算 voxel_temp 的体素的坐标。voxel::coordinates 是一个函数或方法，接受一个点的坐标 point_temp.point 和一个体素地图的大小 kSizeVoxelMap 作为参数，返回一个体素的坐标。
                //voxel_map.find(voxel_temp) 返回一个迭代器，指向 voxel_temp 在 voxel_map 中的位置（如果存在。voxel_map.end() 是一个迭代器，指向 voxel_map 的末尾。voxel_map.find(voxel_temp) == voxel_map.end() 的含义是：如果 voxel_temp 不在 voxel_map 中，即迭代器指向 voxel_map 的末尾，那么条件成立，返回 true
                if (voxel_map.find(voxel_temp) == voxel_map.end())//检查 voxel_temp 是否在 voxel_map 中。
                    ratio_empty_voxel += 1;//如果 voxel_temp 不在 voxel_map 中，说明这是一个空的体素，将 ratio_empty_voxel 增加 1。
                if (voxel_map.find(voxel_temp) != voxel_map.end() &&//检查 voxel_temp 是否在 voxel_map 中，并且该体素中的点数是否超过了 options.max_num_points_in_voxel 的一半
                    voxel_map.at(voxel_temp).NumPoints() > options.max_num_points_in_voxel / 2) {
                    // Only count voxels which have at least
                    ratio_half_full_voxel += 1;//如果是，说明这是至少半满的体素，将 ratio_half_full_voxel 增加 1
                }//这段代码用于计算空体素和至少半满体素的比例。
            }

            ratio_empty_voxel /= p_frame->point_frame.size();//将 ratio_empty_voxel 的值除以 p_frame->point_frame.size()，也就是点帧中点的数量。结果是空体素的比例
            ratio_half_full_voxel /= p_frame->point_frame.size();//结果是至少半满体素的比例。

            if (ratio_half_full_voxel < options.robust_full_voxel_threshold ||
                ratio_empty_voxel > options.robust_empty_voxel_threshold)
            {// 如果至少一半满的体素的比例小于阈值或者空体素的比例大于阈值
                success = false; // 设置成功状态为失败
                if (ratio_empty_voxel > options.robust_empty_voxel_threshold)
                    summary.error_message = "[Odometry::AssessRegistration] Ratio of empty voxels " +
                                            std::to_string(ratio_empty_voxel) + "above threshold."; // 生成一条日志消息。设置错误信息。 将 ratio_empty_voxel 的值转换为字符串
                else
                    summary.error_message = "[Odometry::AssessRegistration] Ratio of half full voxels " +
                                            std::to_string(ratio_half_full_voxel) + "below threshold.";

            }
        }
    }

    if (summary.relative_distance > options.distance_error_threshold)
    {//如果相对距离超过设定的阈值
        return false;
    }

    return success;// 返回评估的成功状态
}//这段代码的主要目的是评估注册的质量，并根据一系列条件判断注册是否成功，其中包括了稳健性评估和邻域评估

estimationSummary lioOptimization::poseEstimation(cloudFrame *p_frame)//姿态估计函数
{
    auto start = std::chrono::steady_clock::now();//记录开始时间。是 C++ 中用于获取当前时间的函数。返回一个表示当前时间的时间点（time point）。

    icpOptions optimize_options = options.optimize_options;// 获取优化选项
    const double kSizeVoxelInitSample = options.voxel_size;//初始采样体素大小

    const double kSizeVoxelMap = optimize_options.size_voxel_map;//优化选项中的体素地图大小
    const double kMinDistancePoints = options.min_distance_points;//最小点距离
    const int kMaxNumPointsInVoxel = options.max_num_points_in_voxel;//体素内最大点数限制

    const state* initial_state = new state(p_frame->p_state, true);// // 创建指向当前帧状态的初始状态指针。state 是一个类或数据类型。new state 在堆上创建了一个 state 类的实例。构造函数会使用两个参数进行初始化
    estimationSummary summary;// 创建姿态估计摘要对象
    summary.state_frame = new state(initial_state, true);// 将初始状态赋值给姿态估计摘要中的状态帧
    state* previous_state = new state(initial_state, true);// 创建指向上一个状态的状态指针

    if(p_frame->frame_id > 1)// 如果当前帧的 ID 大于 1，执行姿态估计
    {
        bool good_enough_registration = false;
        summary.number_of_attempts = 1;
        double sample_voxel_size = p_frame->frame_id < options.init_num_frames ? options.init_sample_voxel_size : options.sample_voxel_size;
        double min_voxel_size = std::min(options.init_voxel_size, options.voxel_size);

        auto increaseRobustnessLevel = [&]() {
            previous_state->release();
            previous_state = new state(summary.state_frame, true);
            
            p_frame->p_state = new state(initial_state, true);

            optimize_options.voxel_neighborhood = std::min(++optimize_options.voxel_neighborhood,
                                                         options.robust_max_voxel_neighborhood);
            optimize_options.ls_max_num_iters += 30;
            if (optimize_options.max_num_residuals > 0)
                optimize_options.max_num_residuals = optimize_options.max_num_residuals * 2;
            optimize_options.num_iters_icp = std::min(optimize_options.num_iters_icp + 20, 50);
            optimize_options.threshold_orientation_norm = std::max(
                    optimize_options.threshold_orientation_norm / 10, 1.e-5);
            optimize_options.threshold_translation_norm = std::max(
                    optimize_options.threshold_orientation_norm / 10, 1.e-4);
            sample_voxel_size = std::max(sample_voxel_size / 1.5, min_voxel_size);
            optimize_options.ls_sigma *= 1.2;
            optimize_options.max_dist_to_plane_icp *= 1.5;
        };

        summary.robust_level = 0;
        do {
            if(summary.robust_level < next_robust_level)
            {
                //increaseRobustnessLevel();
                continue;
            }

            auto start_ct_icp = std::chrono::steady_clock::now();
            optimize(p_frame, optimize_options, summary, sample_voxel_size);
            auto end_ct_icp = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_icp = (end_ct_icp - start);

            if(p_frame->frame_id > 1)
            {
                summary.distance_correction = (p_frame->p_state->translation_begin - all_cloud_frame[p_frame->id - 1]->p_state->translation).norm();

                summary.relative_orientation = AngularDistance(all_cloud_frame[p_frame->id - 1]->p_state->rotation, p_frame->p_state->rotation);

                summary.ego_orientation = AngularDistance(summary.state_frame->rotation_begin, summary.state_frame->rotation);

            }

            summary.relative_distance = (p_frame->p_state->translation - p_frame->p_state->translation_begin).norm();

            good_enough_registration = assessRegistration(p_frame, summary);

            if(options.robust_fail_early)
                summary.success = good_enough_registration;

            if(!good_enough_registration)
            {
                if(options.robust_registration && summary.number_of_attempts < options.robust_num_attempts)
                {
                    double trans_distance = (previous_state->translation_begin - summary.state_frame->translation_begin).norm()
                                          + (previous_state->translation - summary.state_frame->translation).norm();

                    double rot_distance = ((previous_state->rotation_begin * summary.state_frame->rotation_begin.inverse()).toRotationMatrix() - Eigen::Matrix3d::Identity()).norm() 
                                        + ((previous_state->rotation * summary.state_frame->rotation.inverse()).toRotationMatrix() - Eigen::Matrix3d::Identity()).norm();

                    //increaseRobustnessLevel();
                    summary.robust_level++;
                    summary.number_of_attempts++;
                }
                else
                {
                    good_enough_registration = true;
                }
            }
        } while (!good_enough_registration);

        p_frame->success = summary.success;

        if(!summary.success)
        {
            return summary;
        }

        if(summary.number_of_attempts >= options.robust_num_attempts)
            robust_num_consecutive_failures++;
        else
            robust_num_consecutive_failures = 0;
    }

    bool add_points = true;

    if(options.robust_registration)
    {
        suspect_registration_error = summary.number_of_attempts >= options.robust_num_attempts;

        if (summary.ego_orientation > options.robust_threshold_ego_orientation ||
            summary.relative_orientation > options.robust_threshold_relative_orientation)
        {
            add_points = false;
        }

        if (suspect_registration_error) {
            add_points |= (robust_num_consecutive_failures > 5);
        }

        next_robust_level = add_points ? options.robust_minimal_level : options.robust_minimal_level + 1;
        if (!summary.success)
            next_robust_level = options.robust_minimal_level + 2;
        else {
            if (summary.relative_orientation > options.robust_threshold_relative_orientation ||
                summary.ego_orientation > options.robust_threshold_ego_orientation) {
                next_robust_level = options.robust_minimal_level + 1;
            }
            if (summary.number_of_attempts > 1) {
                next_robust_level = options.robust_minimal_level + 1;
            }
        }

    }

    if(add_points)
        addPointsToMap(voxel_map, p_frame, kSizeVoxelMap, kMaxNumPointsInVoxel, kMinDistancePoints);

    const double kMaxDistance = options.max_distance;
    const Eigen::Vector3d location = p_frame->p_state->translation;

    removePointsFarFromLocation(voxel_map, location, kMaxDistance);

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

    summary.corrected_points = p_frame->point_frame;
    summary.all_corrected_points = p_frame->const_frame;

    Eigen::Quaterniond q_begin = summary.state_frame->rotation_begin;
    Eigen::Quaterniond q_end = summary.state_frame->rotation;

    for (auto &point_temp: summary.all_corrected_points) {
        double alpha_time = point_temp.alpha_time;
        Eigen::Quaterniond slerp = q_begin.slerp(alpha_time, q_end).normalized();
        point_temp.point = slerp.toRotationMatrix() * point_temp.raw_point +
                     summary.state_frame->translation_begin * (1.0 - alpha_time) + alpha_time * summary.state_frame->translation;
    }

    return summary;
}

void lioOptimization::stateEstimation(std::vector<point3D> &const_frame, double timestamp_begin, double timestamp_offset)
{
    stateInitialization(imu_pro->current_state);//访问名为 imu_pro 的对象的 current_state 成员
//对IMU当前状态初始化
    cloudFrame *p_frame = buildFrame(const_frame, imu_pro->current_state, timestamp_begin, timestamp_offset);
//帧对象，传入点云数据、IMU当前状态、时间戳信息作为参数传递给函数
    estimationSummary summary = poseEstimation(p_frame);//对帧对象姿态估计
    summary.release();//释放内存空间

    if (options.optimize_options.solver == LIO && !initial_flag)//lio并且false，则执行相应的系统初始化操作
    {
        if(options.method_system_init == MOTION_INIT)
            motionInitialization();
        else if(options.method_system_init == STATIC_INIT)
            staticInitialization(p_frame);
    }

    std::cout << "after solution: " << std::endl;//在控制台上打印一些输出信息，开始和结束时可的旋转和平移信息
    std::cout << "rotation_begin: " << p_frame->p_state->rotation_begin.x() << " " << p_frame->p_state->rotation_begin.y() << " " 
              << p_frame->p_state->rotation_begin.z() << " " << p_frame->p_state->rotation_begin.w() << std::endl;
    std::cout << "translation_begin: " << p_frame->p_state->translation_begin.x() << " " << p_frame->p_state->translation_begin.y() << " " << p_frame->p_state->translation_begin.z() << std::endl;

    std::cout << "rotation_end: " << p_frame->p_state->rotation.x() << " " << p_frame->p_state->rotation.y() << " " 
              << p_frame->p_state->rotation.z() << " " << p_frame->p_state->rotation.w() << std::endl;
    std::cout << "translation_end: " << p_frame->p_state->translation.x() << " " << p_frame->p_state->translation.y() << " " << p_frame->p_state->translation.z() << std::endl;

    imu_pro->last_state = imu_pro->current_state;//更新IMU上一次状态为当前状态
    imu_pro->current_state = new state(imu_pro->last_state, false);//创建一个新的状态对象作为当前状态

    publish_odometry(pub_odom,p_frame);//发布里程计信息
    publish_path(pub_path,p_frame);  //路径 

    if(debug_output)//如果启用了调试输出，则将点云数据保存到PCD文件中
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr p_cloud_temp;//后指向前（对象）的指针
        p_cloud_temp.reset(new pcl::PointCloud<pcl::PointXYZINormal>());//初始化，使其指向一个新的点云对象
        point3DtoPCL(p_frame->point_frame, p_cloud_temp);//函数，点云数据转化为PCL格式并存储在p_cloud_temp 中。

        std::string pcd_path(output_path + "/cloud_frame/" + std::to_string(index_frame) + std::string(".pcd"));//构建PCD文件的路径。输出路径前缀。文件夹名称。整数变量转换为字符串，构建PCD文件路径，也就是文件名
        saveCutCloud(pcd_path, p_cloud_temp);//将后者保存到前者中
    }

    int num_remove = 0;

    if (initial_flag)//在满足特定条件时对缓存中的点云帧对象进行清理并更新其ID值以确保连续性。根据initial_flag不同有不同处理方式)
    {
        if (index_frame > 1)
        {
            while (all_cloud_frame.size() > 2)//元素对象数量>2时
            {
                recordSinglePose(all_cloud_frame[0]);//记录第一个点云帧的姿态
                all_cloud_frame[0]->release();//释放第一个点云帧对象内存
                all_cloud_frame.erase(all_cloud_frame.begin());//删除第一个元素
                num_remove++;//计数器
            }
            assert(all_cloud_frame.size() == 2);//这段代码的目的是在满足特定条件时对点云帧对象进行清理，并确保只保留两个点云帧对象，以保持连续性。
        }
    }
    else
    {
        while (all_cloud_frame.size() > options.num_for_initialization)
        {
            recordSinglePose(all_cloud_frame[0]);
            all_cloud_frame[0]->release();
            all_cloud_frame.erase(all_cloud_frame.begin());
            num_remove++;
        }
    }
    
//最后, 释放已删除帧对象所占内存，并更新其ID值以确保连续性。
    for(int i = 0; i < all_cloud_frame.size(); i++)
        all_cloud_frame[i]->id = all_cloud_frame[i]->id - num_remove;
}//该段代码主要实现了从构建框架、姿态估计、系统初始化到发布消息等一系列与状态估计相关功能。同时也包含了一些额外功能如保存点云数据、清理缓冲区等。

void lioOptimization::recordSinglePose(cloudFrame *p_frame)
{
    std::ofstream foutC(std::string(output_path + "/pose.txt"), std::ios::app);

    foutC.setf(std::ios::scientific, std::ios::floatfield);
    foutC.precision(6);

    foutC << std::fixed << p_frame->time_sweep_end << " ";
    foutC << p_frame->p_state->translation.x() << " " << p_frame->p_state->translation.y() << " " << p_frame->p_state->translation.z() << " ";
    foutC << p_frame->p_state->rotation.x() << " " << p_frame->p_state->rotation.y() << " " << p_frame->p_state->rotation.z() << " " << p_frame->p_state->rotation.w();
    foutC << std::endl; 

    foutC.close();
}

void lioOptimization::set_posestamp(geometry_msgs::PoseStamped &body_pose_out,cloudFrame *p_frame)//设置位姿信息。一个 geometry_msgs::PoseStamped 类型的引用，用于存储位姿信息
{
    body_pose_out.pose.position.x = p_frame->p_state->translation.x();//将 p_frame 中的位移信息（translation.x()）赋值给 body_pose_out 的 x 坐标。这通常用于将传感器或机器人的位姿信息转换为 ROS 消息格式
    body_pose_out.pose.position.y = p_frame->p_state->translation.y();
    body_pose_out.pose.position.z = p_frame->p_state->translation.z();
    
    body_pose_out.pose.orientation.x = p_frame->p_state->rotation.x();//旋转四元数
    body_pose_out.pose.orientation.y = p_frame->p_state->rotation.y();
    body_pose_out.pose.orientation.z = p_frame->p_state->rotation.z();
    body_pose_out.pose.orientation.w = p_frame->p_state->rotation.w();
}//将传感器或机器人的位姿信息转换为ROS消息格式，并存储在指定的变量中。这段代码的目的是将`cloudFrame`对象中的位移信息和旋转信息转换为ROS消息格式，并设置到一个`geometry_msgs::PoseStamped`类型的引用变量中。

void lioOptimization::publish_path(ros::Publisher pub_path,cloudFrame *p_frame)//publish_path 函数负责使用 ROS 发布者来发布路径信息（可能是点云路径。pub_path：一个 ROS 发布者对象，用于发布路径信息。路径数据从 p_frame 指向的 cloudFrame 对象中获取
{
    set_posestamp(msg_body_pose,p_frame);
    msg_body_pose.header.stamp = ros::Time().fromSec(p_frame->time_sweep_end);//时间戳，扫描结束时刻
    msg_body_pose.header.frame_id = "camera_init";

    static int i = 0;//代码会统计调用次数，并每隔10次调用将当前姿态信息添加到路径中，并使用ROS发布者将路径消息发布出去。
    i++;
    if (i % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pub_path.publish(path);
    }
}//这段代码实现了一个函数用于定期发布传感器或导航系统的运动轨迹或路径信息。

void lioOptimization::publishCLoudWorld(ros::Publisher &pub_cloud_world, pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points, cloudFrame* p_frame)
{
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_points, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(p_frame->time_sweep_end);
    laserCloudmsg.header.frame_id = "camera_init";
    pub_cloud_world.publish(laserCloudmsg);
}

void lioOptimization::addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points, const Eigen::Vector3d& point, cloudFrame *p_frame)
{
    pcl::PointXYZI cloudTemp;
    
    cloudTemp.x = point.x();
    cloudTemp.y = point.y();
    cloudTemp.z = point.z();
    cloudTemp.intensity = 50*(point.z()- p_frame->p_state->translation.z());
    pcl_points->points.push_back(cloudTemp);
}


void lioOptimization::publish_odometry(const ros::Publisher & pubOdomAftMapped, cloudFrame *p_frame)
{
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(p_frame->p_state->rotation.z(), -p_frame->p_state->rotation.x(), -p_frame->p_state->rotation.y());

    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(p_frame->time_sweep_end);
    odomAftMapped.pose.pose.orientation.x = p_frame->p_state->rotation.x();
    odomAftMapped.pose.pose.orientation.y = p_frame->p_state->rotation.y();
    odomAftMapped.pose.pose.orientation.z = p_frame->p_state->rotation.z();
    odomAftMapped.pose.pose.orientation.w = p_frame->p_state->rotation.w();
    odomAftMapped.pose.pose.position.x = p_frame->p_state->translation.x();
    odomAftMapped.pose.pose.position.y = p_frame->p_state->translation.y();
    odomAftMapped.pose.pose.position.z = p_frame->p_state->translation.z();
    pubOdomAftMapped.publish(odomAftMapped);

    laserOdometryTrans.frame_id_ = "/camera_init";
    laserOdometryTrans.child_frame_id_ = "/laser_odom";
    laserOdometryTrans.stamp_ = ros::Time().fromSec(p_frame->time_sweep_end);
    laserOdometryTrans.setRotation(tf::Quaternion(p_frame->p_state->rotation.x(), 
                                                  p_frame->p_state->rotation.y(), 
                                                  p_frame->p_state->rotation.z(), 
                                                  p_frame->p_state->rotation.w()));
    laserOdometryTrans.setOrigin(tf::Vector3(p_frame->p_state->translation.x(), 
                                             p_frame->p_state->translation.y(), 
                                             p_frame->p_state->translation.z()));
    tfBroadcaster.sendTransform(laserOdometryTrans);
}

void lioOptimization::run()
{
    std::vector<std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>, std::vector<point3D>>, std::pair<double, double>>> measurements = getMeasurements();
//获取测量数据。总之，这行代码创建了一个名为measurements的变量，它是一个std::vector，其中包含了多层嵌套的std::pair，其中存储了传感器数据、三维点和其他信息。使用括号初始化了名为measurements的变量，将其赋值为getMeasurements()函数的返回值
    if(measurements.size() == 0) return;
//如果没有测量数据，直接返回
    for (auto &measurement : measurements)//遍历测量数据。for是一个循环关键字，创建一个循环结构。auto类型推断关键字允许编译器根据变量的初始化值自动推断其类型。表示编译器会根据measurements中的元素类型来确定measurement的类型。引用声明，允许我们在循环中修改原始数据。: measurements：这是范围循环的语法，它指定了要遍历的容器（在这里是measurements
    {//这行代码的作用是遍历名为measurements的容器中的每个元素，并将每个元素赋值给名为measurement的变量。在循环体内，您可以使用measurement来访问容器中的数据。
        auto v_point_cloud = measurement.first.second;//成员访问操作符，的第一个元素的第二个元素
        double time_frame = measurement.second.first + measurement.second.second;
        double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;//初始化变量
//提取点云数据和时间戳
        for (auto &imu_msg : measurement.first.first)//遍历IMU测量数据
        {
            double time_imu = imu_msg->header.stamp.toSec();
            if (time_imu <= time_frame)//若当前 IMU 时间小于等于当前时间帧，则更新状态
            { //如果当前时间小于等于当前时间帧，则将线性加速度和角速度作为IMU处理的输入，并更新状态。否则，根据时间间隔对线性加速度和角速度进行加权融合，并将融合后的值作为IMU处理的输入。
                if(current_time < 0)// 如果当前时间小于 0，则将当前时间设为当前时间帧的开始时间
                    current_time = measurement.second.first;
                double dt = time_imu - current_time;//计算时间间隔

                if(dt < -1e-6) continue;//-1e-6 表示负一百万分之一，即 -0.000001。这是一个非常小的数值，通常用于比较浮点数的精度或进行数值计算时的容差判断。在这段代码中，它被用来检查变量 dt 是否小于这个阈值。如果 dt 小于 -1e-6，则会跳过当前循环的剩余部分。
                assert(dt >= 0);//assert 是一个调试宏，用于在运行时检查条件是否为真。如果条件为假（即 dt 的值小于零），则程序会终止并显示错误消息。在这里，它确保 dt 的值不会小于零
                current_time = time_imu;
                dx = imu_msg->linear_acceleration.x;
                dy = imu_msg->linear_acceleration.y;
                dz = imu_msg->linear_acceleration.z;
                rx = imu_msg->angular_velocity.x;
                ry = imu_msg->angular_velocity.y;
                rz = imu_msg->angular_velocity.z;//提取线性加速的和角速度
                imu_pro->process(dt, Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(rx, ry, rz), time_imu);//处理IMU数据

                if (options.optimize_options.solver == LIO && !initial_flag)//如果选定的优化器为LIO，并且尚未进行初始化，则存储静态加速度和角速度
                {//此外，如果选定的优化器是LIO（Lidar-Inertial Odometry），并且尚未进行初始化，则会存储静态加速度和角速度。
//总之，这段代码通过处理传感器数据并利用IMU信息估计相机或导航传感器在不同时刻下的姿态、位置和速度等状态。
                    v_acc_static.push_back(Eigen::Vector3d(dx, dy, dz));
                    v_gyr_static.push_back(Eigen::Vector3d(rx, ry, rz));
                }
            }
            else
            {
                double dt_1 = time_frame - current_time;
                double dt_2 = time_imu - time_frame;//计算时间间隔
                current_time = time_frame;
                assert(dt_1 >= 0);
                assert(dt_2 >= 0);
                assert(dt_1 + dt_2 > 0);
                double w1 = dt_2 / (dt_1 + dt_2);
                double w2 = dt_1 / (dt_1 + dt_2);//计算加权系数
                dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                rz = w1 * rz + w2 * imu_msg->angular_velocity.z;//对线性加速度和角速度进行加权融合
                imu_pro->process(dt_1, Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(rx, ry, rz), time_frame);//处理IMU数据
            }
        }

        stateEstimation(v_point_cloud, measurement.second.first, measurement.second.second);//状态估计
        
        last_time_frame = time_frame;//更新最后时间帧和索引
        index_frame++;

        std::vector<point3D>().swap(measurement.first.second);//清空点云数据
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_optimization");//初始化ros节点，这个节点将在ros系统中注册自己
    ros::Time::init();//初始化ros时间，必要步骤以便在ros中使用时间戳
    
    lioOptimization LIO;//创建类的实例。这个类可能包含了某种里程计算法或系统的逻辑。

    ros::Rate rate(200);//循环频率，以每秒200次的速度运行
    while (ros::ok())//进入一个无限循环，只要 ROS 系统处于运行状态。
    {
        ros::spinOnce();//处理所有待处理的 ROS 消息。这通常包括来自传感器、其他节点或服务的消息。

        LIO.run();//执行 lioOptimization 类中的逻辑。这可能涉及到里程计算法、传感器融合或其他相关任务。

        rate.sleep();//等待一段时间以保持循环的频率
    }

    return 0;//退出程序
}//这段代码初始化一个 ROS 节点，创建一个 lioOptimization 类的实例，并在循环中处理消息和执行相关的逻辑
