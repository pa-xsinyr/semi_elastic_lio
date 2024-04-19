#include "cloudProcessing.h"
#include "utility.h"

cloudProcessing::cloudProcessing()
{
    point_filter_num = 1;
    sweep_id = 0;
}

void cloudProcessing::setLidarType(int para)
{
    lidar_type = para;
}

void cloudProcessing::setNumScans(int para)
{
    N_SCANS = para;

    for(int i = 0; i < N_SCANS; i++){
        pcl::PointCloud<pcl::PointXYZINormal> v_cloud_temp;//模板类，结构体
        v_cloud_temp.clear();
        scan_cloud.push_back(v_cloud_temp);//scan_cloud可能是一个存储多个点云的容器，比如std::vector<pcl::PointCloud<pcl::PointXYZINormal>>
    }

    assert(N_SCANS == scan_cloud.size());//检查函数

    for(int i = 0; i < N_SCANS; i++){
        std::vector<extraElement> v_elem_temp;
        v_extra_elem.push_back(v_elem_temp);//为每次扫描创建一个空向量，并将它们添加到一个容器中
    }

    assert(N_SCANS == v_extra_elem.size());
}

void cloudProcessing::setScanRate(int para)
{
    SCAN_RATE = para;
}//设置扫描率

void cloudProcessing::setTimeUnit(int para)
{
    time_unit = para;

    switch (time_unit)
    {
    case SEC:
        time_unit_scale = 1.e3f;//1000
        break;
    case MS:
        time_unit_scale = 1.f;//1
        break;
    case US:
        time_unit_scale = 1.e-3f;//0.001
        break;
    case NS:
        time_unit_scale = 1.e-6f;//0.000001
        break;
    default:
        time_unit_scale = 1.f;
        break;
    }
}//将不同时间单位转换为同一时间单位

void cloudProcessing::setBlind(double para)
{
    blind = para;//盲区或阈值
}

void cloudProcessing::setExtrinR(Eigen::Matrix3d &R)//矩阵，表示旋转
{
    R_imu_lidar = R;
}

void cloudProcessing::setExtrinT(Eigen::Vector3d &t)//向量，平移
{
    t_imu_lidar = t;
}

void cloudProcessing::setPointFilterNum(int para)
{
    point_filter_num = para;
}

void cloudProcessing::process(const sensor_msgs::PointCloud2::ConstPtr &msg, std::vector<point3D> &v_cloud_out, double &dt_offset)//消息类型，点云结构和内容。模板类的实例，类型的动态数组，自定义的类或结构体
{
    switch (lidar_type)
    {
    case OUST:
        ousterHandler(msg, v_cloud_out, dt_offset);
        break;

    case VELO:
        velodyneHandler(msg, v_cloud_out, dt_offset);
        break;

    case ROBO:
        robosenseHandler(msg, v_cloud_out, dt_offset);
        break;

    default:
        ROS_ERROR("Only Velodyne LiDAR interface is supported currently.");
        break;
    }

    sweep_id++;
}

void cloudProcessing::ousterHandler(const sensor_msgs::PointCloud2::ConstPtr &msg, std::vector<point3D> &v_cloud_out, double &dt_offset)//处理后的点云数据，时间偏移
{
    pcl::PointCloud<ouster_ros::Point> raw_cloud;
    pcl::fromROSMsg(*msg, raw_cloud);//ros消息转换为点云数据
    int size = raw_cloud.points.size();

    double dt_last_point;

    if (size == 0)
    {
        dt_offset = 1000.0 / double(SCAN_RATE);//计算时间偏移值，将扫描率转换为时间间隔

        return;
    }

    if (raw_cloud.points[size - 1].t > 0)
        given_offset_time = true;
    else
        given_offset_time = false;

    if (given_offset_time)
    {
        sort(raw_cloud.points.begin(), raw_cloud.points.end(), time_list_ouster);//排序函数，定义点的排序规则
        dt_last_point = raw_cloud.points.back().t * time_unit_scale;//计算将时间戳转换为特定时间单位
    }

    double omega = 0.361 * SCAN_RATE;

    std::vector<bool> is_first;//储存布尔值
    is_first.resize(N_SCANS);
    fill(is_first.begin(), is_first.end(), true);//函数，将所有元素设置为true

    std::vector<double> yaw_first_point;
    yaw_first_point.resize(N_SCANS);
    fill(yaw_first_point.begin(), yaw_first_point.end(), 0.0);

    std::vector<point3D> v_point_full;//存储处理后的点云数据

    for (int i = 0; i < size; i++)
    {
        point3D point_temp;//自定义的类或结构体表示三维空间中的一个点

        point_temp.raw_point = Eigen::Vector3d(raw_cloud.points[i].x, raw_cloud.points[i].y, raw_cloud.points[i].z);
        point_temp.point = point_temp.raw_point;
        point_temp.relative_time = raw_cloud.points[i].t * time_unit_scale;//计算将时间戳转换为特定时间单位


        if (!given_offset_time)
        {
            int layer = raw_cloud.points[i].ring;//点云数据中的一个特定层
            double yaw_angle = atan2(point_temp.raw_point.y(), point_temp.raw_point.x()) * 57.2957;//计算了一个点相对于原点的偏航角，将弧度转化为度的常数

            if (is_first[layer])//检查是否是每一层的第一个点，层的索引
            {
                yaw_first_point[layer] = yaw_angle;//数组元素的访问，数组名，用作数组的索引，表示数组中索引为layer的元素，layer层的第一个点的偏航角
                is_first[layer] = false;//表示已经处理过了
                point_temp.relative_time = 0.0;//点的实例，是一个自定义的类或结构体，有两个成员

                v_point_full.push_back(point_temp);

                continue;//跳过本次循环的剩余部分，直接开始下次循环
            }

            if (yaw_angle <= yaw_first_point[layer])//计算点的相对时间
            {
                point_temp.relative_time = (yaw_first_point[layer] - yaw_angle) / omega;
            }
            else
            {
                point_temp.relative_time = (yaw_first_point[layer] - yaw_angle + 360.0) / omega;
            }

            point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();//计算点的时间戳，ROS消息实例，消息的头部，包含时间戳和帧ID，成员函数，将时间戳转化为秒
            v_point_full.push_back(point_temp);
        }

        if (given_offset_time && i % point_filter_num == 0)//并且，除以
        {
            if (point_temp.raw_point.x() * point_temp.raw_point.x() + point_temp.raw_point.y() * point_temp.raw_point.y() + point_temp.raw_point.z() * point_temp.raw_point.z() > (blind * blind))
            {
                point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
                point_temp.alpha_time = point_temp.relative_time / dt_last_point;//计算，点的相对时间除以上一个点的时间间隔

                v_cloud_out.push_back(point_temp);
            }
        }
    }

    if (!given_offset_time)
    {
        assert(v_point_full.size() == size);//函数，确保点的向量大小等于预期大小，否则，程序会停止并打印错误消息

        sort(v_point_full.begin(), v_point_full.end(), time_list);
        dt_last_point = v_point_full.back().relative_time;//获取排序后的最后一个点的相对时间

        for (int i = 0; i < size; i++)
        {
            if (i % point_filter_num == 0)
            {
                point3D point_temp = v_point_full[i];
                point_temp.alpha_time = (point_temp.relative_time / dt_last_point);

                if (point_temp.alpha_time > 1) point_temp.alpha_time = 1;
                if (point_temp.alpha_time < 0) point_temp.alpha_time = 0;

                v_cloud_out.push_back(point_temp);
            }
        }
    }

    dt_offset = dt_last_point;
}

void cloudProcessing::velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg, std::vector<point3D> &v_cloud_out, double &dt_offset)
{
    pcl::PointCloud<velodyne_ros::Point> raw_cloud;
    pcl::fromROSMsg(*msg, raw_cloud);
    int size = raw_cloud.points.size();

    double dt_last_point;

    if(size == 0)
    {
        dt_offset = 1000.0 / double(SCAN_RATE);

    	  return;
    }

    if (raw_cloud.points[size - 1].time > 0)
        given_offset_time = true;
    else
        given_offset_time = false;

    if(given_offset_time)
    {
        sort(raw_cloud.points.begin(), raw_cloud.points.end(), time_list_velodyne);

        // KAIST LiDAR's relative timestamp > 0.1s
        while ((raw_cloud.points[size - 1].time >= 0.1)&&(time_unit == SEC)) 
        {
            size--;
            raw_cloud.points.pop_back();//删除最后一个元素
        }
        // KAIST LiDAR's relative timestamp > 0.1s

        dt_last_point = raw_cloud.points.back().time * time_unit_scale;
    }

    double omega = 0.361 * SCAN_RATE;

    std::vector<bool> is_first;
    is_first.resize(N_SCANS);
    fill(is_first.begin(), is_first.end(), true);

    std::vector<double> yaw_first_point;
    yaw_first_point.resize(N_SCANS);
    fill(yaw_first_point.begin(), yaw_first_point.end(), 0.0);

    std::vector<point3D> v_point_full;

    for(int i = 0; i < size; i++)
    {
        point3D point_temp;

        point_temp.raw_point = Eigen::Vector3d(raw_cloud.points[i].x, raw_cloud.points[i].y, raw_cloud.points[i].z);
        point_temp.point = point_temp.raw_point;
        point_temp.relative_time = raw_cloud.points[i].time * time_unit_scale;

        if(!given_offset_time)
		    {
            int layer = raw_cloud.points[i].ring;
            double yaw_angle = atan2(point_temp.raw_point.y(), point_temp.raw_point.x()) * 57.2957;

            if (is_first[layer])
            {
				        yaw_first_point[layer] = yaw_angle;
				        is_first[layer] = false;
				        point_temp.relative_time = 0.0;

				        v_point_full.push_back(point_temp);

				        continue;
            }

            if (yaw_angle <= yaw_first_point[layer])
            {
				        point_temp.relative_time = (yaw_first_point[layer] - yaw_angle) / omega;
            }
            else
            {
				      point_temp.relative_time = (yaw_first_point[layer] - yaw_angle + 360.0) / omega;
            }

            point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
            v_point_full.push_back(point_temp);
        }

        if(given_offset_time && i % point_filter_num == 0)
        {
            if(point_temp.raw_point.x() * point_temp.raw_point.x() + point_temp.raw_point.y() * point_temp.raw_point.y()
              + point_temp.raw_point.z() * point_temp.raw_point.z() > (blind * blind))
            {
                point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
                point_temp.alpha_time = point_temp.relative_time / dt_last_point;

                v_cloud_out.push_back(point_temp);
            }
        }
    }

    if(!given_offset_time)
    {
        assert(v_point_full.size() == size);

        sort(v_point_full.begin(), v_point_full.end(), time_list);
        dt_last_point = v_point_full.back().relative_time;

        for(int i = 0; i < size; i++)
        {
            if(i % point_filter_num == 0)
            {
                point3D point_temp = v_point_full[i];
                point_temp.alpha_time = (point_temp.relative_time / dt_last_point);

                if(point_temp.alpha_time > 1) point_temp.alpha_time = 1;
                if(point_temp.alpha_time < 0) point_temp.alpha_time = 0;

                v_cloud_out.push_back(point_temp);
            }
        }
    }

    dt_offset = dt_last_point;
}//整体而言,这段代码实现了Velodyne雷达传感器数据处理功能,主要有以下几个功能模块组成: 雷达传感器原始消息转换成PointCloud2格式 对比较老式雷达设备在距离测量上存在误差问题,通过前面两帧激光束触发信号角度差构建恢复因子,修正单帧激光束测距 对重复探测目标制作动态衰减系数因子,加入强度衰减阈值
//总结: 类型主要涉及Pcl库定义CloudProcessing类及ros相关头文件定义sensor_msgs命名空间内PointCloud2类;结构体包括point3D; 函数库有Pcl库、Eigen库

void cloudProcessing::robosenseHandler(const sensor_msgs::PointCloud2::ConstPtr &msg, std::vector<point3D> &v_cloud_out, double &dt_offset)
{
    pcl::PointCloud<robosense_ros::Point> raw_cloud;
    pcl::fromROSMsg(*msg, raw_cloud);
    int size = raw_cloud.points.size();

    double dt_last_point;

    if (size == 0)
    {
        dt_offset = 1000.0 / double(SCAN_RATE);

        return;
    }

    if (raw_cloud.points[size - 1].timestamp > 0)//访问点云最后一个点的时间戳
        given_offset_time = true;
    else
        given_offset_time = false;//检查点云的最后一个点的时间戳t是否大于0

    if (given_offset_time)
    {
        sort(raw_cloud.points.begin(), raw_cloud.points.end(), time_list_robosense);
        dt_last_point = (raw_cloud.points.back().timestamp - raw_cloud.points.front().timestamp) * time_unit_scale;
    }

    double omega = 0.361 * SCAN_RATE;

    std::vector<bool> is_first;
    is_first.resize(N_SCANS);
    fill(is_first.begin(), is_first.end(), true);

    std::vector<double> yaw_first_point;
    yaw_first_point.resize(N_SCANS);
    fill(yaw_first_point.begin(), yaw_first_point.end(), 0.0);

    std::vector<point3D> v_point_full;

    for (int i = 0; i < size; i++)
    {
        point3D point_temp;

        point_temp.raw_point = Eigen::Vector3d(raw_cloud.points[i].x, raw_cloud.points[i].y, raw_cloud.points[i].z);
        point_temp.point = point_temp.raw_point;
        point_temp.relative_time = (raw_cloud.points[i].timestamp - raw_cloud.points.front().timestamp) * time_unit_scale;

        if (!given_offset_time)
        {
            int layer = raw_cloud.points[i].ring;
            double yaw_angle = atan2(point_temp.raw_point.y(), point_temp.raw_point.x()) * 57.2957;

            if (is_first[layer])
            {
                yaw_first_point[layer] = yaw_angle;
                is_first[layer] = false;
                point_temp.relative_time = 0.0;

                v_point_full.push_back(point_temp);

                continue;
            }

            if (yaw_angle <= yaw_first_point[layer])
            {
                point_temp.relative_time = (yaw_first_point[layer] - yaw_angle) / omega;
            }
            else
            {
                point_temp.relative_time = (yaw_first_point[layer] - yaw_angle + 360.0) / omega;
            }

            point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
            v_point_full.push_back(point_temp);
        }

        if (given_offset_time && i % point_filter_num == 0)
        {
            if (point_temp.raw_point.x() * point_temp.raw_point.x() + point_temp.raw_point.y() * point_temp.raw_point.y() + point_temp.raw_point.z() * point_temp.raw_point.z() > (blind * blind))
            {
                point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
                point_temp.alpha_time = point_temp.relative_time / dt_last_point;

                v_cloud_out.push_back(point_temp);
            }
        }
    }

    if (!given_offset_time)
    {
        assert(v_point_full.size() == size);

        sort(v_point_full.begin(), v_point_full.end(), time_list);
        dt_last_point = v_point_full.back().relative_time;

        for (int i = 0; i < size; i++)
        {
            if (i % point_filter_num == 0)
            {
                point3D point_temp = v_point_full[i];
                point_temp.alpha_time = (point_temp.relative_time / dt_last_point);

                if (point_temp.alpha_time > 1) point_temp.alpha_time = 1;
                if (point_temp.alpha_time < 0) point_temp.alpha_time = 0;

                v_cloud_out.push_back(point_temp);
            }
        }
    }

    dt_offset = dt_last_point;
}