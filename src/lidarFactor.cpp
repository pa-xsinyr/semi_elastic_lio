#include "lidarFactor.h"

double LidarPlaneNormFactor::sqrt_info;//类，成员变量，激光雷达平面法向量
Eigen::Vector3d LidarPlaneNormFactor::t_il;//
Eigen::Quaterniond LidarPlaneNormFactor::q_il;

double CTLidarPlaneNormFactor::sqrt_info;
Eigen::Vector3d CTLidarPlaneNormFactor::t_il;
Eigen::Quaterniond CTLidarPlaneNormFactor::q_il;

LidarPlaneNormFactor::LidarPlaneNormFactor(const Eigen::Vector3d &point_body_, const Eigen::Vector3d &norm_vector_, const double norm_offset_, double weight_)
     : point_body(point_body_), norm_vector(norm_vector_), norm_offset(norm_offset_), weight(weight_)//类名，构造函数，参数，激光雷达点的三维坐标，法向量的三维坐标，法向量偏移量，权重。将传入的参数赋值给类的成员变量，初始化成员变量
{

}

bool LidarPlaneNormFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const//成员函数。指向指针数组的指针，两个指针。平移向量，旋转四元数。平移和旋转参数的雅可比矩阵
{

    Eigen::Vector3d translation(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond rotation(parameters[1][3], parameters[1][0], parameters[1][1], parameters[1][2]);//提取旋转四元数

    Eigen::Vector3d point_world = rotation * point_body + translation;//转换到世界坐标系
    double distance = norm_vector.dot(point_world) + norm_offset;//计算投影。计算点到法向量的距离

    residuals[0] = sqrt_info * weight * distance;


    if (jacobians)
    {
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian_tran(jacobians[0]);
            jacobian_tran.setZero();

            jacobian_tran.block<1, 3>(0, 0) = sqrt_info * norm_vector.transpose() * weight;//从 jacobian_tran 矩阵中提取一个1x3的子矩阵，起始位置是第0行第0列，即整个矩阵本身
        }//根据平移参数计算相应的雅可比矩阵
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jacobian_rot(jacobians[1]);
            jacobian_rot.setZero();

            jacobian_rot.block<1, 3>(0, 0) = - sqrt_info * norm_vector.transpose() * rotation.toRotationMatrix() * numType::skewSymmetric(point_body) * weight;//根据旋转参数计算相应的雅可比矩阵
        }
    }

    return true;
}//评估激光平面法向因子，计算残差（residuals）和雅可比矩阵（jacobians）

CTLidarPlaneNormFactor::CTLidarPlaneNormFactor(const Eigen::Vector3d &raw_keypoint_, const Eigen::Vector3d &norm_vector_, const double norm_offset_, double alpha_time_, double weight_)//原始关键点
     : norm_vector(norm_vector_), norm_offset(norm_offset_), alpha_time(alpha_time_), weight(weight_)
{
    raw_keypoint = q_il * raw_keypoint_ + t_il;//从局部转到世界
}

bool CTLidarPlaneNormFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{//用于评估激光平面法向因子的残差和雅可比矩阵

    const Eigen::Vector3d tran_begin(parameters[0][0], parameters[0][1], parameters[0][2]);
    const Eigen::Vector3d tran_end(parameters[2][0], parameters[2][1], parameters[2][2]);
    const Eigen::Quaterniond rot_begin(parameters[1][3], parameters[1][0], parameters[1][1], parameters[1][2]);
    const Eigen::Quaterniond rot_end(parameters[3][3], parameters[3][0], parameters[3][1], parameters[3][2]);
//首先根据输入参数提取了平移和旋转参数
    Eigen::Quaterniond rot_slerp = rot_begin.slerp(alpha_time, rot_end);
    rot_slerp.normalize();
    Eigen::Vector3d tran_slerp = tran_begin * (1 - alpha_time) + tran_end * alpha_time;//根据时间参数进行线性插值，计算得到了平移和旋转的插值结果
    Eigen::Vector3d point_world = rot_slerp * raw_keypoint + tran_slerp;//插值后转到世界

    double distance = norm_vector.dot(point_world) + norm_offset;

    residuals[0] = sqrt_info * weight * distance;


    if (jacobians)
    {
        Eigen::Matrix<double, 1, 3> jacobian_rot_slerp = - norm_vector.transpose() * rot_slerp.toRotationMatrix() * numType::skewSymmetric(raw_keypoint) * weight;

        Eigen::Quaterniond rot_delta = rot_begin.inverse() * rot_end;
        Eigen::Quaterniond rot_identity(Eigen::Matrix3d::Identity());
        Eigen::Quaterniond rot_delta_slerp = rot_identity.slerp(alpha_time, rot_delta);

        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian_tran_begin(jacobians[0]);
            jacobian_tran_begin.setZero();

            jacobian_tran_begin.block<1, 3>(0, 0) = norm_vector.transpose() * weight * (1 - alpha_time);//计算相应雅可比矩阵
            jacobian_tran_begin = sqrt_info * jacobian_tran_begin;
        }
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jacobian_rot_begin(jacobians[1]);
            jacobian_rot_begin.setZero();

            Eigen::Matrix<double, 3, 3> jacobian_slerp_begin = (rot_delta_slerp.toRotationMatrix()).transpose() * (Eigen::Matrix3d::Identity() - alpha_time * numType::Qleft(rot_delta_slerp).bottomRightCorner<3, 3>() * (numType::Qleft(rot_delta).bottomRightCorner<3, 3>()).inverse());
            jacobian_rot_begin.block<1, 3>(0, 0) = jacobian_rot_slerp * jacobian_slerp_begin;

            jacobian_rot_begin = sqrt_info * jacobian_rot_begin;
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian_tran_end(jacobians[2]);
            jacobian_tran_end.setZero();

            jacobian_tran_end.block<1, 3>(0, 0) = norm_vector.transpose() * weight * alpha_time;
            jacobian_tran_end = sqrt_info * jacobian_tran_end;
        }
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jacobian_rot_end(jacobians[3]);
            jacobian_rot_end.setZero();

            Eigen::Matrix<double, 3, 3> jacobian_slerp_end = alpha_time * numType::Qright(rot_delta_slerp).bottomRightCorner<3, 3>() * (numType::Qright(rot_delta).bottomRightCorner<3, 3>()).inverse();
            jacobian_rot_end.block<1, 3>(0, 0) = jacobian_rot_slerp * jacobian_slerp_end;

            jacobian_rot_end = sqrt_info * jacobian_rot_end;
        }
    }

    return true;
}//整个类的作用是用于在激光SLAM系统中评估激光平面法向因子的残差和雅可比矩阵，从而在优化过程中进行参数优化。★

LocationConsistencyFactor::LocationConsistencyFactor(const Eigen::Vector3d &previous_location_, double beta_)//输入的先前位置，权重系数
{
    previous_location = previous_location_;
    beta = beta_;
}

bool LocationConsistencyFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
{//评估位置一致性因子的残差和雅可比矩阵
    residuals[0] = beta * (parameters[0][0] - previous_location(0, 0));
    residuals[1] = beta * (parameters[0][1] - previous_location(1, 0));
    residuals[2] = beta * (parameters[0][2] - previous_location(2, 0));

    if (jacobians)
    {
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_tran_begin(jacobians[0]);
            jacobian_tran_begin.setZero();

            jacobian_tran_begin(0, 0) = beta;
            jacobian_tran_begin(1, 1) = beta;
            jacobian_tran_begin(2, 2) = beta;
        }
    }

    return true;
}//该类用于评估位置一致性因子，计算残差（residuals）和雅可比矩阵（jacobians）。用于在优化过程中评估位置一致性因子的残差和雅可比矩阵，从而进行参数优化

RotationConsistencyFactor::RotationConsistencyFactor(const Eigen::Quaterniond &previous_rotation_, double beta_)//先前旋转四元数，权重系数
{
    previous_rotation = previous_rotation_;
    beta = beta_;
}

bool RotationConsistencyFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
{
    Eigen::Quaterniond rot_cur(parameters[0][3], parameters[0][0], parameters[0][1], parameters[0][2]);//当前旋转四元数
    Eigen::Quaterniond q_et = previous_rotation.inverse() * rot_cur;
    Eigen::Vector3d error = 2 * q_et.vec();//转化为向量形式：乘以2取四元数向量部分

    residuals[0] = error[0] * beta;
    residuals[1] = error[1] * beta;
    residuals[2] = error[2] * beta;

    if (jacobians)
    {
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian_rot_begin(jacobians[0]);
            jacobian_rot_begin.setZero();

            jacobian_rot_begin.block<3, 3>(0, 0) = q_et.w() * Eigen::Matrix3d::Identity() + numType::skewSymmetric(q_et.vec());
            jacobian_rot_begin = jacobian_rot_begin * beta;//根据相对四元数 q_et 的公式计算雅可比矩阵，并乘以权重系数
        }
    }

    return true;
}//该类用于评估旋转一致性因子，计算残差（residuals）和雅可比矩阵（jacobians）。在优化过程中评估旋转一致性因子的残差和雅可比矩阵，从而进行参数优化。

SmallVelocityFactor::SmallVelocityFactor(double beta_)
{
    beta = beta_;
}

bool SmallVelocityFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
{
    residuals[0] = beta * (parameters[0][0] - parameters[1][0]);
    residuals[1] = beta * (parameters[0][1] - parameters[1][1]);
    residuals[2] = beta * (parameters[0][2] - parameters[1][2]);

    if (jacobians)
    {
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_tran_begin(jacobians[0]);
            jacobian_tran_begin.setZero();

            jacobian_tran_begin(0, 0) = beta;
            jacobian_tran_begin(1, 1) = beta;
            jacobian_tran_begin(2, 2) = beta;
        }//将权重系数 beta 赋值给矩阵的对角线元素

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_tran_end(jacobians[1]);
            jacobian_tran_end.setZero();

            jacobian_tran_end(0, 0) = -beta;
            jacobian_tran_end(1, 1) = -beta;
            jacobian_tran_end(2, 2) = -beta;//将-beta 赋值给矩阵的对角线元素。
        }
    }

    return true;
}//该类用于评估小速度因子，计算残差（residuals）和雅可比矩阵（jacobians）。

VelocityConsistencyFactor::VelocityConsistencyFactor(state* previous_state_, double beta_)
{
    previous_velocity = previous_state_->velocity;
    previous_ba = previous_state_->ba;
    previous_bg = previous_state_->bg;
    beta = beta_;
}

bool VelocityConsistencyFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
{//参数是一个双重指针，因为是二维数组，每行代表一个参数块，每列代表参数值。每行代表残差块，每列代表对应参数块的雅可比矩阵
    residuals[0] = beta * (parameters[0][0] - previous_velocity(0, 0));
    residuals[1] = beta * (parameters[0][1] - previous_velocity(1, 0));
    residuals[2] = beta * (parameters[0][2] - previous_velocity(2, 0));
    residuals[3] = beta * (parameters[0][3] - previous_ba(0, 0));
    residuals[4] = beta * (parameters[0][4] - previous_ba(1, 0));
    residuals[5] = beta * (parameters[0][5] - previous_ba(2, 0));
    residuals[6] = beta * (parameters[0][6] - previous_bg(0, 0));
    residuals[7] = beta * (parameters[0][7] - previous_bg(1, 0));
    residuals[8] = beta * (parameters[0][8] - previous_bg(2, 0));

    if (jacobians)
    {
        if (jacobians[0])//检查数组中第一个元素是否为非空指针
        {
            Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor>> jacobian_velocity_bias_begin(jacobians[0]);//指针数组的第一个元素。jacobians[0]是一个指向某个内存块的指针，通过Eigen::Map将这个内存块映射为一个9x9的Eigen矩阵jacobian_velocity_bias_begin。后续对jacobian_velocity_bias_begin的操作实际上就是对jacobians[0]指向的内存块进行操作
            jacobian_velocity_bias_begin.setZero();//所有元素初始化为0

            jacobian_velocity_bias_begin(0, 0) = beta;
            jacobian_velocity_bias_begin(1, 1) = beta;
            jacobian_velocity_bias_begin(2, 2) = beta;
            jacobian_velocity_bias_begin(3, 3) = beta;
            jacobian_velocity_bias_begin(4, 4) = beta;
            jacobian_velocity_bias_begin(5, 5) = beta;
            jacobian_velocity_bias_begin(6, 6) = beta;
            jacobian_velocity_bias_begin(7, 7) = beta;
            jacobian_velocity_bias_begin(8, 8) = beta;//对角线上元素设置为beta
        }
    }

    return true;
}//用于评估速度一致性因子的残差和雅可比矩阵

void TruncatedLoss::Evaluate(double s, double *rho) const {//截断损失函数的评估方法
    if (s < sigma2_) {
        rho[0] = s;//损失函数
        rho[1] = 1.0;//导数值
        rho[2] = 0.0;//二阶导数
        return;
    }
    rho[0] = sigma2_;
    rho[1] = 0.0;
    rho[2] = 0.0;
}//用于评估损失函数，根据输入的参数 s 的值来计算损失函数的值，并将结果存储在指定的数组 rho 中
//这是论文最核心的那个三部分组成的公式的具体代码内容