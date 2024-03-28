#include "poseParameterization.h"

bool PoseParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);//指针操作，获取数组x中的第四个元素。将指针向后移动3个位置。姿态旋转

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    // std::cout << "dp = " << dp.transpose() << std::endl;

    Eigen::Quaterniond dq = numType::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);//使用EIGEN库进行线性代数操作

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
}
bool PoseParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);//7行6列
    j.topRows<6>().setIdentity();//前6行设置为单位矩阵，初始化雅可比的前6个自由度位置和方向
    j.bottomRows<1>().setZero();//最后1行设置为0.额外自由度比如时间

    return true;
}

bool RotationParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Quaterniond> _q(x);

    Eigen::Quaterniond dq = numType::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta));

    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta);

    q = (_q * dq).normalized();

    return true;
}
bool RotationParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
    j.topRows<3>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}