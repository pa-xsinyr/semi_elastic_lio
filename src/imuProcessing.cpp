#include "imuProcessing.h"

state::state(const Eigen::Quaterniond &rotation_, const Eigen::Vector3d &translation_, 
        const Eigen::Vector3d &velocity_, const Eigen::Vector3d& ba_, const Eigen::Vector3d& bg_)
    : rotation{rotation_}, translation{translation_}, velocity{velocity_}, ba{ba_}, bg{bg_}
{
    dt_buf.push_back(0);
    rot_buf.push_back(rotation);
    trans_buf.push_back(translation);
    velo_buf.push_back(velocity);

    pre_integration = nullptr;
}//这段代码的目的是初始化state类的实例，设置其初始状态，并准备好用于存储历史数据的缓冲区。

state::state(const state* state_temp, bool copy)
{
    if(copy)
    {
        rotation = state_temp->rotation;
        translation = state_temp->translation;

        rotation_begin = state_temp->rotation_begin;
        translation_begin = state_temp->translation_begin;

        velocity = state_temp->velocity;
        ba = state_temp->ba;
        bg = state_temp->bg;

        velocity_begin = state_temp->velocity_begin;
        ba_begin = state_temp->ba_begin;
        bg_begin = state_temp->bg_begin;

        pre_integration = new imuIntegration(state_temp->pre_integration, 
            state_temp->pre_integration->linearized_acc, state_temp->pre_integration->linearized_gyr);

        dt_buf.insert(dt_buf.end(), state_temp->dt_buf.begin(), state_temp->dt_buf.end());
        rot_buf.insert(rot_buf.end(), state_temp->rot_buf.begin(), state_temp->rot_buf.end());
        trans_buf.insert(trans_buf.end(), state_temp->trans_buf.begin(), state_temp->trans_buf.end());
        velo_buf.insert(velo_buf.end(), state_temp->velo_buf.begin(), state_temp->velo_buf.end());
        un_acc_buf.insert(un_acc_buf.end(), state_temp->un_acc_buf.begin(), state_temp->un_acc_buf.end());
        un_omega_buf.insert(un_omega_buf.end(), state_temp->un_omega_buf.begin(), state_temp->un_omega_buf.end());
    }
    else
    {
        rotation_begin = state_temp->rotation;
        translation_begin = state_temp->translation;
        velocity_begin = state_temp->velocity;
        ba_begin = state_temp->ba;
        bg_begin = state_temp->bg;
        
        rotation = state_temp->rotation;
        translation = state_temp->translation;
        velocity = state_temp->velocity;
        ba = state_temp->ba;
        bg = state_temp->bg;

        dt_buf.push_back(0);
        rot_buf.push_back(rotation);
        trans_buf.push_back(translation);
        velo_buf.push_back(velocity);
        un_acc_buf.push_back(state_temp->un_acc_buf.back());
        un_omega_buf.push_back(state_temp->un_omega_buf.back());

        pre_integration = nullptr;
    }
}//这段代码实现了`state`类的构造函数的另一个版本，它接受一个指向另一个`state`对象的指针和一个布尔值参数。
// 如果`copy`为真，表示要进行深拷贝。在这种情况下，它会复制传入的`state_temp`对象的所有属性，并创建一个新的`imuIntegration`对象进行深拷贝。同时，它还将`state_temp`对象的历史数据缓冲区中的数据复制到当前对象的对应缓冲区中。
// 如果`copy`为假，表示要进行浅拷贝。在这种情况下，它会将传入的`state_temp`对象的属性直接复制到当前对象，并初始化历史数据缓冲区的第一个元素。
//无论是深拷贝还是浅拷贝，这个构造函数都是为了创建一个新的`state`对象，并根据参数选择是否复制传入的`state_temp`对象的属性。

void state::release()
{
    if(pre_integration != nullptr)
        pre_integration->release();

    delete pre_integration;//关键字释放内存空间

    pre_integration = nullptr;//设为空指针，避免悬空指针

    std::vector<double>().swap(dt_buf);//匿名的临时对象，空向量。交换，清空dt_buf内存，并且临时向量（空向量）已经销毁了，释放了它的内存。
    std::vector<Eigen::Quaterniond>().swap(rot_buf);
    std::vector<Eigen::Vector3d>().swap(trans_buf);
    std::vector<Eigen::Vector3d>().swap(velo_buf);
    std::vector<Eigen::Vector3d>().swap(un_acc_buf);
    std::vector<Eigen::Vector3d>().swap(un_omega_buf);
}//这段代码定义了`state`类的`release()`函数，用于释放当前对象所占用的资源，并将其成员变量重置为默认值或空状态。
//首先，通过检查`pre_integration`指针是否为空，来决定是否需要调用`pre_integration`对象的`release()`函数释放其所占用的资源。
//接着，通过`delete`关键字释放`pre_integration`指针指向的内存空间，并将`pre_integration`指针设置为`nullptr`，以避免悬空指针。
//然后，通过调用`swap()`函数和临时的空向量来清空`dt_buf`、`rot_buf`、`trans_buf`、`velo_buf`、`un_acc_buf`和`un_omega_buf`，从而释放它们占用的内存空间，并将它们的容量缩减为0。
//总之，这段代码的作用是在释放当前对象时，清理和释放它所占用的资源，以防止内存泄漏和资源泄漏。

imuIntegration::imuIntegration(const Eigen::Vector3d &acc_0_, const Eigen::Vector3d &gyr_0_,
               const Eigen::Vector3d &linearized_ba_, const Eigen::Vector3d &linearized_bg_, //线性化的加速度偏移，线性化的陀螺仪偏移
               const double acc_cov_, const double gyr_cov_, const double b_acc_cov_, const double b_gyr_cov_)
    : acc_0{acc_0_}, gyr_0{gyr_0_}, linearized_acc{acc_0_}, linearized_gyr{gyr_0_},//冒号用于成员初始化列表，在类的构造函数中，用于对类的成员变量进行初始化，这种语法允许在构造函数体执行之前对成员变量进行初始化，可以提高代码的效率和可读性。
      linearized_ba{linearized_ba_}, linearized_bg{linearized_bg_}, 
      acc_cov{acc_cov_}, gyr_cov{gyr_cov_}, b_acc_cov{b_acc_cov_}, b_gyr_cov{b_gyr_cov_}, 
        jacobian{Eigen::Matrix<double, 15, 15>::Identity()}, covariance{Eigen::Matrix<double, 15, 15>::Zero()},
      sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()}, delta_g{Eigen::Vector3d::Zero()}//用于存储增量的变量

{
    noise = Eigen::Matrix<double, 18, 18>::Zero();//传感器测量噪声
    noise.block<3, 3>(0, 0) =  (acc_cov * acc_cov) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(3, 3) =  (gyr_cov * gyr_cov) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(6, 6) =  (acc_cov * acc_cov) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(9, 9) =  (gyr_cov * gyr_cov) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(12, 12) =  (b_acc_cov * b_acc_cov) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(15, 15) =  (b_gyr_cov * b_gyr_cov) * Eigen::Matrix3d::Identity();
}//初始化类的对象，设置初始值和噪声方差，并准备好用于存储增量的变量。

imuIntegration::imuIntegration(const imuIntegration* integration_temp, const Eigen::Vector3d &linearized_acc_, const Eigen::Vector3d &linearized_gyr_)
    : linearized_acc{linearized_acc_}, linearized_gyr{linearized_gyr_}//成员初始化列表的语法对成员变量初始化，将参数的值赋给成员变量
{
    dt = integration_temp->dt;//将integration_temp对象各个成员变量的值赋给新的对应成员变量，对对象初始化
    acc_0 = integration_temp->acc_0;
    gyr_0 = integration_temp->gyr_0;
    acc_1 = integration_temp->acc_1;
    gyr_1 = integration_temp->gyr_1;

    linearized_ba = integration_temp->linearized_ba;
    linearized_bg = integration_temp->linearized_bg;

    jacobian = integration_temp->jacobian;
    covariance = integration_temp->covariance;
    step_jacobian = integration_temp->step_jacobian;
    step_V = integration_temp->step_V;
    noise = integration_temp->noise;

    sum_dt = integration_temp->sum_dt;
    delta_p = integration_temp->delta_p;
    delta_q = integration_temp->delta_q;
    delta_v = integration_temp->delta_v;
    delta_g = integration_temp->delta_g;

    dt_buf.insert(dt_buf.end(), integration_temp->dt_buf.begin(), integration_temp->dt_buf.end());//函数，将对象的缓冲区buf的内容插入到新对象对应的缓冲区中，以保留历史数据
    acc_buf.insert(acc_buf.end(), integration_temp->acc_buf.begin(), integration_temp->acc_buf.end());
    gyr_buf.insert(gyr_buf.end(), integration_temp->gyr_buf.begin(), integration_temp->gyr_buf.end());

    acc_cov = integration_temp->acc_cov;
    gyr_cov = integration_temp->gyr_cov;
    b_acc_cov = integration_temp->b_acc_cov;
    b_gyr_cov = integration_temp->b_gyr_cov;
}//在这个场景下，有两个构造函数的存在是为了提供更灵活的对象创建方式和对象复制功能。第一个：用于初始化一个新的对象，通过传递各种参数直接创建一个新的对象。第二个：用于初始化一个新的对象，通过传递另一个对象的指针和其他参数，实现对象的复制功能。这样的构造函数通常用于需要复制现有对象的情况，使得可以轻松地创建一个与现有对象相同状态的新对象。因此，第二个构造函数的存在是为了满足对象复制的需求，以便在需要时创建对象的副本。

void imuIntegration::push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
{
    dt_buf.push_back(dt);
    acc_buf.push_back(acc);
    gyr_buf.push_back(gyr);
    propagate(dt, acc, gyr);
}

void imuIntegration::repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
{//用于重新传播IMU积分过程，即重新计算IMU积分的结果，以更新状态。
    sum_dt = 0.0;//时间总和初始化为0，重新传播需要重新计算时间总和
    acc_0 = linearized_acc;
    gyr_0 = linearized_gyr;//确保重新传播的起始状态与当前状态一致
    delta_p.setZero();//初始化
    delta_q.setIdentity();
    delta_v.setZero();
    linearized_ba = _linearized_ba;更新以传入新值
    linearized_bg = _linearized_bg;
    jacobian.setIdentity();
    covariance.setZero();
    for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
        propagate(dt_buf[i], acc_buf[i], gyr_buf[i]);//遍历存储的时间步长、加速度和角速度数据，并对每个时间步长进行传播操作，使用存储的加速度和角速度数据重新计算IMU积分过程，从而更新状态估计的结果。
}

void imuIntegration::midPointIntegration(double dt_, //时间步长
                        const Eigen::Vector3d &acc_0_, const Eigen::Vector3d &gyr_0_,
                        const Eigen::Vector3d &acc_1_, const Eigen::Vector3d &gyr_1_,//两个时间点的加速度和陀螺仪数据
                        const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,//上一个状态的位移，旋转，速度
                        const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,//线性化的加速度偏移，线性化的陀螺仪偏移
                        Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,//中点法积分后的位移、旋转和速度
                        Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)//是否需要更新雅可比矩阵
{//根据 IMU 传感器的加速度和陀螺仪数据，以及其他参数，进行中点法积分，计算出在一个时间步长内的位姿变化和状态更新。如果需要更新雅可比矩阵，则根据状态更新计算相应的雅可比矩阵和协方差矩阵，以用于后续的状态估计或滤波算法。
    Eigen::Vector3d un_acc_0 = delta_q * (acc_0_ - linearized_ba);//利用当前旋转计算非线性加速度
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0_ + gyr_1_) - linearized_bg;//两个时刻的陀螺仪数据的平均值，减去线性化的bg
    result_delta_q = delta_q * Eigen::Quaterniond(1, un_gyr(0) * dt_ / 2, un_gyr(1) * dt_ / 2, un_gyr(2) * dt_ / 2);//根据非线性的陀螺仪数据计算姿态更新的增量。中点法积分的公式
    Eigen::Vector3d un_acc_1 = result_delta_q * (acc_1_ - linearized_ba);//根据姿态更新的增量和线性化的加速度偏移，计算当前时刻的非线性加速度
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    result_delta_p = delta_p + delta_v * dt_ + 0.5 * un_acc * dt_ * dt_;
    result_delta_v = delta_v + un_acc * dt_;//根据两个时刻的非线性加速度的平均值，计算位移的更新增量和速度的更新增量
    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;

    if(update_jacobian)
    {
        Eigen::Vector3d omega = 0.5 * (gyr_0_ + gyr_1_) - linearized_bg;
        Eigen::Vector3d acc0_m = acc_0_ - linearized_ba;
        Eigen::Vector3d acc1_m = acc_1_ - linearized_ba;
        Eigen::Matrix3d R_omega_x, R_acc0_x, R_acc1_x;

        R_omega_x << 0, -omega(2), omega(1), omega(2), 0, -omega(0), -omega(1), omega(0), 0;
        R_acc0_x << 0, -acc0_m(2), acc0_m(1), acc0_m(2), 0, -acc0_m(0), -acc0_m(1), acc0_m(0), 0;
        R_acc1_x << 0, -acc1_m(2), acc1_m(1), acc1_m(2), 0, -acc1_m(0), -acc1_m(1), acc1_m(0), 0;

        Eigen::MatrixXd F_x = Eigen::MatrixXd::Zero(15, 15);
        F_x.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        F_x.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_acc0_x * dt_ * dt_ + 
                              -0.25 * result_delta_q.toRotationMatrix() * R_acc1_x * (Eigen::Matrix3d::Identity() - R_omega_x * dt_) * dt_ * dt_;
        F_x.block<3, 3>(0, 6) = Eigen::MatrixXd::Identity(3, 3) * dt_;
        F_x.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * dt_ * dt_;
        F_x.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_acc1_x * dt_ * dt_ * -dt_;
        F_x.block<3, 3>(3, 3) = Eigen::MatrixXd::Identity(3, 3) - R_omega_x * dt_;
        F_x.block<3, 3>(3, 12) = -1.0 * Eigen::MatrixXd::Identity(3, 3) * dt_;
        F_x.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_acc0_x * dt_ + 
                                -0.5 * result_delta_q.toRotationMatrix() * R_acc1_x * (Eigen::Matrix3d::Identity() - R_omega_x * dt_) * dt_;
        F_x.block<3, 3>(6, 6) = Eigen::MatrixXd::Identity(3, 3);
        F_x.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * dt_;
        F_x.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_acc1_x * dt_ * -dt_;
        F_x.block<3, 3>(9, 9) = Eigen::MatrixXd::Identity(3, 3);
        F_x.block<3, 3>(12, 12) = Eigen::MatrixXd::Identity(3, 3);

        Eigen::MatrixXd F_w = Eigen::MatrixXd::Zero(15, 18);
        F_w.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * dt_ * dt_;
        F_w.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() * R_acc1_x  * dt_ * dt_ * 0.5 * dt_;
        F_w.block<3, 3>(0, 6) =  0.25 * result_delta_q.toRotationMatrix() * dt_ * dt_;
        F_w.block<3, 3>(0, 9) =  F_w.block<3, 3>(0, 3);
        F_w.block<3, 3>(3, 3) =  0.5 * Eigen::MatrixXd::Identity(3, 3) * dt_;
        F_w.block<3, 3>(3, 9) =  0.5 * Eigen::MatrixXd::Identity(3, 3) * dt_;
        F_w.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * dt_;
        F_w.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() * R_acc1_x  * dt_ * 0.5 * dt_;
        F_w.block<3, 3>(6, 6) =  0.5 * result_delta_q.toRotationMatrix() * dt_;
        F_w.block<3, 3>(6, 9) =  F_w.block<3, 3>(6, 3);
        F_w.block<3, 3>(9, 12) = Eigen::MatrixXd::Identity(3, 3) * dt_;
        F_w.block<3, 3>(12, 15) = Eigen::MatrixXd::Identity(3, 3) * dt_;

        jacobian = F_x * jacobian;
        covariance = F_x * covariance * F_x.transpose() + F_w * noise * F_w.transpose();
    }
}

void imuIntegration::propagate(double dt_, const Eigen::Vector3d &acc_1_, const Eigen::Vector3d &gyr_1_)
{
    dt = dt_;
    acc_1 = acc_1_;
    gyr_1 = gyr_1_;
    Eigen::Vector3d result_delta_p;
    Eigen::Quaterniond result_delta_q;
    Eigen::Vector3d result_delta_v;
    Eigen::Vector3d result_linearized_ba;
    Eigen::Vector3d result_linearized_bg;

    midPointIntegration(dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v,
                        linearized_ba, linearized_bg,
                        result_delta_p, result_delta_q, result_delta_v,
                        result_linearized_ba, result_linearized_bg, true);

    delta_p = result_delta_p;
    delta_q = result_delta_q;
    delta_v = result_delta_v;
    linearized_ba = result_linearized_ba;
    linearized_bg = result_linearized_bg;
    delta_q.normalize();
    sum_dt += dt;
    acc_0 = acc_1;
    gyr_0 = gyr_1;  
}

Eigen::Matrix<double, 15, 1> imuIntegration::evaluate(const Eigen::Vector3d &p_last, const Eigen::Quaterniond &q_last, const Eigen::Vector3d &v_last, const Eigen::Vector3d &ba_last, 
	const Eigen::Vector3d &bg_last, const Eigen::Vector3d &p_cur, const Eigen::Quaterniond &q_cur, const Eigen::Vector3d &v_cur, const Eigen::Vector3d &ba_cur, const Eigen::Vector3d &bg_cur)
{
    Eigen::Matrix<double, 15, 1> residuals;

    Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
    Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

    Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

    Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
    Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

    Eigen::Vector3d dba = ba_last - linearized_ba;
    Eigen::Vector3d dbg = bg_last - linearized_bg;

    Eigen::Quaterniond corrected_delta_q = delta_q * numType::deltaQ(dq_dbg * dbg);
    Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
    Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

    residuals.block<3, 1>(O_P, 0) = q_last.inverse() * (0.5 * G * sum_dt * sum_dt + p_cur - p_last - v_last * sum_dt) - corrected_delta_p;
    residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (q_last.inverse() * q_cur)).vec();
    residuals.block<3, 1>(O_V, 0) = q_last.inverse() * (G * sum_dt + v_cur - v_last) - corrected_delta_v;
    residuals.block<3, 1>(O_BA, 0) = ba_cur - ba_last;
    residuals.block<3, 1>(O_BG, 0) = bg_cur - bg_last;
    return residuals;
}

void imuIntegration::release()
{
    std::vector<double>().swap(dt_buf);
    std::vector<Eigen::Vector3d>().swap(acc_buf);
    std::vector<Eigen::Vector3d>().swap(gyr_buf);
}





imuProcessing::imuProcessing()
{
	first_imu = false;

	current_state = new state(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
	last_state = nullptr;

	acc_0 = Eigen::Vector3d::Zero();
	gyr_0 = Eigen::Vector3d::Zero();
}

void imuProcessing::setAccCov(double para)
{
	acc_cov = para;
}

void imuProcessing::setGyrCov(double para)
{
	gyr_cov = para;
}

void imuProcessing::setBiasAccCov(double para)
{
	b_acc_cov = para;
}

void imuProcessing::setBiasGyrCov(double para)
{
	b_gyr_cov = para;
}

void imuProcessing::setExtrinR(Eigen::Matrix3d &R)
{
	R_imu_lidar = R;
}

void imuProcessing::setExtrinT(Eigen::Vector3d &t)
{
	t_imu_lidar = t;
}

void imuProcessing::process(double dt, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity, double timestamp)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;

        Eigen::Vector3d un_acc_0_temp = current_state->rotation * (acc_0 - current_state->ba) - G;
        Eigen::Vector3d un_gyr_temp = 0.5 * (gyr_0 + angular_velocity) - current_state->bg;
        Eigen::Quaterniond rotation_temp = current_state->rotation * numType::deltaQ(un_gyr_temp * dt);
        Eigen::Vector3d un_acc_1_temp = rotation_temp * (linear_acceleration - current_state->ba) - G;
        Eigen::Vector3d un_acc_temp = 0.5 * (un_acc_0_temp + un_acc_1_temp);

        current_state->un_acc_buf.push_back(un_acc_temp);
        current_state->un_omega_buf.push_back(un_gyr_temp);

        assert(current_state->un_acc_buf.size() == current_state->trans_buf.size());
        assert(current_state->un_omega_buf.size() == current_state->rot_buf.size());
    }

    if (!current_state->pre_integration)
        current_state->pre_integration = new imuIntegration(acc_0, gyr_0, current_state->ba, current_state->bg, acc_cov, gyr_cov, b_acc_cov, b_gyr_cov);

    current_state->pre_integration->push_back(dt, linear_acceleration, angular_velocity);

    Eigen::Vector3d un_acc_0 = current_state->rotation * (acc_0 - current_state->ba) - G;
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - current_state->bg;
    current_state->rotation *= numType::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = current_state->rotation * (linear_acceleration - current_state->ba) - G;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    current_state->translation += dt * current_state->velocity + 0.5 * dt * dt * un_acc;
    current_state->velocity += dt * un_acc;

    current_state->dt_buf.push_back(dt + current_state->dt_buf.back());
    current_state->rot_buf.push_back(current_state->rotation);
    current_state->trans_buf.push_back(current_state->translation);
    current_state->velo_buf.push_back(current_state->velocity);
    current_state->un_acc_buf.push_back(un_acc);
    current_state->un_omega_buf.push_back(un_gyr);

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}