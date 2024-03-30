#include "imuFactor.h"
//这个文件是IMU、CT-IMU分别的因子图优化

ImuFactor::ImuFactor(imuIntegration* pre_integration_, state* last_state_) : pre_integration(pre_integration_)//将指针赋值给成员变量
{//用于处理与IMU相关的因子图优化
	rot_last = last_state_->rotation;
	tran_last = last_state_->translation;
	velocity_last = last_state_->velocity;
	ba_last = last_state_->ba;
	bg_last = last_state_->bg;
}//成员变量

bool ImuFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{

    Eigen::Vector3d tran_cur(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond rot_cur(parameters[1][3], parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d velocity_cur(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Vector3d ba_cur(parameters[2][3], parameters[2][4], parameters[2][5]);//行列索引
    Eigen::Vector3d bg_cur(parameters[2][6], parameters[2][7], parameters[2][8]);
//从参数中提取当前状态的平移、旋转、速度、加速度计偏置和陀螺仪偏置。
    Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);//数据结构，用于将现有内存映射到EIGEN矩阵或向量
    residual = pre_integration->evaluate(tran_last, rot_last, velocity_last, ba_last, bg_last, tran_cur, rot_cur, velocity_cur, ba_cur, bg_cur);//调用对象的函数，表示IMU测量预测与实际测量之间差异

    Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration->covariance.inverse()).matrixL().transpose();//这段代码涉及了对预积分对象的协方差矩阵进行 LLT 分解，并计算其下三角矩阵的转置
//pre_integration->covariance：这表示从预积分对象 pre_integration 中获取协方差矩阵。
//.inverse()：对协方差矩阵求逆，得到协方差矩阵的逆矩阵。
//Eigen::LLT<Eigen::Matrix<double, 15, 15>>(...)：这是对协方差矩阵的逆矩阵进行 LLT 分解。LLT 分解将协方差矩阵的逆矩阵分解为一个下三角矩阵和它的转置的乘积。
//.matrixL()：获取 LLT 分解得到的下三角矩阵。
//.transpose()：将下三角矩阵进行转置，得到其转置矩阵。
//Eigen::Matrix<double, 15, 15> sqrt_info = ...：定义了一个名为 sqrt_info 的 15x15 的矩阵，并将 LLT 分解得到的下三角矩阵的转置赋值给它。这段代码的目的是通过对预积分对象的协方差矩阵的逆矩阵进行 LLT 分解，得到其下三角矩阵的转置。这个转置矩阵通常被称为信息矩阵（information matrix），用于在后续的优化过程中进行加权。   
    residual = sqrt_info * residual;//将权重9信息矩阵0应用于残差，将其转换为加权残差向量
//这段代码可能属于一个因子图优化框架（例如 g2o），用于计算残差并将信息矩阵应用于优化目的
    if (jacobians)
    {
    	double sum_dt = pre_integration->sum_dt;//积累的时间间隔，用于IMU测量
    	Eigen::Matrix3d dp_dba = pre_integration->jacobian.template block<3, 3>(O_P, O_BA);//一个对象（类或实例），其中包含了与某些计算相关的雅可比矩阵（偏导数）。jacobian 可能是一个矩阵或数据结构，其中存储了与某个问题相关的偏导数信息。.template block<3, 3>(O_P, O_BA) 是对 jacobian 矩阵的一个方法调用。位置p相对于加速度计偏置ba的偏导数。模板函数bool，行列的起始索引，这段代码从 jacobian 矩阵中提取了一个 3x3 的子矩阵，其起始行和列索引由 O_P 和 O_BA 指定
        Eigen::Matrix3d dp_dbg = pre_integration->jacobian.template block<3, 3>(O_P, O_BG);
        Eigen::Matrix3d dq_dbg = pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
        Eigen::Matrix3d dv_dba = pre_integration->jacobian.template block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = pre_integration->jacobian.template block<3, 3>(O_V, O_BG);
//提取与不同参数相关的雅可比矩阵（偏导数）。PQV：位移，旋转，速度
        if (pre_integration->jacobian.maxCoeff() > 1e8 || pre_integration->jacobian.minCoeff() < -1e8)
        {
            ROS_WARN("numerical unstable in preintegration");
        }//检查雅可比矩阵的最大值和最小值是否超出阈值（1e8）。如果是，打印警告消息，指示在预积分中存在数值不稳定性

        if (jacobians[0])//误差函数residual关于位置p的雅可比,再加权。在因子图优化中，雅可比矩阵描述了误差函数关于优化变量的变化率。在这个情况下，优化变量是位置p
        {//对第一个优化变量组位置p进行雅可比计算
        	Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_tran_cur(jacobians[0]);//15x3的矩阵
        	jacobian_tran_cur.setZero();

            jacobian_tran_cur.block<3, 3>(O_P, O_P) = rot_last.inverse().toRotationMatrix();//上一个状态到当前状态的旋转。求逆得到当前状态到上一个状态的旋转。将逆四元数转换为一个3x3的旋转矩阵。将误差函数关于位置p部分的雅可比矩阵设置为上一个状态旋转矩阵的逆
            jacobian_tran_cur = sqrt_info * jacobian_tran_cur;//用之前计算得到的信息矩阵对雅可比加权
        }
        if (jacobians[1])//误差函数关于旋转矩阵q的雅可比。在因子图优化中，雅可比矩阵描述了误差函数关于优化变量的变化率。在这个情况下，优化变量是旋转q
        {//对于第二个优化变量组，旋转q进行雅可比矩阵计算
        	Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>> jacobian_rot_cur(jacobians[1]);//将创建的jacobian_rot_cur映射到指针数组jacobians[1]指向的内存块中
//15x4的矩阵    
        	jacobian_rot_cur.setZero();

        	Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * numType::deltaQ(dq_dbg * (bg_last - pre_integration->linearized_bg));
//修正后的增量四元数。pre_integration对象的delta_q。乘以。旋转对bg的偏导，线性化的bg。计算由于偏置校正引起的旋转变化。计算四元数表示。     	
            jacobian_rot_cur.block<3, 3>(O_R, O_R - O_R) = numType::Qleft(corrected_delta_q.inverse() * rot_last.inverse() * rot_cur).bottomRightCorner<3, 3>();
//残差相对于旋转参数的偏导数。起始索引。旋转导数的第一列。整体的旋转变化。提取3x3块，位于右下角，用于从旋转矩阵中提取旋转部分，旋转矩阵的右下角就是旋转部分。numType::Qleft 可能是一个自定义的函数或函数模板，用于计算四元数左乘操作的结果。其输入参数可能包括一个四元数 q 和一个三维矢量 v，然后返回左乘操作的结果，是一个旋转后的矢量    	
            jacobian_rot_cur = sqrt_info * jacobian_rot_cur;
        }
        if (jacobians[2])//误差函数关于v,ba,bg的雅可比。在因子图优化中，雅可比矩阵描述了误差函数关于优化变量的变化率。在这个情况下，优化变量是速度 v 以及加速度计偏置ba和陀螺仪偏置 bg.表示对于第三个优化变量组
        {
        	Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_velocity_bias_cur(jacobians[2]);
            jacobian_velocity_bias_cur.setZero();

            jacobian_velocity_bias_cur.block<3, 3>(O_V, O_V - O_V) = rot_last.inverse().toRotationMatrix();//将误差函数关于速度v的部分的雅可比矩阵设置为旋转矩阵R-1,R是上一个状态的旋转矩阵
            jacobian_velocity_bias_cur.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();//将残差关于ba的雅可比设置为单位矩阵
            jacobian_velocity_bias_cur.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();//将残差关于bg的雅可比设置为单位矩阵
            jacobian_velocity_bias_cur = sqrt_info * jacobian_velocity_bias_cur;//考虑优化过程中的权重
        }//jacobian_velocity_bias_cur 是一个 15x9 的矩阵，表示误差函数关于v,ba,bg 的雅可比矩阵
    }

    return true;
}//这段代码实现了 IMU 因子的评估函数，其中包括计算残差和雅可比矩阵。主要用于处理与 IMU 相关的因子图优化。它计算了 IMU 测量的预测值与实际测量之间的差异，并根据预积分对象的协方差矩阵对这些差异进行了加权，以便在后续的优化过程中更准确地估计系统状态。

CTImuFactor::CTImuFactor(imuIntegration* pre_integration_, int beta_) : pre_integration(pre_integration_), beta(beta_)//赋值给成员变量
{

}

bool CTImuFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{

	Eigen::Vector3d tran_last(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond rot_last(parameters[1][3], parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d velocity_last(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Vector3d ba_last(parameters[2][3], parameters[2][4], parameters[2][5]);
    Eigen::Vector3d bg_last(parameters[2][6], parameters[2][7], parameters[2][8]);

    Eigen::Vector3d tran_cur(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Quaterniond rot_cur(parameters[4][3], parameters[4][0], parameters[4][1], parameters[4][2]);
    Eigen::Vector3d velocity_cur(parameters[5][0], parameters[5][1], parameters[5][2]);
    Eigen::Vector3d ba_cur(parameters[5][3], parameters[5][4], parameters[5][5]);
    Eigen::Vector3d bg_cur(parameters[5][6], parameters[5][7], parameters[5][8]);
//从参数中提取上一个状态和当前状态的平移、旋转、速度、加速度计偏置和陀螺仪偏置
    Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
    residual = pre_integration->evaluate(tran_last, rot_last, velocity_last, ba_last, bg_last, tran_cur, rot_cur, velocity_cur, ba_cur, bg_cur);
//使用预积分对象的 evaluate 函数计算 CT-IMU 测量预测与实际测量之间的残差，并将结果赋值给 residuals
    Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration->covariance.inverse()).matrixL().transpose();
//对预积分对象的协方差矩阵的逆矩阵进行 LLT 分解，然后计算其下三角矩阵的转置，并将结果赋值给 sqrt_info。这个步骤用于计算信息矩阵，以便在后续的优化过程中进行加权    
    residual = sqrt_info * residual * beta;//将残差向量应用加权，即将信息矩阵乘以残差向量，再乘以权重系数 beta

    if (jacobians)
    {
    	double sum_dt = pre_integration->sum_dt;
    	Eigen::Matrix3d dp_dba = pre_integration->jacobian.template block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = pre_integration->jacobian.template block<3, 3>(O_P, O_BG);
        Eigen::Matrix3d dq_dbg = pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
        Eigen::Matrix3d dv_dba = pre_integration->jacobian.template block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = pre_integration->jacobian.template block<3, 3>(O_V, O_BG);

        if (pre_integration->jacobian.maxCoeff() > 1e8 || pre_integration->jacobian.minCoeff() < -1e8)
        {
            ROS_WARN("numerical unstable in preintegration");
        }

        if (jacobians[0])//误差函数关于位移变量的雅可比。在因子图优化中，雅可比矩阵描述了误差函数关于优化变量的变化率。在这个情况下，优化变量是位移。 表示对于第一个优化变量组，即位移，进行雅可比矩阵的计算
        {
            Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_tran_last(jacobians[0]);//15x3矩阵
            jacobian_tran_last.setZero();

            jacobian_tran_last.block<3, 3>(O_P, O_P) = - rot_last.inverse().toRotationMatrix();//将误差函数关于位移的部分的雅可比矩阵设置为旋转矩阵的负逆矩阵，表示误差函数对于位移的偏导数为旋转矩阵的负逆矩阵
            jacobian_tran_last = sqrt_info * jacobian_tran_last * beta;//为了考虑优化过程中的权重
        }
        if (jacobians[1])//计算的是误差函数关于旋转变量（姿态）的雅可比矩阵。表示对于第二个优化变量组，即旋转，进行雅可比矩阵的计算
        {
            Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>> jacobian_rot_last(jacobians[1]);//15x4
            jacobian_rot_last.setZero();

            jacobian_rot_last.block<3, 3>(O_P, O_R - O_R) = numType::skewSymmetric(rot_last.inverse() * (0.5 * G * sum_dt * sum_dt + tran_cur - tran_last - velocity_last * sum_dt));//将误差函数关于旋转的部分的雅可比矩阵设置为对旋转变量（姿态）的扰动造成的误差的贡献。这部分计算基于误差函数的定义和误差项的求导规则。
            Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * numType::deltaQ(dq_dbg * (bg_last - pre_integration->linearized_bg));
            jacobian_rot_last.block<3, 3>(O_R, O_R - O_R) = - (numType::Qleft(rot_cur.inverse() * rot_last) * numType::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
            jacobian_rot_last.block<3, 3>(O_V, O_R - O_R) = numType::skewSymmetric(rot_last.inverse() * (G * sum_dt + velocity_cur - velocity_last));
            jacobian_rot_last = sqrt_info * jacobian_rot_last * beta;
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_velocity_bias_last(jacobians[2]);
            jacobian_velocity_bias_last.setZero();
            jacobian_velocity_bias_last.block<3, 3>(O_P, O_V - O_V) = - rot_last.inverse().toRotationMatrix() * sum_dt;
            jacobian_velocity_bias_last.block<3, 3>(O_P, O_BA - O_V) = - dp_dba;
            jacobian_velocity_bias_last.block<3, 3>(O_P, O_BG - O_V) = - dp_dbg;

            jacobian_velocity_bias_last.block<3, 3>(O_R, O_BG - O_V) = - numType::Qleft(rot_cur.inverse() * rot_last * pre_integration->delta_q).bottomRightCorner<3, 3>() * dq_dbg;

            jacobian_velocity_bias_last.block<3, 3>(O_V, O_V - O_V) = - rot_last.inverse().toRotationMatrix();
            jacobian_velocity_bias_last.block<3, 3>(O_V, O_BA - O_V) = - dv_dba;
            jacobian_velocity_bias_last.block<3, 3>(O_V, O_BG - O_V) = - dv_dbg;

            jacobian_velocity_bias_last.block<3, 3>(O_BA, O_BA - O_V) = - Eigen::Matrix3d::Identity();

            jacobian_velocity_bias_last.block<3, 3>(O_BG, O_BG - O_V) = - Eigen::Matrix3d::Identity();

            jacobian_velocity_bias_last = sqrt_info * jacobian_velocity_bias_last * beta;
        }
        if (jacobians[3])
        {
        	Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_tran_cur(jacobians[3]);
        	jacobian_tran_cur.setZero();

            jacobian_tran_cur.block<3, 3>(O_P, O_P) = rot_last.inverse().toRotationMatrix();
            jacobian_tran_cur = sqrt_info * jacobian_tran_cur * beta;
        }
        if (jacobians[4])
        {
        	Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>> jacobian_rot_cur(jacobians[4]);
        	jacobian_rot_cur.setZero();

        	Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * numType::deltaQ(dq_dbg * (bg_last - pre_integration->linearized_bg));
        	jacobian_rot_cur.block<3, 3>(O_R, O_R - O_R) = numType::Qleft(corrected_delta_q.inverse() * rot_last.inverse() * rot_cur).bottomRightCorner<3, 3>();
        	jacobian_rot_cur = sqrt_info * jacobian_rot_cur * beta;
        }
        if (jacobians[5])
        {
        	Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_velocity_bias_cur(jacobians[5]);
            jacobian_velocity_bias_cur.setZero();

            jacobian_velocity_bias_cur.block<3, 3>(O_V, O_V - O_V) = rot_last.inverse().toRotationMatrix();
            jacobian_velocity_bias_cur.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();
            jacobian_velocity_bias_cur.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();
            jacobian_velocity_bias_cur = sqrt_info * jacobian_velocity_bias_cur * beta;
        }
    }

    return true;
}//如果需要计算雅可比矩阵，则：
//提取预积分对象的雅可比矩阵中与位置、姿态和速度相关的部分。
//检查雅可比矩阵是否存在数值不稳定性，并在需要时输出警告信息。
//计算并设置上一个状态的位置和姿态关于加速度计偏置和陀螺仪偏置的雅可比矩阵。
//计算并设置上一个状态的速度关于加速度计偏置和陀螺仪偏置的雅可比矩阵。
//计算并设置当前状态的位置和姿态关于加速度计偏置和陀螺仪偏置的雅可比矩阵。
//计算并设置当前状态的速度关于加速度计偏置和陀螺仪偏置的雅可比矩阵。
//最后，返回 true 表示评估成功
//这段代码主要用于处理与 CT-IMU 相关的因子图优化。它计算了 CT-IMU 测量的预测值与实际测量之间的差异，并根据预积分对象的协方差矩阵对这些差异进行了加权，以便在后续的优化过程中更准确地估计系统状态
