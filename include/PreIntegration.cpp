//
// Created by jh on 18-4-18.
//

#include "PreIntegration.h"
#include "parameters.h"
  //注意这里要设置初始变量
PreIntegration::PreIntegration(Vector3d acc_0_,Vector3d acc_1_,Vector3d gyr_0_,Vector3d gyr_1_,Vector3d ba_,Vector3d bg_,double dt_):
        dt(dt_),sum_t(0.0),acc_0(acc_0_),gyr_0(gyr_0_),acc_1(acc_1_),gyr_1(gyr_1_),ba_pre(ba_),bg_pre(bg_),jacobian(Eigen::Matrix<double,15,15>::Identity()),
        covariance(Eigen::Matrix<double,15,15>::Identity()),dp(Eigen::Vector3d::Zero()),dv(Eigen::Vector3d::Zero()),dq(Eigen::Quaterniond::Identity())
{
    noise = Eigen::Matrix<double ,18,18>::Zero();
    noise.block<3,3>(0,0)=(acc_n*acc_n)* Matrix3d::Identity();
    noise.block<3,3>(3,3)=(gyr_n*gyr_n)* Matrix3d::Identity();
    noise.block<6,6>(0,0)=(acc_n*acc_n)* Matrix3d::Identity();
    noise.block<9,9>(3,3)=(gyr_n*gyr_n)* Matrix3d::Identity();
    noise.block<3,3>(12,12)=(acc_w*acc_w)* Matrix3d::Identity();
    noise.block<3,3>(15,15)=(gyr_w*gyr_w)* Matrix3d::Identity();
}

//预积分的主要函数
void PreIntegration::run()
{
    //预积分结果
    Vector3d dp_1,dv_1;
    Quaterniond dq_1;

    dt_buf.push_back(dt);
    acc_buf.push_back(acc_1);
    gyr_buf.push_back(gyr_1);

    Vector3d acc_0_nb=dq*(acc_0-ba_pre);
    Vector3d gyr_aver=0.5*(gyr_0+gyr_1)-bg_pre;

    //中值法
    dq_1=dq*Quaterniond(1,0.5*gyr_aver(0)*dt,0.5*gyr_aver(1)*dt,0.5*gyr_aver(2)*dt);
    Vector3d acc_1_nb=dq*(acc_1-ba_pre);
    dp_1=dp+dv*dt+0.5*dt*dt*(0.5*(acc_0_nb+acc_1_nb));
    dv_1=dv+dt*0.5*(acc_0_nb+acc_1_nb);

    //计算更新雅克比矩阵和协方差矩阵
    {


    }









    //更新dq dv dp
    dq=dq_1;
    dp=dp_1;
    dv=dv_1;

}

void PreIntegration::clearState()
{

    dt_buf.clear();
    acc_buf.clear();
    gyr_buf.clear();
}