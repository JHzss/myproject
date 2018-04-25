//
// Created by jh on 18-4-18.
//

#include "PreIntegration.h"


  //注意这里要设置初始变量

Eigen::Matrix3d skew(Vector3d vector_)
{
//    Eigen::Matrix3d skew_;
//    skew_<<0,-1*vector_(2),vector_(1),
//        vector_(2),0,-1*vector_(0),
//        -1*vector_(1),vector_(0),0;
    return SO3::hat(vector_);
}

PreIntegration::PreIntegration(Vector3d acc_0_,Vector3d acc_1_,Vector3d gyr_0_,Vector3d gyr_1_,Vector3d ba_,Vector3d bg_,double dt_):
        dt(dt_),sum_t(0.0),acc_0(acc_0_),gyr_0(gyr_0_),acc_1(acc_1_),gyr_1(gyr_1_),ba_pre(ba_),bg_pre(bg_),jacobian(Eigen::Matrix<double,15,15>::Identity()),
        covariance(Eigen::Matrix<double,15,15>::Identity()),dp(Eigen::Vector3d::Zero()),dv(Eigen::Vector3d::Zero()),dq(Eigen::Quaterniond::Identity())
{
    noise = Eigen::Matrix<double ,18,18>::Zero();
    noise.block<3, 3>(0,0)=(acc_n*acc_n)* Eigen::Matrix3d::Identity();
    noise.block<3, 3>(3,3)=(gyr_n*gyr_n)* Matrix3d::Identity();
    noise.block<3,3>(6,6)=(acc_n*acc_n)* Matrix3d::Identity();
    noise.block<3,3>(9,9)=(gyr_n*gyr_n)* Matrix3d::Identity();
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
    cout<<"acc_0:"<<endl<<acc_0<<endl;
    cout<<"acc_1:"<<endl<<acc_1<<endl;
    cout<<"gyr_0:"<<endl<<gyr_0<<endl;
    cout<<"gyr_1:"<<endl<<gyr_1<<endl;

    Vector3d acc_0_nb=dq.toRotationMatrix()*(acc_0 - ba_pre);
    Vector3d gyr_aver=0.5*(gyr_0+gyr_1)-bg_pre;

    //中值法
    dq_1=dq*Quaterniond(1,0.5*gyr_aver(0)*dt,0.5*gyr_aver(1)*dt,0.5*gyr_aver(2)*dt);
    Vector3d acc_1_nb=dq.toRotationMatrix()*(acc_1-ba_pre);
    dp_1=dp+dv*dt+0.5*dt*dt*(0.5*(acc_0_nb+acc_1_nb));
    dv_1=dv+dt*0.5*(acc_0_nb+acc_1_nb);

    //计算更新雅克比矩阵和协方差矩阵
    {
        //雅克比矩阵
        Eigen::MatrixXd F=MatrixXd::Zero(15,15);
        Eigen::MatrixXd V=MatrixXd::Zero(15,18);

        F.block<3,3>(0,0)=Eigen::Matrix3d::Identity();
        F.block<3,3>(0,3)=-0.25*dq.toRotationMatrix()*skew((acc_0-ba_pre))*dt*dt-0.25*dq_1.toRotationMatrix()*skew((acc_0-ba_pre))*(Matrix3d::Identity()-skew((0.5*gyr_0+0.5*gyr_1-bg_pre))*dt)*dt*dt;
        F.block<3,3>(0,6)=Matrix3d::Identity()*dt;
        F.block<3,3>(0,9)=-0.25*(dq_1.toRotationMatrix()+dq.toRotationMatrix())*dt*dt;
        F.block<3,3>(0,12)=0.25*(-1*dq_1.toRotationMatrix()*skew(acc_1-ba_pre)*dt*dt)*(-1*dt);
        F.block<3,3>(3,3)=Matrix3d::Identity()-skew(0.5*gyr_0+0.5*gyr_1-bg_pre)*dt;
        F.block<3,3>(3,12)=Matrix3d::Identity()*(-1)*dt;
        F.block<3,3>(6,3)=-0.5*dq.toRotationMatrix()*skew(acc_0-ba_pre)*dt-0.5*dq_1.toRotationMatrix()*skew(acc_1-ba_pre)*(Matrix3d::Identity()-skew(0.5*gyr_0+0.5*gyr_1-bg_pre)*dt)*dt;
        F.block<3,3>(6,6)=Matrix3d::Identity();
        F.block<3,3>(6,9)=-0.5*(dq_1.toRotationMatrix()+dq.toRotationMatrix())*dt;
        F.block<3,3>(6,12)=0.5*(-1*dq_1.toRotationMatrix()*skew(acc_1-ba_pre)*dt)*(-1*dt);
        F.block<3,3>(9,9)=Matrix3d::Identity();
        F.block<3,3>(12,12)=Matrix3d::Identity();

        //V矩阵
        V.block<3,3>(0,0)=0.25*dq.toRotationMatrix()*dt*dt;
        V.block<3,3>(0,3)=0.25*(-1*dq_1.toRotationMatrix()*skew(acc_1-ba_pre)*dt*dt)*(0.5*dt);
        V.block<3,3>(0,6)=0.25*dq_1.toRotationMatrix()*dt*dt;
        V.block<3,3>(0,9)=0.25*(-1*dq_1.toRotationMatrix()*skew(acc_1-ba_pre)*dt*dt)*(0.5*dt);
        V.block<3,3>(3,3)=Matrix3d::Identity()*0.5*dt;
        V.block<3,3>(3,9)=Matrix3d::Identity()*0.5*dt;
        V.block<3,3>(6,0)=0.5*dq.toRotationMatrix()*dt;
        V.block<3,3>(6,3)=0.5*(-1*dq_1.toRotationMatrix()*skew(acc_1-ba_pre)*dt)*(0.5*dt);
        V.block<3,3>(6,6)=0.5*dq_1.toRotationMatrix()*dt;
        V.block<3,3>(6,9)=0.5*(-1*dq_1.toRotationMatrix()*skew(acc_1-ba_pre)*dt)*(0.5*dt);
        V.block<3,3>(9,12)=Matrix3d::Identity()*dt;
        V.block<3,3>(12,15)=Matrix3d::Identity()*dt;
        jacobian=F*jacobian;
        covariance=F*covariance*F.transpose()+V*noise*V.transpose();
    }
    //更新dq dv dp
    dq=dq_1;
    dp=dp_1;
    dv=dv_1;
    dq.normalize();
    cout<<"p:"<<endl<<dp<<endl;
    cout<<"v:"<<endl<<dv<<endl;
    cout<<"q:"<<endl<<dq.toRotationMatrix()<<endl;
}

void PreIntegration::clearState()
{

    dt_buf.clear();
    acc_buf.clear();
    gyr_buf.clear();
}