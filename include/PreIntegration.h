//
// Created by jh on 18-4-18.
//

#ifndef MYPROJECT_PREINTEGRATION_H
#define MYPROJECT_PREINTEGRATION_H
#include "myheader.h"
#include "parameters.h"

class PreIntegration
{
public:
    typedef shared_ptr<PreIntegration > Ptr;
    PreIntegration(Vector3d acc_0_,Vector3d acc_1_,Vector3d gyr_0_,Vector3d gyr_1_,Vector3d ba_,Vector3d bg_,double dt_);
    inline static PreIntegration::Ptr creat(Vector3d acc_0,Vector3d acc_1,Vector3d gyr_0,Vector3d gyr_1,Vector3d ba,Vector3d bg,double dt)
        {return PreIntegration::Ptr(new PreIntegration(acc_0,acc_1,gyr_0,gyr_1,ba,bg,dt));};

    void run();
    void rerun();
    void clearState();

    Eigen::Matrix<double, 15,15 > jacobian,covariance;
    Eigen::Vector3d ba_pre, bg_pre;
    Eigen::Matrix<double,18,18> noise;

    double dt,sum_t;
    vector<double > dt_buf;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_first, gyr_first;
    Eigen::Vector3d acc_1, gyr_1;
    vector<Vector3d > acc_buf;
    vector<Vector3d > gyr_buf;

    Vector3d dp;
    Vector3d dv;
    Eigen::Quaterniond dq;
    double img_stamp;//这一组imu得到的预积分对应的图像的stamp

};
#endif //MYPROJECT_PREINTEGRATION_H
