//
// Created by jh on 18-4-17.
//
#ifndef MYPROJECT_IMU_PROCESS_H
#define MYPROJECT_IMU_PROCESS_H

#include "myheader.h"

//vector<Vector3d> ba;//加速度计bias
//vector<Vector3d > bg;//陀螺仪bias
vector<Vector3d> P_preIntegration; //预积分得到的P V R
vector<Vector3d> V_preIntegration;
vector<Matrix3d> R_preIntegration;
double first_t;
double last_time=-1;
sensor_msgs::ImuPtr first_imu;
int imu_number;//本次imu的处理数目
Vector3d acc_0,acc_1,gyr_0,gyr_1;//前后两次IMU的测量值



#endif //MYPROJECT_IMU_PROCESS_H
