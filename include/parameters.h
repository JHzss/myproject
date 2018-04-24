//
// Created by jh on 18-3-8.
//
#pragma once

#ifndef MYPROJECT_PARAMETERS_H
#define MYPROJECT_PARAMETERS_H

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include "myheader.h"

const int WINDOW_SIZE = 10;
extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
void LoadParameters(ros::NodeHandle &n);

extern double camera_fx;
extern double camera_fy;
extern double camera_cx;
extern double camera_cy;

extern double camera_k1;
extern double camera_k2;
extern double camera_p1;
extern double camera_p2;

extern cv::Mat camera_k;
extern int number_of_features;

extern double acc_n,acc_w;
extern double gyr_n,gyr_w;

extern int image_width;
extern int image_height;
extern int slideWindowsize;
extern float init_dist;

#endif //MYPROJECT_PARAMETERS_H
