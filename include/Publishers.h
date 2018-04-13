//
// Created by jh on 18-4-12.
//
#pragma once

#ifndef MYPROJECT_PUBLISHERS_H
#define MYPROJECT_PUBLISHERS_H

#include "myheader.h"
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "parameters.h"
#include <fstream>

extern ros::Publisher pub_image;

void setPublishers(ros::NodeHandle &n);
void PubImage(sensor_msgs::ImagePtr &raw_image);
#endif //MYPROJECT_PUBLISHERS_H
