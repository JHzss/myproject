//
// Created by jh on 18-3-5.
//

#ifndef MYPROJECT_MYHEADER_H
#define MYPROJECT_MYHEADER_H

#include <iostream>
#include <fstream>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

using Eigen::Vector2d;
using Eigen::Vector3d;::
using Eigen::Matrix3d;
using Eigen::Quaterniond;
//有的sophus版本要改成.h
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
using Sophus::SE3;
using Sophus::SO3;
using Sophus::Quaterniond;

#include <vector>
#include <list>
#include <math.h>
#include <string>
#include <memory>
#include <thread>
#include <map>
#include <iostream>
#include <mutex>
#include <condition_variable>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
using namespace std;
using namespace cv;
#endif //MYPROJECT_MYHEADER_H

