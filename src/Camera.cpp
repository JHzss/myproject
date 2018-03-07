//
// Created by jh on 18-3-5.
//

#include "../include/Camera.h"

Point2d Camera::uv2camera(Point point_uv_, Mat K_)
{
    Point2d point_cam;
    point_cam.x=(double)point_uv_.x-K_.at<double>(0,2)/K_.at<double>(0,0);
    point_cam.y=(double)point_uv_.y-K_.at<double>(1,2)/K_.at<double>(1,1);
    return point_cam;
}