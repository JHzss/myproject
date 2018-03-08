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
Mat Camera::getK(const Parameters& param)
{
    Mat K=(Mat_<double>(3,3)<< param.camera_fx,0,param.camera_cx,0,param.camera_fy,param.camera_cy,0,0,1.0);
    return K;
}