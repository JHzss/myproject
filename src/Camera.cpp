//
// Created by jh on 18-3-5.
//

#include "../include/Camera.h"
//const 类型数据必须初始化
Camera::Camera(const Parameters::Ptr& param):fx_(param->camera_fx),fy_(param->camera_fy),cx_(param->camera_cx),cy_(param->camera_cy)
{
    this->K_=calcK();
}



Point2d Camera::uv2camera(Point point_uv_, Mat K_)
{
    Point2d point_cam;
    point_cam.x=(double)point_uv_.x-K_.at<double>(0,2)/K_.at<double>(0,0);
    point_cam.y=(double)point_uv_.y-K_.at<double>(1,2)/K_.at<double>(1,1);
    return point_cam;
}
Mat Camera::calcK()
{
    Mat K=(Mat_<double>(3,3)<< fx_,0,cx_,0,fy_,cy_,0,0,1.0);
    return K;
}