//
// Created by jh on 18-3-5.
//

#include "Camera.h"
//const 类型数据必须初始化
Camera::Camera():fx_(camera_fx),fy_(camera_fy),cx_(camera_cx),cy_(camera_cy)
{
    this->K_=calcK();
}

Point2f Camera::camera2uv(Point2d point_camera_, Mat K_)
{
    Point2f point;
    point.x=point_camera_.x*K_.at<double>(0,0)+K_.at<double>(0,2);
    point.y=point_camera_.y*K_.at<double>(1,1)+K_.at<double>(1,2);
    return point;
}

Point2d Camera::uv2camera(Point2f &point_uv_, Mat K_)//归一化平面上
{
    Point2d point_cam;
    point_cam.x=((double)point_uv_.x-K_.at<double>(0,2))/K_.at<double>(0,0);
    point_cam.y=((double)point_uv_.y-K_.at<double>(1,2))/K_.at<double>(1,1);
    return point_cam;
}

Point2f Camera::removeDistort(Point2f &pre,double k1,double k2,double k3,double p1,double p2, Mat &k_)
{
    Point2d po,later;
    po=uv2camera(pre,k_);

    double x2, y2, xy, r2, rad_dist;
    x2 = po.x * po.x;
    y2 = po.y * po.y;
    xy = po.x * po.y;
    r2 = x2 + y2;
    rad_dist = 1 + k1* r2 + k2 * r2 * r2 + k3* r2 * r2 * r2;
    po.x=(float)(po.x * rad_dist +2 * p1 * xy + p2 * (r2+ 2 * x2));
    po.y=(float)(po.y * rad_dist +2 * p2 * xy + p1 * (r2+ 2 * x2));

    later=camera2uv(po,k_);
    return (Point2f)later;
}

Mat Camera::calcK()
{
    Mat K=(Mat_<double>(3,3)<< fx_,0,cx_,0,fy_,cy_,0,0,1.0);
    return K;
}