//
// Created by jh on 18-3-5.
//

#ifndef MYPROJECT_CAMERA_H
#define MYPROJECT_CAMERA_H
#include "myheader.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/opencv.hpp>

#include "parameters.h"
class Parameters;
class Camera {
public:
    typedef shared_ptr<Camera> Ptr;
    Camera(const Parameters::Ptr& param);
//    int id;
    //vector<Pose> camera_poses;
private:
    double fx_,fy_,cx_,cy_;
    double k1_,k2_,p1_,p2_;
    Mat K_;
    Mat K_inv_;
//    Mat R_,t_;
//    Point2d point_camera_;
//    Point   point_uv_;
//    Point3d point_3d_;
public:

    Mat getK(){ return K_;}
    static Point2d uv2camera(Point point_uv_,Mat K_);//todo,还有一些函数没写
    static Point2d world2camera(Point3d point_3d_);
    static Point   camera2uv(Point2d point_camera_,Mat K_);
    static Point   world2uv(Point3d point_3d_);
    static Point3d camera2world(Point2d point_camera_);
    static Point3d uv2world(Point point_uv_);
    Mat calcK();
    inline static Camera::Ptr creat(const Parameters::Ptr& param){ return Camera::Ptr(new Camera(param)); }


};


#endif //MYPROJECT_CAMERA_H
