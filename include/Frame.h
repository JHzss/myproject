//
// Created by jh on 18-3-9.
//

#ifndef MYPROJECT_FRAME_H
#define MYPROJECT_FRAME_H

#include "myheader.h"
#include "Camera.h"

class Frame {
public:
    typedef shared_ptr<Frame> Ptr;
    int id_,next_id_;
    const double timestamps_;
    vector<Point> features_;
    vector<Point2d> points_cam_;
    vector<Point3d> points_world_;
    SE3 T_c2w;
    SE3 T_w2c;
    const Camera::Ptr camara_;
    const Mat img_;
    Eigen::Matrix3d R_;
    Vector3d t_;
public:
    Frame(const Mat& img, const double& timeStamps, const int& id, const Camera::Ptr &cam);

    void setPose(const SE3& pose);//T_w2c,T_c2w,R,t
    int getFeatureNumber();
    vector<Point2d> getPoints_cam(const Camera::Ptr &cam);//利用提取出的特征点计算其在相机坐标系下的坐标
    vector<Point3d> getPoints_world(const Camera::Ptr &cam);//利用提取出的特征点计算其在世界坐标系下的坐标

};


#endif //MYPROJECT_FRAME_H
