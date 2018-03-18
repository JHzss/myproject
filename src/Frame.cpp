//
// Created by jh on 18-3-9.
//
#include "myheader.h"
#include "Frame.h"

uint64_t Frame::next_id_=0;
Frame::Frame(const Mat &img, const double& timeStamps, const Camera::Ptr &cam) :camara_(cam),timestamps_(timeStamps),img_(img),id_(next_id_++)
{

}


void Frame::setPose(const SE3 &pose)
{
    this->T_w2c=pose;
    this->T_c2w=pose.inverse();
    this->R_=pose.rotationMatrix();
    this->t_=pose.translation();
}

int Frame::getFeatureNumber()
{
    return this->featuresInThisFrame_.size();
}
//利用提取出的特征点计算其在相机坐标系下的坐标
vector<Point2d> Frame::getPoints_cam(const Camera::Ptr &cam)
{
//    vector<Point2d> point_cam_pose;
//    for(auto f:this->featuresInThisFrame_)
//    {
//        point_cam_pose.push_back(Camera::uv2camera( ,cam->getK()));//todo
//    }
//    return point_cam_pose;
}

vector<Point3d> Frame::getPoints_world(const Camera::Ptr &cam)
{
//    vector<Point3d> point_world_pose;
//    for(auto f:this->featuresInThisFrame_)
//    {
//        point_world_pose.push_back(Camera::uv2world(f));//todo
//    }
//    return point_world_pose;
}
