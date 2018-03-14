//
// Created by jh on 18-3-12.
//

#ifndef MYPROJECT_SYSTEM_H
#define MYPROJECT_SYSTEM_H

#include "myheader.h"
#include "Camera.h"
#include "Feature.h"
#include "Frame.h"
#include "Feature_tracking.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "Optimizer.h"
#include "parameters.h"
#include "Read_dataset.h"


class System
{
public:
    enum Status {
        Init_good,
        Init_bad,
        Init_fail,
        Track_good,
        Track_bad,
        Track_fail
    } SystemStatus_;

    void Imu_process();
    void Image_process();
    void Imu_Visual_align();

    void slideWindow();
    void optimization();


    void track();


    Camera::Ptr camera_s;                    //相机指针
    Vector3d init_t;                         //初始化得到的相对位移
    Eigen::Matrix3d init_R;                  //初始化得到的相对旋转
    vector<Feature::Ptr> allFeatures_s;      //所有的特征点
    vector<MapPoint::Ptr> allMapPoints_s;    //所有的特征点
    vector<Frame::Ptr> all_Frames_s;         //所有的帧
    vector<KeyFrame::Ptr> all_Keyframes_s;   //所有的关键帧
    Feature_tracking::Ptr tracker_s;         //特征跟踪
    Frame::Ptr currFrame_s;                  //当前帧
    KeyFrame::Ptr refKeyFrame_s;             //当前参考帧
    Read_dataset::Ptr dataset_s;             //数据集读取

    list<double> timestamps_s;
    list<Vector3d> pose_frames_s;
    list<Vector3d> pose_keyframes_s;





};


#endif //MYPROJECT_SYSTEM_H
