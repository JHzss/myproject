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
#include "Initializer.h"



class System
{
public:

    typedef shared_ptr<System> Ptr;

    System();
    inline static System::Ptr creat(){return System::Ptr(new System());}
    enum Status {
        begin,
        Init,
        Track_good,
        Track_bad,
        Track_fail
    } SystemStatus_;

    void track();
    void Imu_Visual_align();

    void slideWindow();                      //边缘化，控制变量

    bool checkInit();                        //检测是否可以初始化

    Initializer::Ptr initializer_s;          //初始化指针

    bool init_flag_s;                        //是否可以初始化的标志
    bool result_flag_s;                      //初始化是否成功
    bool load_init_flag_s;                     //初始化是否加载了足够的图片

    Vector3d init_t;                         //初始化得到的相对位移
    Eigen::Matrix3d init_R;                  //初始化得到的相对旋转

    vector<Feature::Ptr> allFeatures_s;      //所有的特征点
    vector<MapPoint::Ptr> allMapPoints_s;    //所有的特征点
    vector<Frame::Ptr> all_Frames_s;         //所有的帧
    vector<KeyFrame::Ptr> all_Keyframes_s;   //所有的关键帧
    vector<Mat> all_images_s;//所有的图片


    Feature_tracking::Ptr tracker_s;         //特征跟踪

    Frame::Ptr currFrame_s;                  //当前帧
    Mat currImg_s;
    KeyFrame::Ptr refKeyFrame_s;             //当前参考帧

//    Read_dataset::Ptr dataset_s;             //数据集读取

    list<double> timestamps_s;
    list<Vector3d> pose_frames_s;
    list<Vector3d> pose_keyframes_s;

    //todo 定义 slideWindow中的变量
};


#endif //MYPROJECT_SYSTEM_H
