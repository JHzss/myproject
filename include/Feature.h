//
// Created by jh on 18-3-12.
//

#ifndef MYPROJECT_FEATURE_H
#define MYPROJECT_FEATURE_H

#include "myheader.h"

class Feature{

public:
    typedef shared_ptr<Feature> Ptr;
    enum quality_{
        good=1,
        normal=0,
        bad=-1
    } quality_feature;
    Mat descriptor_;
    uint64_t id_,next_id_;
    vector<map<uint64_t,map<uint64_t,Point>>> feature_uv;//map<feature_id,map<frame_id,uv>>
    vector<map<uint64_t,map<uint64_t,Point2d>>> feature_camera_norm;//map<feature_id,map<frame_id,feature_camera_norm>>
    Vector3d pose_world_;
    int track_times_;
    int refKeyFrameID_;//参考帧ID-投影误差最小的关键帧
    int first_seen_; //首先观测到的Frame ID
    int last_seen_;  //最后观测到的Frame ID

    Feature(Vector3d &p);
    void updatePose();                                                      //更新特征点位姿
    int getTrackNumber(const Feature::Ptr& feature);                        //获取跟踪次数
    void addTrack(int& frame_id);                                       //增加特征点的观测
    void eraseTrack(int& frame_id);                                     //删除特征点的观测

    inline static Feature::Ptr creat(Vector3d &p) {return Feature::Ptr(new Feature(p));}
    void fuse_feature(Feature::Ptr& feature_bad,Feature::Ptr& feature_good);//融合特征点


};
#endif //MYPROJECT_FEATURE_H
