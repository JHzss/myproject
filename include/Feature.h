//
// Created by jh on 18-3-12.
//

#ifndef MYPROJECT_FEATURE_H
#define MYPROJECT_FEATURE_H

#include "myheader.h"
#include "Camera.h"

class Feature{

public:
    typedef shared_ptr<Feature> Ptr;
    enum quality_{
        good=1,
        normal=0,
        bad=-1
    } quality_feature;
//    Mat descriptor_;
    uint64_t id_;
    static uint64_t next_id_;
    vector<pair<uint64_t,Point> > point_pre_frame;
    vector<pair<uint64_t,Point2f> > point_pre_camera;

    Point3d pose_world_;
    int track_times_;
    int refKeyFrameID_;//参考帧ID-投影误差最小的关键帧
    int first_seen_; //首先观测到的Frame ID
    int last_seen_;  //最后观测到的Frame ID
    bool if3D;
    Mat camera_k;

    Feature(Point2f &p);
    bool TrackBy(uint64_t& frame_id);
    Point2f pointIn(uint64_t& frame_id);
    Point2f pointIncamera(uint64_t& frame_id);
    void updatePose();                                                      //更新特征点位姿
    int getTrackNumber(const Feature::Ptr& feature);                        //获取跟踪次数
    void addTrack(uint64_t frame_id, Point2f& point);                                       //增加特征点的观测
    void eraseTrack(uint64_t frame_id, Point& point);                                     //删除特征点的观测

    inline static Feature::Ptr creat(Point2f &p) {return Feature::Ptr(new Feature(p));}

    static void addTrack(vector<Feature::Ptr>& features, vector<uint64_t > &pre_points_ids,uint64_t frame_id, vector<Point2f>& point,Mat &k);
    static void fuseSameFeature(vector<Feature::Ptr>& features,uint64_t l);
    void destoryFeature();


};
#endif //MYPROJECT_FEATURE_H
