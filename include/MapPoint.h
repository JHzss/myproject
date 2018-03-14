//
// Created by jh on 18-3-6.
//

#ifndef MYPROJECT_MAPPOINT_H
#define MYPROJECT_MAPPOINT_H

#include "myheader.h"
#include "Frame.h"
#include "KeyFrame.h"

class MapPoint
{
public:


public:
    typedef shared_ptr<MapPoint> Ptr;
    const uint64_t id_;
    Vector3d pose_;
    Vector3d pose_norm_;
    Mat descriptor_;
    int seenTimes_;

    KeyFrame::Ptr refKeyFrame_;//todo 定义keyframe类

    Frame::Ptr firse_seen_;
    Frame::Ptr last_seen_;


};


#endif //MYPROJECT_MAPPOINT_H
