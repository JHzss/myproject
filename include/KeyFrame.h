//
// Created by jh on 18-3-9.
//

#ifndef MYPROJECT_KEYFRAME_H
#define MYPROJECT_KEYFRAME_H

#include "myheader.h"
#include "Frame.h"


class KeyFrame:public Frame
{
public:
    typedef shared_ptr<KeyFrame> Ptr;
    const uint64_t id_;
    static uint64_t next_id_;//关键帧ID
    const uint64_t frame_id_;//在普通帧中的id
public:
    KeyFrame(const Frame& frame);


};


#endif //MYPROJECT_KEYFRAME_H
