//
// Created by jh on 18-3-9.
//

#ifndef MYPROJECT_KEYFRAME_H
#define MYPROJECT_KEYFRAME_H

#include "myheader.h"
#include "Frame.h"

class Frame;

class KeyFrame:public Frame
{
public:
    typedef shared_ptr<KeyFrame> Ptr;
    const uint64_t id_;
    static uint64_t next_id_;//关键帧ID
    const uint64_t frame_id_;//在普通帧中的id
    inline static KeyFrame::Ptr creat(std::shared_ptr<Frame>& frame){ return KeyFrame::Ptr(new KeyFrame(frame));}

    KeyFrame(std::shared_ptr<Frame>& frame);


};


#endif //MYPROJECT_KEYFRAME_H
