//
// Created by jh on 18-3-9.
//

#include "KeyFrame.h"
#include "myheader.h"
#include "Frame.h"

uint64_t KeyFrame::next_id_=0;
KeyFrame::KeyFrame(const Frame& frame):Frame(frame.img_,frame.timestamps_,next_id_++,frame.camara_),id_(next_id_++),frame_id_(frame.id_)
{

}