//
// Created by jh on 18-3-12.
//

#include "System.h"

System::System():SystemStatus_(Init),currFrame_s(nullptr),refKeyFrame_s(nullptr),init_flag_s(false),load_init_flag_s(false)
{
    tracker_s=Feature_tracking::creat(image_width,image_height);         //特征跟踪
}

void System::track()
{
//    cout<<"tracker_s->init_frame_count:"<<tracker_s->init_frame_count<<endl;
//    pair<vector<sensor_msgs::ImuConstPtr>,sensor_msgs::ImageConstPtr> measurement;
//
//
//    currImg_s=image.clone();
//    currFrame_s=Frame::creat(currImg_s,timestamp);
//    all_Frames_s.push_back(currFrame_s);
//    cout<<"frameid"<<currFrame_s->id_<<endl;
//
//    if(SystemStatus_==Init)
//    {
//        if(!load_init_flag_s)//初始化需要的图片还不够
//        {
//            load_init_flag_s= tracker_s->loadInitImage(currImg_s,currFrame_s);//加载初始化用到的图片,并提取特征点和匹配点,找出具有相对位移的帧
//            all_Keyframes_s.push_back(refKeyFrame_s);
//        }
//        if(load_init_flag_s)//加载了足够的图片用于初始化
//        {
//            init_flag_s=tracker_s->initialization();  //todo 10帧的初始化操作
//            if(init_flag_s)
//            {
//                tracker_s->initReset();    //todo 初始化完成后就清空状态量，以备下次跟丢之后初始化时使用
//                SystemStatus_=Track_good;
//            }
//        }
//    }
//    else if(SystemStatus_==Track_good)      //跟踪效果好（特征点匹配多）
//    {
//        //tracker_s->track_new();
//        slideWindow();
//        //todo 设置状态
//    }
//    else if(SystemStatus_==Track_bad)       //跟踪效果不好（特征匹配少）
//    {
//        Optimizer::windowBA();//todo 跟踪效果不好时加个优化
//        //tracker_s->track_new();
//        slideWindow();
//        //todo 设置状态
//    }
//    else
//    {
//        SystemStatus_=Init;//todo 想想跟踪失败后应该如何做
//    }

}


void System::slideWindow()
{

}
