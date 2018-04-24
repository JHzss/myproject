//
// Created by jh on 18-3-12.
//

#include "System.h"
#include "imu_process.h"

System::System():SystemStatus_(Init),currFrame_s(nullptr),refKeyFrame_s(nullptr),init_flag_s(false),load_init_flag_s(false)
{
    tracker_s=Feature_tracking::creat(image_width,image_height);         //特征跟踪
}
/**
 * @brief 处理IMU数据，包括：
 * （1）插值法将IMU首尾数据进行对齐
 * （2）预积分（中值积分），更新J矩阵和P矩阵
 * （3）更新变量
 */
void calcImu(sensor_msgs::ImuPtr imu)
{

}
void System::Imu_process(vector<sensor_msgs::ImuPtr> imus)
{
    int imu_number=0;
    double first_t;
    double last_time=-1;
    sensor_msgs::ImuPtr first_imu;
    Vector3d acc_0,acc_1,gyr_0,gyr_1;//前后两次IMU的测量值
    for(auto imu:imus)
    {
//        calcImu(imu);
        //todo 需要检查一下这里定义成double结果对不对
        double t=imu->header.stamp.toSec();
        if(last_time<0)
            last_time=t;
        double dt=t-last_time;
        last_time=t;

        double ba_temp[]{0.0,0.0,0.0};
        double bg_temp[]{0.0,0.0,0.0};
        double ax=imu->linear_acceleration.x-ba_temp[0];
        double ay=imu->linear_acceleration.y-ba_temp[1];
        double az=imu->linear_acceleration.z-ba_temp[2];
        double gx=imu->angular_velocity.x-bg_temp[0];
        double gy=imu->angular_velocity.y-bg_temp[1];
        double gz=imu->angular_velocity.z-bg_temp[2];
        //这里逻辑应该是正确的
        acc_0=acc_1;//上次的IMU数据
        gyr_0=gyr_1;
        acc_1=Vector3d(ax,ay,az);//本次IMU数据组成
        gyr_1=Vector3d(gx,gy,gz);
        //frame_count 应该是本次的
        //todo 这个判断方法原理是什么
        if(!preIntegrations[frame_count])
            preIntegrations[frame_count]=PreIntegration::creat(acc_0,acc_1,gyr_0,gyr_1,ba[frame_count],bg[frame_count],dt);
        if(dt!=0)
        {
            //todo 开始对预积分进行操作
            preIntegrations[frame_count]->run();
            //todo 将IMU和图像对应上，需要定义frame_count,预积分
        }
        imu_number++;//表示这两张图像之间的IMU计数，begin对应imu-0
    }
}

void System::track(pair<vector<sensor_msgs::ImuPtr>,sensor_msgs::ImagePtr> measurement)
{
    Imu_process(measurement.first);
    frame_count++;
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

