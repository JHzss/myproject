//
// Created by jh on 18-3-12.
//

#include "System.h"
#include "imu_process.h"

fstream pre_imu_txt("/home/jh/catkin_ws/src/myproject/pre_imu.txt", ios::out);

System::System():SystemStatus_(Init),currFrame_s(nullptr),refKeyFrame_s(nullptr),init_flag_s(false),load_init_flag_s(false)
{
    frame_count=0;
    tracker_s=Feature_tracking::creat(image_width,image_height);         //特征跟踪
}
/**
 * @brief 处理IMU数据，包括：
 * （1）插值法将IMU首尾数据进行对齐
 * （2）预积分（中值积分），更新J矩阵和P矩阵
 * （3）更新变量
 */

void System::Imu_process(vector<sensor_msgs::ImuPtr> imus)
{
    int imu_number=0;
    double last_time=-1;
    sensor_msgs::ImuPtr first_imu;
    Vector3d acc_1,gyr_1;//前后两次IMU的测量值
    Vector3d acc_0(0.0,0.0,0.0);
    Vector3d gyr_0(0.0,0.0,0.0);

    for(auto imu:imus)
    {
        //todo 需要检查一下这里定义成double结果对不对
        double t=imu->header.stamp.toSec();
        if(last_time<0)
            last_time=t;
        double dt=t-last_time;
        last_time=t;

        double ba_temp[]{0.0,0.0,0.0};
        double bg_temp[]{0.0,0.0,0.0};
        double ax=imu->linear_acceleration.x-ba_temp[0];//todo 这里重传播的时候还需要检查一遍
        double ay=imu->linear_acceleration.y-ba_temp[1];
        double az=imu->linear_acceleration.z-ba_temp[2];
        double gx=imu->angular_velocity.x-bg_temp[0];
        double gy=imu->angular_velocity.y-bg_temp[1];
        double gz=imu->angular_velocity.z-bg_temp[2];
        //这里逻辑应该是正确的

        //frame_count 应该是本次的
        //todo 这个判断方法原理是什么
        cout<<"if exist?"<<endl;
        if(imu_number==0)
        {
            cout<<"init preintegration"<<endl;
            PreIntegration::Ptr pre_integration_tmp=PreIntegration::creat(acc_0,acc_1,gyr_0,gyr_1,ba[frame_count],bg[frame_count],dt);
            preIntegrations.emplace_back(pre_integration_tmp);
//            preIntegrations[frame_count]=PreIntegration::creat(acc_0,acc_1,gyr_0,gyr_1,ba[frame_count],bg[frame_count],dt);
            preIntegrations[frame_count]->acc_1=Vector3d(ax,ay,az);//本次IMU数据组成
            preIntegrations[frame_count]->gyr_1=Vector3d(gx,gy,gz);
        }
        if(imu_number!=0)
        {
            preIntegrations[frame_count]->dt=dt;
            preIntegrations[frame_count]->acc_0=preIntegrations[frame_count]->acc_1;
            preIntegrations[frame_count]->gyr_0=preIntegrations[frame_count]->gyr_1;
            preIntegrations[frame_count]->acc_1=Vector3d(ax,ay,az);
            preIntegrations[frame_count]->gyr_1=Vector3d(gx,gy,gz);
            //todo 开始对预积分进行操作
            cout<<"run preintegration"<<endl;
            preIntegrations[frame_count]->run();
            //todo 将IMU和图像对应上，需要定义frame_count,预积分
        }
        imu_number++;//表示这两张图像之间的IMU计数，begin对应imu-0
    }
    preIntegrations[frame_count]->img_stamp=imus.back()->header.stamp.toSec();
    pre_imu_txt<<fixed;
    pre_imu_txt<<preIntegrations[frame_count]->img_stamp<<endl;
    pre_imu_txt<<"dp: "<<preIntegrations[frame_count]->dp(0)<<" "<<preIntegrations[frame_count]->dp(1)<<" "<<preIntegrations[frame_count]->dp(2)<<endl;
    pre_imu_txt<<"dv: "<<preIntegrations[frame_count]->dv(0)<<" "<<preIntegrations[frame_count]->dv(1)<<" "<<preIntegrations[frame_count]->dv(2)<<endl;
    pre_imu_txt<<"dq: "<<preIntegrations[frame_count]->dq.w()<<" "<<preIntegrations[frame_count]->dq.x()<<" "<<preIntegrations[frame_count]->dq.y()<<" "<<preIntegrations[frame_count]->dq.z()<<endl;
}

void System::track(pair<vector<sensor_msgs::ImuPtr>,sensor_msgs::ImagePtr> measurement)
{
    cout<<"--------frame count--------------------------------------------------"<<frame_count<<endl;
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

