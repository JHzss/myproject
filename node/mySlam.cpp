//
// Created by jh on 18-3-12.
//
#include "myheader.h"
#include "Camera.h"
#include "parameters.h"
#include "Read_dataset.h"
#include "System.h"
#include "Publishers.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

condition_variable con;
mutex load_lock;
mutex image_lock;

int imu=0,image=0;
queue<sensor_msgs::ImuPtr> imu_buf;
queue<sensor_msgs::ImagePtr> image_buf;
sensor_msgs::ImagePtr first_image;
ros::Time image_first_time,image_second_time;
vector<pair<vector<sensor_msgs::ImuPtr>,sensor_msgs::ImagePtr>> measurements;
//数值分析P150
double Newton(uint32_t stamp,uint32_t stamp0, double a0,uint32_t stamp1, double a1,uint32_t stamp2, double a2,uint32_t stamp3, double a3)
{
    double f01,f12,f23,f012,f123,f0123;
    double N1,N2,N3;
    f01=(a1-a0)/(stamp1-stamp0);
    f12=(a2-a1)/(stamp2-stamp1);
    f23=(a3-a2)/(stamp3-stamp2);
    f012=(f12-f01)/(stamp2-stamp0);
    f123=(f23-f12)/(stamp3-stamp1);
    f0123=(f123-f012)/(stamp3-stamp0);
    N1=a0+(stamp-stamp0)*f01;
    N2=N1+(stamp-stamp0)*(stamp-stamp1)*f012;
    N3=N2+(stamp-stamp0)*(stamp-stamp1)*(stamp-stamp2)*f0123;
    return N3;
}
sensor_msgs::ImuPtr NewtonImu(sensor_msgs::ImuPtr &imu0,sensor_msgs::ImuPtr &imu1,sensor_msgs::ImuPtr &imu2,sensor_msgs::ImuPtr &imu3,ros::Time image_time)
{
    cout<<"input:"<<endl;
    cout<<"imu0 "<<"stamp: "<<imu0->header.stamp<<" "<<"acc:"<<endl<<imu0->linear_acceleration<<endl<<"gyr:"<<endl<<imu0->angular_velocity<<endl;
    cout<<"imu1 "<<"stamp: "<<imu1->header.stamp<<" "<<"acc:"<<endl<<imu1->linear_acceleration<<endl<<"gyr:"<<endl<<imu1->angular_velocity<<endl;
    cout<<"imu2 "<<"stamp: "<<imu2->header.stamp<<" "<<"acc:"<<endl<<imu2->linear_acceleration<<endl<<"gyr:"<<endl<<imu2->angular_velocity<<endl;
    cout<<"imu3 "<<"stamp: "<<imu3->header.stamp<<" "<<"acc:"<<endl<<imu3->linear_acceleration<<endl<<"gyr:"<<endl<<imu3->angular_velocity<<endl;
    sensor_msgs::ImuPtr imu= imu0;
    uint32_t stamp;
    double ax,ay,az;
    double gx,gy,gz;
    stamp=image_time.nsec;

    gx=Newton(stamp,imu0->header.stamp.nsec,imu0->angular_velocity.x,imu1->header.stamp.nsec,imu1->angular_velocity.x,imu2->header.stamp.nsec,imu2->angular_velocity.x,imu3->header.stamp.nsec,imu3->angular_velocity.x);
    gy=Newton(stamp,imu0->header.stamp.nsec,imu0->angular_velocity.y,imu1->header.stamp.nsec,imu1->angular_velocity.y,imu2->header.stamp.nsec,imu2->angular_velocity.y,imu3->header.stamp.nsec,imu3->angular_velocity.y);
    gz=Newton(stamp,imu0->header.stamp.nsec,imu0->angular_velocity.z,imu1->header.stamp.nsec,imu1->angular_velocity.z,imu2->header.stamp.nsec,imu2->angular_velocity.z,imu3->header.stamp.nsec,imu3->angular_velocity.z);

    ax=Newton(stamp,imu0->header.stamp.nsec,imu0->linear_acceleration.x,imu1->header.stamp.nsec,imu1->linear_acceleration.x,imu2->header.stamp.nsec,imu2->linear_acceleration.x,imu3->header.stamp.nsec,imu3->linear_acceleration.x);
    ay=Newton(stamp,imu0->header.stamp.nsec,imu0->linear_acceleration.y,imu1->header.stamp.nsec,imu1->linear_acceleration.y,imu2->header.stamp.nsec,imu2->linear_acceleration.y,imu3->header.stamp.nsec,imu3->linear_acceleration.y);
    az=Newton(stamp,imu0->header.stamp.nsec,imu0->linear_acceleration.z,imu1->header.stamp.nsec,imu1->linear_acceleration.z,imu2->header.stamp.nsec,imu2->linear_acceleration.z,imu3->header.stamp.nsec,imu3->linear_acceleration.z);

    imu->header.stamp=image_time;
    imu->angular_velocity.x=gx;
    imu->angular_velocity.y=gy;
    imu->angular_velocity.z=gz;
    imu->linear_acceleration.x=ax;
    imu->linear_acceleration.y=ay;
    imu->linear_acceleration.z=az;
    cout<<"imu "<<"stamp: "<<imu->header.stamp<<" "<<"acc:"<<endl<<imu->linear_acceleration<<endl<<"gyr:"<<endl<<imu->angular_velocity<<endl;
    return imu;
}

pair<vector<sensor_msgs::ImuPtr>,sensor_msgs::ImagePtr> getMeasurement()
{
    cout<<"in"<<endl;
    cout<<image_buf.size()<<"     "<<imu_buf.size()<<endl;
    pair<vector<sensor_msgs::ImuPtr>,sensor_msgs::ImagePtr> measurement;
    vector<sensor_msgs::ImuPtr> imus;
    sensor_msgs::ImagePtr img;
    sensor_msgs::ImuPtr imu0,imu1,imu2,imu3,imu4,imu5,imu6,imu7;
    sensor_msgs::ImuPtr imu_begin,imu_end;

    Vector3d acc_begin,acc_end;
    Vector3d gyr_begin,gyr_end;

    //image信息或IMU信息太少
    if((image_buf.size()<4)||imu_buf.empty())
    {
        ROS_INFO("Too little data");
        return measurement;
    }
    //刚开始的时候IMU信息比Image信息来的晚
    if(first_image==nullptr)
    {
        if(imu_buf.front()->header.stamp>image_buf.front()->header.stamp)
        {
            ROS_INFO("IMU is later than image,wait for imu and delete image");
            image_buf.pop();
            return measurement;
        }
    }
    //保证IMU信息早于image信息之后就设置img信息
    if(imu_buf.front()->header.stamp<=image_buf.front()->header.stamp)
    {
        if(first_image==nullptr)
        {
            img=image_buf.front();
            image_first_time=img->header.stamp;
            first_image=image_buf.front();
            image_buf.pop();
        }
        image_second_time=image_buf.front()->header.stamp;
        //两帧之间的IMU信息不全
        if(imu_buf.back()->header.stamp<image_second_time)
        {
            ROS_INFO("IMU is not enough,wait for imu and delete image");
            return measurement;
        }
        image++;//判断这是第几帧（系统的首帧作为第0帧）
    }
    //只保留第一帧图像最近两个的imu进行牛顿差值
    if(image==1)
    {

        while (imu_buf.front()->header.stamp<image_first_time)
        {
            if(imu1!= nullptr)
            {
                cout<<"imu1 is live,set imu0"<<endl;
                imu0=imu1;//todo 需要判定图像帧之前是否存在两帧以上的imu信息
            }

            imu1=imu_buf.front();
            if(imu0== nullptr)
            {
                cout<<"set imu0"<<endl;
                imu0=imu1;//todo 需要判定图像帧之前是否存在两帧以上的imu信息
            }
            cout<<"test "<<imu1->header.stamp<<endl;
            imu_buf.pop();
        }
    }
    ///添加 imus
    if(image!=1)
    {
        imu_begin=imu_end;
        imus.emplace_back(imu_begin);
        imus.emplace_back(imu6);
    }
    else
    {
        imu_begin=imu1;
        imus.emplace_back(imu_begin);
    }
    while(imu_buf.front()->header.stamp<=image_second_time)
    {
        //只有在处理第一帧的时候才需要0 1 2 3
        if(image==1)
        {
            if(imus.size()==1)
                imu3=imu_buf.front();
            if(imus.size()==2)
            {
                imu2=imu3;
                imu3=imu_buf.front();
            }
        }
        imus.emplace_back(imu_buf.front());
        if(imu5!= nullptr)
              imu4=imu5;
        imu5=imu_buf.front();
        imu_buf.pop();
    }
    imu_end=imu5;
    imus.emplace_back(imu_end);
    //添加imus结束
    imu6=imu_buf.front();
    imu_buf.pop();
    imu7=imu_buf.front();
    if(image==1)
    {
        imu_begin=NewtonImu(imu0,imu1,imu2,imu3,image_first_time);
        imu_end=NewtonImu(imu4,imu5,imu6,imu7,image_second_time);
    }
    else
        imu_end=NewtonImu(imu4,imu5,imu6,imu7,image_second_time);
    measurement=(make_pair(imus,image_buf.front()));
    image_buf.pop();
    /*至此，得到的信息应该是这样的：（其中，imu_buf在imu6之前的（包括6）都被pop了，imu_buf的front指针指向7）
     *           0123                4567
     * ,IMU序列  ||||||||||||||||||||||||
     *           -----------------------
     *   图像序列  |                   |
     *     imu_begin,2,3,……,4,5,imu_end;
     */
}
System::Ptr System=System::creat();
/**
 * @brief 处理IMU数据，包括：
 * （1）插值法将IMU首尾数据进行对齐
 * （2）预积分（中值积分），更新J矩阵和P矩阵
 * （3）更新变量
 */
void Imu_process()
{

}

/**
 * @brief 处理图像数据
 * （1）光流跟踪提取特征
 * （2）计算位姿
 * （3）三角化恢复特征点
 */
void Image_process()
{

}
/**
 * system->track();
 * @brief 跟踪
 * （1）相机初始化
 * （2）相机-IMU对齐
 * （3）优化：III-B
 * （4）重力求精
 */

/**
 * @brief 非线性优化
 */
void Optimize()
{

}

/**
 * @brief 处理线程
 */
void process()
{
    while(true)
    {
        cout<<"here"<<endl;

        pair<vector<sensor_msgs::ImuPtr>,sensor_msgs::ImagePtr> measurement;

        unique_lock<mutex> lk(load_lock);
        con.wait(lk,[&]
        {
           return (measurement=getMeasurement()).first.size()!=0;// 等价于 while(!measurement=getMeasurement()).first.size()!=0) { cv.wait(lk); }
        });

        measurements.push_back(measurement);
        cout<<"measurement size:"<<measurements.size()<<endl;
//        cout<<measurement.first.size()<<endl;
//        cout<<"I am out"<<endl;
        lk.unlock();

//        Imu_process();
//        Image_process();
//        System->track();
//        Optimize();
    }
}
void imu_callback(const sensor_msgs::ImuPtr &imu_msg)
{
    load_lock.lock();
//    cout<<"imu:"<<imu++<<"  "<<imu_msg->header.stamp<<endl;
    imu_buf.push(imu_msg);
    load_lock.unlock();

}
void image_callback(const sensor_msgs::ImagePtr &img_msg)
{
    load_lock.lock();
    image_buf.push(img_msg);
//    cout<<"image:"<<image_buf.size()<<endl;
    load_lock.unlock();
    con.notify_one();
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"myproject");
    ros::NodeHandle n("~");
    //todo 编写launch文件，向里面添加参数，包括config文件

    LoadParameters(n);

    setPublishers(n);

    ros::Subscriber sub_imu= n.subscribe(IMU_TOPIC,2000,imu_callback);           //imu_buf
    ros::Subscriber sub_image = n.subscribe(IMAGE_TOPIC, 2000, image_callback);  //image_buf
//  cout<<system->camera_s->getK()<<endl;

    thread main_thread(process);
    thread loop_detection,pose_graph;


    ros::spin();
    return 0;
}
