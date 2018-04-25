//
// Created by jh on 18-3-12.
//
#include "myheader.h"
#include "Camera.h"
#include "parameters.h"
#include "Read_dataset.h"
#include "System.h"
#include "Publishers.h"
#include "imu_process.h"


condition_variable con;
mutex load_lock;
mutex image_lock;

int imu=0,image=0;
queue<sensor_msgs::ImuPtr> imu_buf;
queue<sensor_msgs::ImagePtr> image_buf;
sensor_msgs::ImagePtr first_image;
sensor_msgs::ImuPtr imu_begin,imu_end;
sensor_msgs::ImuPtr imu0,imu1,imu2,imu3,imu4,imu5,imu6,imu7;
ros::Time image_first_time,image_second_time;
vector<pair<vector<sensor_msgs::ImuPtr>,sensor_msgs::ImagePtr>> measurements;
fstream imustxt("/home/jh/catkin_ws/src/myproject/imus.txt", ios::out);



//Hermite插值 数值分析P154
double Hermite(uint64_t stamp_,uint64_t stamp10, double a0,uint64_t stamp11, double a1,uint64_t stamp12, double a2,uint64_t stamp13, double a3)
{
    if(stamp_==stamp11)
        return a1;
    if(stamp_==stamp12)
        return a2;
    double stamp,stamp0,stamp1,stamp2,stamp3;
    double k01,k12,k23,k1,k2;
    double result,result1,result2,result3,result4;

    stamp=((double)stamp_/1000000000.0);
    stamp0=((double)stamp10/1000000000.0);
    stamp1=((double)stamp11/1000000000.0);
    stamp2=((double)stamp12/1000000000.0);
    stamp3=((double)stamp13/1000000000.0);
    k01=(a1-a0)/(stamp1-stamp0);
    k12=(a2-a1)/(stamp2-stamp1);
    k23=(a3-a2)/(stamp3-stamp2);
    k1=(k01+k12)/2;
    k2=(k12+k23)/2;
    result1=(1-2*((stamp-stamp1)/(stamp1-stamp2)))*((stamp-stamp2)/(stamp1-stamp2))*((stamp-stamp2)/(stamp1-stamp2))*a1;
    result2=(1-2*((stamp-stamp2)/(stamp2-stamp1)))*((stamp-stamp1)/(stamp2-stamp1))*((stamp-stamp1)/(stamp2-stamp1))*a2;
    result3=(stamp-stamp1)*((stamp-stamp2)/(stamp1-stamp2))*((stamp-stamp2)/(stamp1-stamp2))*k1;
    result4=(stamp-stamp2)*((stamp-stamp1)/(stamp2-stamp1))*((stamp-stamp1)/(stamp2-stamp1))*k2;
    result=result1+result2+result3+result4;
    cout<<"hermite result:"<<result<<endl;
    return result;
}
/* 牛顿差值
//数值分析P150
double Newton(uint64_t stamp_,uint64_t stamp10, double a0,uint64_t stamp11, double a1,uint64_t stamp12, double a2,uint64_t stamp13, double a3)
{
    cout << fixed;
    cout << setprecision(20);
    long double stamp,stamp0,stamp1,stamp2,stamp3;
    long double f01,f12,f23,f012,f123,f0123;
    long double N1,N2,N3;

    stamp=((double)stamp_/1000000000.0);
    stamp0=((double)stamp10/1000000000.0);
    stamp1=((double)stamp11/1000000000.0);
    stamp2=((double)stamp12/1000000000.0);
    stamp3=((double)stamp13/1000000000.0);
//    cout<<"stamp1-stamp0     "<<stamp1-stamp0<<endl;
    f01=(a1-a0)/(stamp1-stamp0);
    f12=(a2-a1)/(stamp2-stamp1);
    f23=(a3-a2)/(stamp3-stamp2);
    f012=(f12-f01)/(stamp2-stamp0);
    f123=(f23-f12)/(stamp3-stamp1);
    f0123=(f123-f012)/(stamp3-stamp0);
    N1=a0+(stamp-stamp0)*f01;
    N2=N1+(stamp-stamp0)*f012*(stamp-stamp1);
    N3=N2+(stamp-stamp0)*f0123*(stamp-stamp1)*(stamp-stamp2);


    if((N3>=a1)&&(N3<=a2)||(N3<=a1)&&(N3>=a2))
    {
        cout<<"right"<<endl;
    }
    else
    {
        if((fabs(a1/a2)>1.02)||(fabs(a2/a1)>1.02))
        {
            cout<<"stamp   "<<stamp <<endl;
            cout<<"stamp0   "<<stamp0 <<endl;
            cout<<"stamp1   "<<stamp1 <<endl;
            cout<<"stamp2   "<<stamp2 <<endl;
            cout<<"stamp3   "<<stamp3 <<endl;

            cout<<"a0   "<<a0 <<endl;
            cout<<"a1   "<<a1 <<endl;
            cout<<"a2   "<<a2 <<endl;
            cout<<"a3   "<<a3 <<endl;

            cout<<"f01  "<<f01<<endl;
            cout<<"f12  "<<f12<<endl;
            cout<<"f23  "<<f23<<endl;
            cout<<"f012  "<<f012<<endl;
            cout<<"f123  "<<f123<<endl;
            cout<<"f0123  "<<f0123<<endl;
            cout<<"N1  "<<N1<<endl;
            cout<<"N2  "<<N2<<endl;
            cout<<"N3  "<<N3<<endl;
            cout<<"------------------------------------------------------------------------------------------error"<<endl;
        }

    }
    return N3;
}
 */
sensor_msgs::ImuPtr HermiteImu(sensor_msgs::ImuPtr imu0,sensor_msgs::ImuPtr imu1,sensor_msgs::ImuPtr imu2,sensor_msgs::ImuPtr imu3,ros::Time image_time)
{
/*
    cout<<"input:"<<endl;
    cout<<"imu0 "<<"stamp: "<<imu0->header.stamp<<endl<<"acc:"<<endl<<imu0->linear_acceleration<<endl<<"gyr:"<<endl<<imu0->angular_velocity<<endl;
    cout<<"imu1 "<<"stamp: "<<imu1->header.stamp<<endl<<"acc:"<<endl<<imu1->linear_acceleration<<endl<<"gyr:"<<endl<<imu1->angular_velocity<<endl;
    cout<<"imu2 "<<"stamp: "<<imu2->header.stamp<<endl<<"acc:"<<endl<<imu2->linear_acceleration<<endl<<"gyr:"<<endl<<imu2->angular_velocity<<endl;
    cout<<"imu3 "<<"stamp: "<<imu3->header.stamp<<endl<<"acc:"<<endl<<imu3->linear_acceleration<<endl<<"gyr:"<<endl<<imu3->angular_velocity<<endl;
    */
    sensor_msgs::ImuPtr imu(new sensor_msgs::Imu(*imu0));
    uint64_t stamp;

    double ax,ay,az;
    double gx,gy,gz;
    stamp=image_time.toNSec();
    gx=Hermite(stamp,imu0->header.stamp.toNSec(),imu0->angular_velocity.x,imu1->header.stamp.toNSec(),imu1->angular_velocity.x,imu2->header.stamp.toNSec(),imu2->angular_velocity.x,imu3->header.stamp.toNSec(),imu3->angular_velocity.x);
    gy=Hermite(stamp,imu0->header.stamp.toNSec(),imu0->angular_velocity.y,imu1->header.stamp.toNSec(),imu1->angular_velocity.y,imu2->header.stamp.toNSec(),imu2->angular_velocity.y,imu3->header.stamp.toNSec(),imu3->angular_velocity.y);
    gz=Hermite(stamp,imu0->header.stamp.toNSec(),imu0->angular_velocity.z,imu1->header.stamp.toNSec(),imu1->angular_velocity.z,imu2->header.stamp.toNSec(),imu2->angular_velocity.z,imu3->header.stamp.toNSec(),imu3->angular_velocity.z);

    ax=Hermite(stamp,imu0->header.stamp.toNSec(),imu0->linear_acceleration.x,imu1->header.stamp.toNSec(),imu1->linear_acceleration.x,imu2->header.stamp.toNSec(),imu2->linear_acceleration.x,imu3->header.stamp.toNSec(),imu3->linear_acceleration.x);
    ay=Hermite(stamp,imu0->header.stamp.toNSec(),imu0->linear_acceleration.y,imu1->header.stamp.toNSec(),imu1->linear_acceleration.y,imu2->header.stamp.toNSec(),imu2->linear_acceleration.y,imu3->header.stamp.toNSec(),imu3->linear_acceleration.y);
    az=Hermite(stamp,imu0->header.stamp.toNSec(),imu0->linear_acceleration.z,imu1->header.stamp.toNSec(),imu1->linear_acceleration.z,imu2->header.stamp.toNSec(),imu2->linear_acceleration.z,imu3->header.stamp.toNSec(),imu3->linear_acceleration.z);

    imu->header.stamp=image_time;
    imu->angular_velocity.x=gx;
    imu->angular_velocity.y=gy;
    imu->angular_velocity.z=gz;
    imu->linear_acceleration.x=ax;
    imu->linear_acceleration.y=ay;
    imu->linear_acceleration.z=az;
//    cout<<"imu "<<"stamp: "<<imu->header.stamp<<endl<<"acc:"<<endl<<imu->linear_acceleration<<endl<<"gyr:"<<endl<<imu->angular_velocity<<endl;
    return imu;
}

vector<pair<vector<sensor_msgs::ImuPtr>,sensor_msgs::ImagePtr>> getMeasurement()
{
    cout<<image_buf.size()<<"     "<<imu_buf.size()<<endl;
    //注意这里为什么把measurements定义在while循环的外面，因为getMeasurement()一直循环进行，而且要得到measurements，定义在里面会导致清空
    vector<pair<vector<sensor_msgs::ImuPtr>,sensor_msgs::ImagePtr>> measurements;
    while (true)
    {
        pair<vector<sensor_msgs::ImuPtr>,sensor_msgs::ImagePtr> measurement;
        vector<sensor_msgs::ImuPtr> imus;
        sensor_msgs::ImagePtr img;
        //image信息或IMU信息太少,设置成50保证image对应足够的imu
        imus.clear();
        if((image_buf.size()<3)||imu_buf.size()<20)
        {
            return measurements;
        }
        //刚开始的时候
        if(first_image==nullptr)
        {
            //IMU信息比Image信息来的晚
            if(imu_buf.front()->header.stamp>image_buf.front()->header.stamp)
            {
                ROS_INFO("IMU is later than image,wait for imu and delete image");
                image_buf.pop();
                continue;
            }
            if(imu_buf.front()->header.stamp<image_buf.front()->header.stamp)
            {
                img=image_buf.front();
                image_first_time=img->header.stamp;
                first_image=image_buf.front();
                image_buf.pop();
                image++;//判断这是第几帧（系统的首帧作为第0帧）
            }
        }
        //表示进入连续跟踪模式
        if(image>0)
        {
            //只保留第一帧图像最近两个的imu进行差值
            if(image==1)
            {
                while (imu_buf.front()->header.stamp<=image_first_time)
                {
                    if(imu1!= nullptr)
                        imu0=imu1;//todo 需要判定图像帧之前是否存在两帧以上的imu信息
                    imu1=imu_buf.front();
                    imu_buf.pop();
                }
                if(imu0== nullptr)
                {
                    imu0=imu1;//todo 需要判定图像帧之前是否存在两帧以上的imu信息
                }
                imu_begin=imu1;
                imus.emplace_back(imu_begin);
            }
            ///添加 imus
            if(image!=1)
            {
                image_first_time=image_second_time;//在连续运行的时候保证first time是上一次的second time
                imu_begin=imu_end;//todo 赋值有问题
                imus.emplace_back(imu_begin);
                imus.emplace_back(imu6);
            }
            image_second_time=image_buf.front()->header.stamp;
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


            if(imu4== nullptr) //保证imu4不为空
            {
                ROS_INFO("set imu4 , maybe wrong");
                imu4=imu5;
            }
            imu_end=imu5;  //对imu_end赋初值
            //todo imu和图像之间额顺序混的话需要给出错误提示
            //添加imus结束
            imu6=imu_buf.front();
            imu_buf.pop();
            imu7=imu_buf.front();
            if(image==1)
            {
                imu_begin=HermiteImu(imu0,imu1,imu2,imu3,image_first_time);
                imus.front()=imu_begin;
                imu_end=HermiteImu(imu4,imu5,imu6,imu7,image_second_time);
                imus.emplace_back(imu_end);
            }
            else
            {
                imu_end=HermiteImu(imu4,imu5,imu6,imu7,image_second_time);
                imus.emplace_back(imu_end);
            }
            measurement=(make_pair(imus,image_buf.front()));
            measurements.push_back(measurement);
            image_buf.pop();//这个必须要有
            /*至此，得到的信息应该是这样的：（其中，imu_buf在imu6之前的（包括6）都被pop了，imu_buf的front指针指向7）
             * 图像序列也被pop了，现在的front是接下来要处理的image
            *           0123                4567
            * ,IMU序列  ||||||||||||||||||||||||
            *           -----------------------
            *   图像序列  |                   |
            *     imu_begin,2,3,……,4,5,imu_end;
            */
            image++;
        }
    }
}
System::Ptr System=System::creat();

/**
 * @brief 处理图像数据
 * （1）光流跟踪提取特征
 * （2）计算位姿
 * （3）三角化恢复特征点
 */
void Image_process(sensor_msgs::ImagePtr image)
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
        vector<pair<vector<sensor_msgs::ImuPtr>,sensor_msgs::ImagePtr>> measurements;
        pair<vector<sensor_msgs::ImuPtr>,sensor_msgs::ImagePtr> measurement;

        unique_lock<mutex> lk(load_lock);
        con.wait(lk,[&]
        {
           return (measurements=getMeasurement()).size()!=0;// 等价于 while(!measurement=getMeasurement()).first.size()!=0) { cv.wait(lk); }
        });
        lk.unlock();
        cout<<"measurement: "<<measurements.size()<<endl;
        for(auto &measure:measurements)
        {
            for(auto imu_:measure.first)
            {
                imustxt<<imu_->header.stamp<<" "<<imu_->linear_acceleration.x<<" "<<imu_->linear_acceleration.y<<" "<<imu_->linear_acceleration.z<<endl;
            }

            imustxt<<endl;
            System->track(measure);
            Optimize();
        }

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
