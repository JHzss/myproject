//
// Created by jh on 18-3-12.
//

#include "System.h"
#include "imu_process.h"


fstream pre_imu_txt("/home/jh/catkin_ws/src/myproject/pre_imu.txt", ios::out);

System::System():SystemStatus_(Init),currFrame_s(nullptr),refKeyFrame_s(nullptr),init_flag_s(false),load_init_flag_s(false)
{
    tracker_s=Feature_tracking::creat(image_width,image_height);         //特征跟踪
    tracker_s->min_init_dist=init_dist;
    cout<<"set dist"<<init_dist<<endl;
    frame_count=0;
}
/**
 * @brief 处理IMU数据，包括：
 * （1）hermite插值法将IMU首尾数据进行对齐
 * （2）预积分（中值积分），更新J矩阵和P矩阵
 * （3）得到预积分类，一组imus对应一个实例
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
        if(imu_number==0)
        {
            ROS_INFO("init preintegration %d",frame_count);
            image_ba_s.push_back(Vector3d(0,0,0));
            image_bg_s.push_back(Vector3d(0,0,0));
            PreIntegration::Ptr pre_integration_tmp=PreIntegration::creat(acc_0,acc_1,gyr_0,gyr_1,image_ba_s[frame_count],image_bg_s[frame_count],dt);
            preIntegrations.emplace_back(pre_integration_tmp);
//            preIntegrations[frame_count]=PreIntegration::creat(acc_0,acc_1,gyr_0,gyr_1,ba[frame_count],bg[frame_count],dt);
            preIntegrations[frame_count]->acc_1=Vector3d(ax,ay,az);//本次IMU数据组成
            preIntegrations[frame_count]->gyr_1=Vector3d(gx,gy,gz);
        }
        if(imu_number!=0)
        {
            preIntegrations[frame_count]->dt=dt;
            preIntegrations[frame_count]->sum_t+=dt;
            preIntegrations[frame_count]->acc_0=preIntegrations[frame_count]->acc_1;
            preIntegrations[frame_count]->gyr_0=preIntegrations[frame_count]->gyr_1;
            preIntegrations[frame_count]->acc_1=Vector3d(ax,ay,az);
            preIntegrations[frame_count]->gyr_1=Vector3d(gx,gy,gz);
            //todo 开始对预积分进行操作

            preIntegrations[frame_count]->run();
            //todo 将IMU和图像对应上，需要定义frame_count,预积分
        }
        imu_number++;//表示这两张图像之间的IMU计数，begin对应imu-0
    }
    preIntegrations[frame_count]->img_stamp=imus.back()->header.stamp.toSec();
    pre_imu_txt<<fixed;
    pre_imu_txt<<preIntegrations[frame_count]->img_stamp<<"  "<<preIntegrations[frame_count]->sum_t<<endl;
    cout<<"p:"<<endl<<preIntegrations[frame_count]->dp<<endl;
    cout<<"v:"<<endl<<preIntegrations[frame_count]->dv<<endl;
    cout<<"q:"<<endl<<preIntegrations[frame_count]->dq.toRotationMatrix()<<endl;
    pre_imu_txt<<"dp: "<<preIntegrations[frame_count]->dp(0)<<" "<<preIntegrations[frame_count]->dp(1)<<" "<<preIntegrations[frame_count]->dp(2)<<endl;
    pre_imu_txt<<"dv: "<<preIntegrations[frame_count]->dv(0)<<" "<<preIntegrations[frame_count]->dv(1)<<" "<<preIntegrations[frame_count]->dv(2)<<endl;
    pre_imu_txt<<"dq: "<<preIntegrations[frame_count]->dq.w()<<" "<<preIntegrations[frame_count]->dq.x()<<" "<<preIntegrations[frame_count]->dq.y()<<" "<<preIntegrations[frame_count]->dq.z()<<endl;
    ROS_INFO("finish preintegration %d",frame_count);
}
void System::Image_process(sensor_msgs::ImageConstPtr image_ptr_)
{
    cv_bridge::CvImagePtr image_ptr=cv_bridge::toCvCopy(image_ptr_,sensor_msgs::image_encodings::MONO8);
    Mat image=image_ptr->image;
    double cur_timestamp=image_ptr->header.stamp.toSec();
    cout<<"load image successfully!"<<endl;
    currImg_s=image.clone();
    currFrame_s=Frame::creat(currImg_s,cur_timestamp);
    all_Frames_s.push_back(currFrame_s);
    cout<<"frame id"<<currFrame_s->id_<<endl;

    if(SystemStatus_==Init)
    {
        if(!load_init_flag_s)//初始化需要的图片还不够
        {
            //todo 添加更多的帧用于初始化
            cout<<"---------------------------------------no enough image, go on---------------------------------------------"<<endl;
            load_init_flag_s= tracker_s->loadInitImage(currImg_s,currFrame_s);//加载初始化用到的图片,并提取特征点和匹配点,找出具有相对位移的帧
        }
        if(load_init_flag_s)//加载了足够的图片用于初始化
        {
            cout<<"--------------------------------------------------begin initialization---------------------------------------------"<<endl;
            result_flag_s=tracker_s->initialization();  //todo 10帧的初始化操作
            if(result_flag_s)
            {
                SystemStatus_=Track_good;
            }
            initialization_frame_count=tracker_s->init_l;
        }
        frame_count++;//todo 暂时放在这里
    }
    else if(SystemStatus_==Track_good)      //跟踪效果好（特征点匹配多）
    {
        //tracker_s->track_new();
        slideWindow();
        //todo 设置状态
    }
    else if(SystemStatus_==Track_bad)       //跟踪效果不好（特征匹配少）
    {
        Optimizer::windowBA();//todo 跟踪效果不好时加个优化
        //tracker_s->track_new();
        slideWindow();
        //todo 设置状态
    }
    else
    {
        SystemStatus_=Init;//todo 想想跟踪失败后应该如何做
    }

}
void System::Imu_Visual_align()
{
    /**
     * 陀螺仪bias矫正
     */
    {
        Matrix3d A;
        Vector3d b;
        Vector3d delta_bw;
        A.setZero();
        b.setZero();
        for(int i=0;i<initialization_frame_count;i++)
        {
            Matrix3d temp_A;
            Vector3d temp_b;
            temp_A.setZero();
            temp_b.setZero();

            Eigen::Quaterniond Qc2b(eigen_Rc2b);
            Eigen::Quaterniond temp_q1,temp_q2,temp_q;

            temp_q1=all_Frames_s[i]->pose_q;
            temp_q1=Qc2b*temp_q1;//转换成qw2b
            temp_q2=all_Frames_s[i+1]->pose_q;
            temp_q2=Qc2b*temp_q2;
            temp_A=0.5*preIntegrations[i]->jacobian.block(O_R,O_BG,3,3);//todo 可能有错
            A+=temp_A.transpose()*temp_A;//VINS里面乘了转置，应该是没有必要吧。
            temp_q=preIntegrations[i]->dq*temp_q1*temp_q2.inverse();
            temp_b=temp_q.vec();//VINS中像最小二乘一样乘上了转置ATAx=ATb，但是感觉这里并不需要
            b+=temp_A.transpose()*temp_b;
        }
        delta_bw=A.ldlt().solve(b);//保证A为实对称正定矩阵
        cout<<"correct bw:"<<endl<<delta_bw<<endl;
        waitKey(0);
        for(int i=0;i<initialization_frame_count;i++)
            image_bg_s[i]+=delta_bw;
        for(int i=0;i<initialization_frame_count;i++)
        {
            preIntegrations[i]->bg_pre=image_bg_s[i];
            preIntegrations[i]->rerun();
        }
    }

    /**
     * scale gravity vectory
     */
    {
        int frame_count=initialization_frame_count;//总的帧数
        cout<<"number:"<<frame_count<<endl;
        int dimension= frame_count*3+3+1;
        MatrixXd A(dimension,dimension);
        Eigen::VectorXd b(dimension);

        for(int i=0;i<frame_count;i++)
        {
            MatrixXd temp_A(6,10);
            temp_A.setZero();
            Eigen::VectorXd temp_b(6);
            temp_b.setZero();
            int j=i+1;
            double dt=preIntegrations[j]->sum_t;
            temp_A.block<3,3>(0,0)=-1*dt*Matrix3d::Identity();
            temp_A.block<3,3>(0,6)=0.5*(eigen_Rc2b*all_Frames_s[i]->pose_q.toRotationMatrix())*dt*dt;
            temp_A.block<3,1>(0,9)=(eigen_Rc2b*all_Frames_s[i]->pose_q.toRotationMatrix())*(-1*all_Frames_s[j]->pose_q.toRotationMatrix().inverse()*all_Frames_s[j]->pose_t+all_Frames_s[i]->pose_q.toRotationMatrix().inverse()*all_Frames_s[i]->pose_t);
            temp_A.block<3,3>(3,0)=-1*Matrix3d::Identity();
            temp_A.block<3,3>(3,3)=(eigen_Rc2b*all_Frames_s[i]->pose_q.toRotationMatrix())*(all_Frames_s[j]->pose_q.toRotationMatrix().inverse()*eigen_Rc2b.inverse());
            temp_A.block<3,3>(3,6)=(eigen_Rc2b*all_Frames_s[i]->pose_q.toRotationMatrix())*dt;
            temp_b.block<3,1>(0,0)=preIntegrations[j]->dp-eigen_tc2b+temp_A.block<3,3>(3,3)*eigen_tc2b;
            temp_b.block<3,1>(3,0)=preIntegrations[j]->dv;
            cout<<"A:"<<temp_A<<endl;
            cout<<"b:"<<temp_b<<endl;
            waitKey(0);

            MatrixXd r_A=temp_A.transpose()*temp_A;
            Eigen::VectorXd r_b=temp_A.transpose()*temp_b;

            A.block<6,6>(i*3,i*3)+=r_A.topLeftCorner<6,6>();
            A.block<6,4>(i*3,dimension-4)+=r_A.topRightCorner<6,4>();
            A.block<4,6>(dimension-4,i*3)+=r_A.bottomLeftCorner<4,6>();
            A.bottomRightCorner<4,4>()+=r_A.bottomRightCorner<4,4>();

//            b.block<6,1>(i*3,0)+=r_b.block<6,1>(0,0); //block 用于矩阵的运算
//            b.bottomLeftCorner<4,1>()+=r_b.bottomLeftCorner<4,1>();
            b.segment<6>(i * 3) += r_b.head<6>();
            b.tail<4>() += r_b.tail<4>();
        }

        MatrixXd x;
        x=A.ldlt().solve(b);
        cout<<"g"<<endl<<x<<endl;

        waitKey(0);
    }
}

void System::track(pair<vector<sensor_msgs::ImuPtr>,sensor_msgs::ImageConstPtr> measurement)
{
    cout<<"----------------------------------------------frame count------------------------------------"<<frame_count<<endl;
    //处理IMU
    Imu_process(measurement.first);//todo 初始化完成之前是否要先终止预积分
    //处理image
    Image_process(measurement.second);

    if(result_flag_s)
    {
        result_flag_s=false;
        for(int i=0;i<all_Frames_s.size();i++)
        {
            all_Frames_s[i]->pose_q=tracker_s->Q_s[i];//设置frame的状态
            all_Frames_s[i]->pose_t=tracker_s->P_s[i];
        }
        Imu_Visual_align();
    }






}


void System::slideWindow()
{

}

