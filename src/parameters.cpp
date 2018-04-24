//
// Created by jh on 18-3-8.
//
#include "myheader.h"
#include "parameters.h"

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
double camera_fx;
double camera_fy;
double camera_cx;
double camera_cy;
double camera_k1;
double camera_k2;
double camera_p1;
double camera_p2;
double acc_n;
double acc_w;
double gyr_n;
double gyr_w;

int number_of_features,image_width,image_height,slideWindowsize;
float init_dist;
cv::Mat camera_k;

template <typename T>
T readParam(ros::NodeHandle &n, string name)
{
    T para;
    if(n.getParam(name,para))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << para);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return para;
}
void LoadParameters(ros::NodeHandle &n)
{
    string config_file;
    config_file=readParam<std::string>(n,"config_file");
    cv::FileStorage filename(config_file,cv::FileStorage::READ);
    if(!filename.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    else
    {
        cout<<"load config file successfully"<<endl;
    }
    filename["imu_topic"]>>IMU_TOPIC;
    filename["image_topic"]>>IMAGE_TOPIC;
    camera_fx=filename["camera.fx"];
    camera_fy=filename["camera.fy"];
    camera_cx=filename["camera.cx"];
    camera_cy=filename["camera.cy"];

    camera_k1=filename["camera.k1"];
    camera_k2=filename["camera.k2"];
    camera_p1=filename["camera.p1"];
    camera_p2=filename["camera.p2"];

    acc_n=filename["acc_n"];
    acc_w=filename["acc_w"];
    gyr_n=filename["gyr_n"];
    gyr_w=filename["gyr_w"];

    number_of_features=filename["number_of_features"];
    init_dist=filename["init_dist"];

    image_width=filename["image.width"];
    image_height=filename["image.height"];
    slideWindowsize=filename["slideWindowsize"];
    camera_k=(Mat_<double>(3,3)<< camera_fx,0,camera_cx,0,camera_fy,camera_cy,0,0,1.0);
    cout<<"load finished"<<endl;
}
