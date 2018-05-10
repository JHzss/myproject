//
// Created by jh on 18-4-12.
//
#include "Publishers.h"

ros::Publisher pub_image;
void setPublishers(ros::NodeHandle &n)
{
    pub_image=n.advertise<sensor_msgs::Image>("raw_image_myproject",1000);
    cout<<"set publisher over"<<endl;
}
void PubImage(sensor_msgs::ImageConstPtr &raw_image)
{
    pub_image.publish(raw_image);
    cout<<"pub over"<<endl;
}
