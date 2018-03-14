//
// Created by jh on 18-3-8.
//

#include "Feature_tracking.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/opencv.hpp>
#include "myheader.h"

bool cmp(DMatch m1,DMatch m2)
{
    return m1.distance<m2.distance;
}
vector<Point3d> normalization(Mat points_position_)
{

    vector<Point3d> points_position_norm_;
    for(int j=0;j<points_position_.cols;j++)
    {
        Point3d point_3d_norm_;
        point_3d_norm_.x=points_position_.col(j).at<double>(0,0)/points_position_.col(j).at<double>(3,0);
        point_3d_norm_.y=points_position_.col(j).at<double>(1,0)/points_position_.col(j).at<double>(3,0);
        point_3d_norm_.z=points_position_.col(j).at<double>(2,0)/points_position_.col(j).at<double>(3,0);
        points_position_norm_.push_back(point_3d_norm_);
    }
    return points_position_norm_;

}

Feature_tracking::Feature_tracking()
{

}

void Feature_tracking::loadImage(const string& img_filename)
{
    Mat img;
    img=imread(img_filename);
    cur_img=img;
    imshow("im",cur_img);
}

bool Feature_tracking::initialization(const Parameters::Ptr& param, Camera::Ptr& camera)
{

    //想要替换其他参数在参数头文件里添加即可。
    cv::Ptr<ORB> orb_detecter=cv::ORB::create(param->number_of_features,1.2f,8,31,0,2,ORB::HARRIS_SCORE,31,20);

    BFMatcher matcher(NORM_HAMMING);
    Mat Essential;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    R.setIdentity();
    t.setZero();
    SE3 cam_pose(R,t);
    camera_pose=cam_pose;


    orb_detecter->detect(cur_img,cur_keypoints);
    orb_detecter->compute(cur_img,cur_keypoints,cur_descriptor);
    if(pre_keypoints.empty())
    {
        pre_keypoints.assign(cur_keypoints.begin(),cur_keypoints.end());
        pre_img=cur_img.clone();
        pre_descriptor=cur_descriptor.clone();

    }

    matcher.match(pre_descriptor,cur_descriptor,matches);//会将所有的特征进行匹配，所以总数是500

    double min_dist=min_element(matches.begin(),matches.end(),cmp )->distance;
    cout<<min_dist<<endl;

    for(DMatch m:matches)
    {
        if(m.distance<max(2*min_dist,30.0))
        {
            good_matches.push_back(m);
        }

    }
    cout<<good_matches.size()<<endl;

    for(DMatch m:good_matches)
    {
        //cout<<"pre_NO."<<m.queryIdx<<endl;
        //cout<<"cur_NO."<<m.trainIdx<<endl;
        pre_points.push_back(pre_keypoints[m.queryIdx].pt);
        cur_points.push_back(cur_keypoints[m.trainIdx].pt);
    }

    //2018.3.6晚，完成2D-2D特征匹配。
    //TODO 计算E/F矩阵
    Essential=findEssentialMat(pre_points,cur_points,camera->getK(),RANSAC,0.999,1.0);
//        cout<<"Essential matrix:"<<endl;
//        cout<<Essential<<endl;
    recoverPose(Essential,pre_points,cur_points,camera->getK(),R_estimate,t_estimate);


    R<<R_estimate.at<double>(0,0),R_estimate.at<double>(0,1),R_estimate.at<double>(0,2),
            R_estimate.at<double>(1,0),R_estimate.at<double>(1,1),R_estimate.at<double>(1,2),
            R_estimate.at<double>(2,0),R_estimate.at<double>(2,1),R_estimate.at<double>(2,2);
    t<<t_estimate.at<double>(0,0),t_estimate.at<double>(1,0),t_estimate.at<double>(2,0);

    SE3 pose_new(R,t);

    Mat initial_pose=(Mat_<double>(3,4) <<
                                        1,0,0,0,
            0,1,0,0,
            0,0,1,0);
    Mat first_pose=(Mat_<double>(3,4)<<
                                     R_estimate.at<double>(0,0),R_estimate.at<double>(0,1),R_estimate.at<double>(0,2),t_estimate.at<double>(0,0),
            R_estimate.at<double>(1,0),R_estimate.at<double>(1,1),R_estimate.at<double>(1,2),t_estimate.at<double>(1,0),
            R_estimate.at<double>(2,0),R_estimate.at<double>(2,1),R_estimate.at<double>(2,2),t_estimate.at<double>(2,0));

    Mat points_position;

    for(int j=0;j<pre_points.size();j++)
    {
        pre_points_cam.push_back(Camera::uv2camera(pre_points[j],camera->getK()));
        cur_points_cam.push_back(Camera::uv2camera(cur_points[j],camera->getK()));
    }

    triangulatePoints(initial_pose,first_pose,pre_points_cam,cur_points_cam,points_position);
    points_position_norm=normalization(points_position);//归一化坐标


    pre_keypoints.clear();
    pre_keypoints.assign(cur_keypoints.begin(),cur_keypoints.end());
    pre_img=cur_img.clone();
    pre_descriptor=cur_descriptor.clone();

}

void Feature_tracking::clearstate()
{
    R_estimate=(Mat_<double >(3,3)<<1.0,0,0,0,1.0,0,0,0,1.0);
    t_estimate=(Mat_<double >(3,1)<<0,0,0);
    good_matches.clear();
    matches.clear();
    cur_keypoints.clear();
}