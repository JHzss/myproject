//
// Created by jh on 18-3-8.
//

#ifndef MYPROJECT_FEATURE_TRACKING_H
#define MYPROJECT_FEATURE_TRACKING_H

#include "myheader.h"
#include "parameters.h"
#include "Camera.h"
#include "Read_dataset.h"


class Feature_tracking
{



public:
    typedef shared_ptr<Feature_tracking > Ptr;
    Feature_tracking(int width, int height,const Parameters::Ptr& para);

    inline static Feature_tracking::Ptr creat(int width, int height,const Parameters::Ptr& para){return Feature_tracking::Ptr(new Feature_tracking(width,height,para));}

    bool loadInitImage(const Mat& image);
    bool initialization();
    void recoverRT();
    void clearstate();
    void initReset();
    void track_new();
    void point_filter(vector<Point2f> &points1,vector<Point2f> &points2);             //利用F矩阵剔除误匹配点
public:
    int init_frame_flag;  //满足相对位移的帧，即与第一帧做初始化的帧数
    float aver_x,aver_y,aver_k,dist;  //定义运动像素
    const int width_,height_,min_init_dist;
    int init_frame_num;
    int init_frame_count;
    bool init_flag= true;
    Mat camera_k;
    Mat pre_img_f,cur_img_f;
    Mat init_first_img_f;

    Mat mask; //提取特征时采用的标记
    Mat F_f,E_f;
    Mat relative_R,relative_t;
    Eigen::Matrix3d eigen_R;
    Eigen::Vector3d eigen_t;

//    Mat pre_descriptor,cur_descriptor;
    vector<KeyPoint> pre_keypoints_f,cur_keypoints_f;
    vector<KeyPoint> pre_keypoints_good_f,cur_keypoints_good_f;
    Mat img_match;
//    list<Point2f> pre_points_f,cur_points_f;
    vector<Point2d> pre_points_cam_f,cur_points_cam_f;
    SE3 camera_pose;
    vector<Point3d> points_3d;
    vector<Point3d> points_position_norm;
    Mat R_estimate,t_estimate;
    vector<DMatch> matches;
    vector<DMatch> good_matches;

    vector<Mat> initImg_f;
    vector<vector<Point2f> > initFeatures_f;
    vector<Point2f> first_points_f,orb_points_first,orb_points_l;
    vector<KeyPoint> first_keypoints_f;

    vector<vector<int> > track_times;

};


#endif //MYPROJECT_FEATURE_TRACKING_H
