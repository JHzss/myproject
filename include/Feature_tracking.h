//
// Created by jh on 18-3-8.
//

#ifndef MYPROJECT_FEATURE_TRACKING_H
#define MYPROJECT_FEATURE_TRACKING_H

#include "myheader.h"
#include "parameters.h"
#include "Camera.h"
#include "Read_dataset.h"
#include "Feature.h"
#include "Frame.h"


class Feature_tracking
{
public:
    typedef shared_ptr<Feature_tracking > Ptr;
    Feature_tracking(int width, int height,const Parameters::Ptr& para);

    inline static Feature_tracking::Ptr creat(int width, int height,const Parameters::Ptr& para){return Feature_tracking::Ptr(new Feature_tracking(width,height,para));}

    vector<Point2f> getFeaturepointsIn(uint64_t& frame_id);
    vector<uint64_t > getFeatureIdIn(uint64_t& frame_id);


    bool loadInitImage(const Mat& image,Frame::Ptr& frame,Parameters::Ptr &para);
    bool initialization(const Parameters::Ptr& para);    //初始化
      bool recoverRT();       //2D-2D恢复姿态
      void recoverStructure(const Parameters::Ptr& para);//恢复总的状态
    void clearstate();
    void initReset();
    void track_new(uint64_t& pre_frame_id,uint64_t& cur_frame_id);
    void point_filter(vector<uint64_t > &ids,vector<Point2f> &points1,vector<Point2f> &points2);                                      //利用F矩阵剔除 id，point，point
    void point_filter(vector<uint64_t > &ids,vector<Point2f> &points1,vector<Point2f> &points2,vector<Point3d> &points_position_norm);//利用F矩阵剔除 id，point，point，3d
    void point_filter(vector<Point2f> &points1,vector<Point2f> &points2);                                              //利用F矩阵剔除 point，point，求RT的时候用
    void reduceVector(vector<uint64_t > &ids,vector<Point2f> &points1,vector<Point2f> &points2, vector<uchar>& status);//剔除 id，point，point
    void reduceVector(vector<Point3d> &points1, vector<uchar> &status);                                                //剔除3d
    void reduceVector(vector<Point2f> &points1,vector<Point2f> &points2,vector<uchar> &status);                        //剔除 point，point，求RT的时候用


    void solveCamPoseByPnP(int first,int second);
    void solvePointsByTri(int i,int j,vector<uint64_t >& pre_points_id_copy, vector<Point2f>& pre_points_f_copy,vector<Point2f>& cur_points_f_copy);
    void addFeature(Point2f &point,uint64_t &frame_id);
    void fuseFeatures(uint64_t id);
public:
    int init_l;  //初始化时使用的帧的id 实际上是第l+1帧
    int init_frame_flag;  //满足相对位移的帧，即与第一帧做初始化的帧数
    float aver_x,aver_y,aver_k,aver_dist,dist, aver_k_raw;  //定义运动像素
    const int width_,height_,min_init_dist;
    int init_frame_num;
    int init_frame_count;
    bool init_flag= true;
    Mat camera_k;

    Mat pre_img_f,cur_img_f;
    Frame::Ptr pre_frame_f,cur_frame_f;


    Mat mask; //提取特征时采用的标记
    Mat F_f,E_f;
    Mat relative_R,relative_t;
    Eigen::Matrix3d eigen_R;
    Eigen::Vector3d eigen_t;
//    vector<map<uint64_t,vector<pair<uint64_t,Point> > > > feature_uv;//map<feature_id,map<frame_id,uv>>
//    vector<map<uint64_t,vector<pair<uint64_t,Point2d> > > > feature_camera;//map<feature_id,map<frame_id,feature_camera_norm>>

    vector<Point2d> pre_points_cam_f,cur_points_cam_f;
    SE3 camera_pose;
    vector<Point3d> points_3d;

    Mat R_estimate,t_estimate;
    vector<DMatch> matches; //第1帧和第l帧之间的orb特征匹配
    vector<DMatch> good_matches;//经过筛选的匹配

    vector<Mat> initImg_f;//所有初始化用到的图像
    vector<vector<Point2f> > initFeatures_f;//所有的初始化用到的帧用光流提取出的特征点
    vector<vector<Point2f> > initFeatures_f_new;//所有的光流没跟到，新检测到的点
//    vector<vector<Point2f> > raw_Features_f;//光流直接跟踪，没有剔除
    vector<vector<uchar> > raw_status_f;
    //

    vector<KeyPoint> first_keypoints_f;//第一帧的keypoint类型的特征点，用ORB特征提取器提取出来的
    vector<Sophus::SE3> init_pose_se3_f;
    vector<Quaterniond> init_Qs;
    vector<Vector3d> init_ts;
    vector<Feature::Ptr> features_f;


};


#endif //MYPROJECT_FEATURE_TRACKING_H
