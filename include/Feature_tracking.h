//
// Created by jh on 18-3-8.
//
#pragma once
#ifndef MYPROJECT_FEATURE_TRACKING_H
#define MYPROJECT_FEATURE_TRACKING_H

#include "myheader.h"
#include "parameters.h"
#include "Camera.h"
#include "Read_dataset.h"
#include "Feature.h"
#include "Frame.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>


class Feature_tracking
{
public:
    typedef shared_ptr<Feature_tracking > Ptr;
    Feature_tracking(int width, int height);

    inline static Feature_tracking::Ptr creat(int width, int height){return Feature_tracking::Ptr(new Feature_tracking(width,height));}

    vector<Point2f> getFeaturepointsIn(uint64_t& frame_id);
    vector<uint64_t > getFeatureIdIn(uint64_t& frame_id);


    bool loadInitImage(const Mat& image,Frame::Ptr& frame);
    bool initialization();    //初始化
      bool recoverRT();       //2D-2D恢复姿态
      void recoverStructure();//恢复总的状态
      bool optimize();//恢复总的状态
    void clearstate();
    void initReset();
    void track_new(uint64_t& pre_frame_id,uint64_t& cur_frame_id);
    void point_filter(vector<uint64_t > &ids,vector<Point2f> &points1,vector<Point2f> &points2);                                      //利用F矩阵剔除 id，point，point
    void point_filter(vector<uint64_t > &ids,vector<Point2f> &points1,vector<Point2f> &points2,vector<Point3d> &points_position_norm);//利用F矩阵剔除 id，point，point，3d
    void point_filter(vector<Point2f> &points1,vector<Point2f> &points2);                                              //利用F矩阵剔除 point，point，求RT的时候用
    void reduceVector(vector<uint64_t > &ids,vector<Point2f> &points1,vector<Point2f> &points2, vector<uchar>& status);//剔除 id，point，point
    void reduceVector(vector<Point3d> &points1, vector<uchar> &status);                                                //剔除3d
    void reduceVector(vector<Point2f> &points1,vector<Point2f> &points2,vector<uchar> &status);                        //剔除 point，point，求RT的时候用

    void recoverMoreFeatures();
    void recover3dPoseOfFeature();
    void solveCamPoseByPnP(int first,int second);
    void solvePointsByTri(int i,int j,vector<uint64_t >& pre_points_id_copy, vector<Point2f>& pre_points_f_copy,vector<Point2f>& cur_points_f_copy);
    void addFeature(Point2f &point,uint64_t &frame_id);
    void fuseFeatures(uint64_t id);
public:
    int init_l;  //初始化时使用的帧的id 实际上是第l+1帧,即0 1 2 3 …… l
    uint64_t init_frame_flag;  //满足相对位移的帧，即与第一帧做初始化的帧数，从0计数
    float aver_k,aver_dist,dist, aver_k_raw;  //定义运动像素
    int width_,height_;
    float min_init_dist;
    int init_frame_num;
    int init_frame_count;//初始化用到的帧的个数
    bool init_flag= true;

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
    vector<Mat> allImg_f;//所有初始化用到的图像
    vector<Sophus::SE3> init_pose_se3_f;
    vector<Eigen::Quaterniond> Q_s;//相机位姿
    vector<Vector3d> P_s;
    vector<Feature::Ptr> features_f;

    //因为这里需要传入参数计算残差，所以定义了一个比较复杂的结构体
    struct ReprojectionError
    {
        double point_u,point_v;
        ReprojectionError(double u_,double v_):point_u(u_),point_v(v_)//构造函数
        {}
        template <typename T>
        bool operator()(const T* const camera_R, const T* const camera_T, const T* point, T* residuals) const
        {
            T pose[3];
            //Pc=Rw2c*Pw+Tw2c
            ceres::QuaternionRotatePoint(camera_R,point,pose);//Rotates a point pt by a quaternion q, pose is the result
            pose[0]+=camera_T[0];
            pose[1]+=camera_T[1];
            pose[2]+=camera_T[2];
            T p_u=pose[0]/pose[2];
            T p_v=pose[1]/pose[2];
            residuals[0]=p_u-T(point_u);
            residuals[1]=p_v-T(point_v);
            return true;
        }

        static ceres::CostFunction* creat(const double point_u,const double point_v)
        {
            return (new ceres::AutoDiffCostFunction<ReprojectionError,2,4,3,3>(new ReprojectionError(point_u,point_v)));
        }

    };


};


#endif //MYPROJECT_FEATURE_TRACKING_H
