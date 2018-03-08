//
// Created by jh on 18-3-8.
//

#ifndef MYPROJECT_FEATURE_TRACKING_H
#define MYPROJECT_FEATURE_TRACKING_H

#include "myheader.h"
#include "parameters.h"
#include "Camera.h"
#include "Read_dataset.h"

class Feature_tracking {
public:
    Feature_tracking();
    void loadImage(const string& img_filename);
    bool initialization(const Parameters& param,Camera& camera);
    void clearstate();
public:
    bool init_flag= true;
    Mat pre_img,cur_img;
    Mat pre_descriptor,cur_descriptor;
    vector<KeyPoint> pre_keypoints,cur_keypoints;
    vector<KeyPoint> pre_keypoints_good,cur_keypoints_good;
    Mat img_match;
    vector<Point> pre_points,cur_points;
    vector<Point2d> pre_points_cam,cur_points_cam;
    SE3 camera_pose;
    vector<Point3d> points_3d;
    vector<Point3d> points_position_norm;
    Mat R_estimate,t_estimate;
    vector<DMatch> matches;
    vector<DMatch> good_matches;

};


#endif //MYPROJECT_FEATURE_TRACKING_H
