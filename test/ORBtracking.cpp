//
// Created by jh on 18-3-6.
//
#include "myheader.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/opencv.hpp>

#include "Camera.h"
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

int main(int argc,char** argv)
{
    if(argc!=2)
    {
        cout<<"usage:node  data_file_name"<<endl;
        return 1;
    }

    string data_file=argv[1];
    string rgb_time,rgb_name,depth_time,depth_name;
    vector<string> rgb_files,depth_files;
    vector<double > rgb_times,depth_times;
    ifstream fin(data_file+"/associate.txt");
    while(!fin.eof())
    {
        fin>>rgb_time>>rgb_name>>depth_time>>depth_name;
        rgb_times.push_back(atof(rgb_time.c_str()));
        depth_times.push_back(atof(depth_time.c_str()));
        rgb_files.push_back(data_file+"/"+rgb_name);
        depth_files.push_back(data_file+"/"+depth_name);
    }

    Mat pre_img,cur_img;
    Mat pre_descriptor,cur_descriptor;
    vector<KeyPoint> pre_keypoints,cur_keypoints;
    vector<KeyPoint> pre_keypoints_good,cur_keypoints_good;
    Mat img_match;
    Ptr<ORB> orb_detecter=cv::ORB::create(500,1.2f,8,31,0,2,ORB::HARRIS_SCORE,31,20);
    vector<DMatch> matches;
    vector<DMatch> good_matches;
    BFMatcher matcher(NORM_HAMMING);
    Mat Essential,R_estimate,t_estimate;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    R.setIdentity();
    t.setZero();
    Mat K=(Mat_<double>(3,3)<< 517.3,0,318.6,0,516.5,255.3,0,0,1.0);
    vector<Point> pre_points,cur_points;
    vector<Point2d> pre_points_cam,cur_points_cam;
    SE3 camera_pose(R,t);
    vector<Point3d> points_3d;
    vector<Point3d> points_position_norm;

    for(int i=0;i<rgb_files.size();i++)
    {

        R_estimate=(Mat_<double >(3,3)<<1.0,0,0,0,1.0,0,0,0,1.0);
        t_estimate=(Mat_<double >(3,1)<<0,0,0);

        good_matches.clear();
        matches.clear();
        cur_keypoints.clear();
        cur_img=imread(rgb_files[i]);
        if(i==0)
        {
            pre_img=cur_img.clone();
            pre_descriptor=cur_descriptor.clone();
            orb_detecter->detect(cur_img,pre_keypoints);
            orb_detecter->compute(cur_img,pre_keypoints,pre_descriptor);
            continue;
        }
        orb_detecter->detect(cur_img,cur_keypoints);
        orb_detecter->compute(cur_img,cur_keypoints,cur_descriptor);

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
        Essential=findEssentialMat(pre_points,cur_points,K,RANSAC,0.999,1.0);
//        cout<<"Essential matrix:"<<endl;
//        cout<<Essential<<endl;
        recoverPose(Essential,pre_points,cur_points,K,R_estimate,t_estimate);

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
            pre_points_cam.push_back(Camera::uv2camera(pre_points[j],K));
            cur_points_cam.push_back(Camera::uv2camera(cur_points[j],K));
        }

        triangulatePoints(initial_pose,first_pose,pre_points_cam,cur_points_cam,points_position);

        points_position_norm=normalization(points_position);//归一化坐标

        for(auto poi:points_position_norm)
        {
            cout<<"["<<poi.x<<","<<poi.y<<","<<poi.z<<"]"<<endl;
        }


        cout<<"camera pose:"<<endl;
        cout<<pose_new.rotationMatrix()<<endl;
        cout<<pose_new.translation()<<endl;

        drawMatches(pre_img,pre_keypoints,cur_img,cur_keypoints,good_matches,img_match);

        imshow("match",img_match);
        waitKey(0);
        pre_keypoints.clear();
        pre_keypoints.assign(cur_keypoints.begin(),cur_keypoints.end());
        pre_img=cur_img.clone();
        pre_descriptor=cur_descriptor.clone();
    }


    destroyAllWindows();
    return 0;

}