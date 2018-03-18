//
// Created by jh on 18-3-8.
//

#include "Feature_tracking.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/opencv.hpp>
#include "myheader.h"
void Feature_tracking::track_new()
{

}



Mat showMatch(const Mat& img1,const Mat& img2,const vector<Point2f>& points1,const vector<Point2f>& points2)
{
    Mat img_show;
    vector<Point2f> points1_copy,points2_copy;
    points1_copy.assign(points1.begin(),points1.end());
    points2_copy.assign(points2.begin(),points2.end());
    for(auto iter2=points2_copy.begin();iter2!=points2_copy.end();)
    {
        iter2->x+=640;
        iter2++;
    }
    cv::RNG rng(time(0));
    hconcat(img1,img2,img_show);
    vector<Point2f>::iterator iter1,iter2;
    for(iter1=points1_copy.begin(),iter2=points2_copy.begin();iter1!=points1_copy.end();iter1++,iter2++)
    {
        line(img_show,*iter1,*iter2,cv::Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255)),1);
        circle(img_show,*iter1,1,0,2);
        circle(img_show,*iter2,1,0,2);
    }

    return img_show;
}

//利用F矩阵剔除误匹配
void Feature_tracking::point_filter(vector<Point2f> &points1, vector<Point2f> &points2)
{
    int k=0;
    int count=0;
    vector<uchar> status_F;
    Mat F;
    //根据F矩阵筛选
    F=findFundamentalMat(points1,points2,FM_RANSAC,1.0,0.99,status_F);
    for(auto iter2=points2.begin(),iter1=points1.begin();iter2!=points2.end();k++)
    {
        if(status_F[k]==0)
        {
            points1.erase(iter1);
            points2.erase(iter2);
            count++;
        }
        else
        {
            iter1++;
            iter2++;
        }
    }
    cout<<"F:"<<F<<endl;
    cout<<"F_filter:"<<count<<endl;
    //根据斜率筛选
    for(auto iter2=points2.begin(),iter1=points1.begin();iter2!=points2.end();)
    {
        aver_k=aver_k+fabs((iter2->y-iter1->y)/(iter2->x+(float)640.0-iter1->x));
        iter1++;
        iter2++;
    }
    aver_k=aver_k/(float)points1.size();
    for(auto iter3=points2.begin(),iter4=points1.begin();iter3!=points2.end();)
    {
        if(fabs((iter3->y-iter4->y)/(iter3->x+640-iter4->x))>1.5*aver_k)
        {
            points1.erase(iter4);
            points2.erase(iter3);
        }
        else
        {
            iter4++;
            iter3++;
        }
    }
}

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

Feature_tracking::Feature_tracking(int width, int height,const Parameters::Ptr& para)
        :width_(width),height_(height),init_frame_count(0),aver_x(0),aver_y(0),aver_k(0),min_init_dist(para->init_dist),camera_k(para->camera_k)
{

}

bool Feature_tracking::loadInitImage(const Mat& image )
{
    cout<<"loading..."<<init_frame_count<<"th image"<<endl;

    vector<Point2f> empty_points;
    vector<int> empty_times;
    initFeatures_f.push_back(empty_points);//必须先插进去一个空的，否则无法按照编号插入
    track_times.push_back(empty_times);
    vector<Point2f> pre_points,cur_points;
    vector<uchar> status;
    vector<float> error;

    if (dist<min_init_dist)//数据太少，不能初始化
    {
        aver_x=0;
        aver_y=0;

        initImg_f.push_back(image);
        cur_img_f=image.clone();
        Mat imshow1=cur_img_f.clone();
        if(init_frame_count==0)
        {
            init_first_img_f=cur_img_f.clone();
        }

        if(init_frame_count!=0)
        {
            pre_img_f=initImg_f[init_frame_count-1].clone();
            pre_points.assign(initFeatures_f[init_frame_count-1].begin(),initFeatures_f[init_frame_count-1].end());
        }

        mask = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(255));

        if(init_frame_count!=0)
        {
            calcOpticalFlowPyrLK(init_first_img_f,cur_img_f,first_points_f,cur_points,status,error,Size(21, 21), 3);

            int k=0;
            vector<Point2f> points1,points2,points3;//跟踪上的第一帧与第i帧的特征点

            for(auto iter2=cur_points.begin(),iter1=first_points_f.begin();iter2!=cur_points.end();k++)
            {
                if(status[k]!=0)
                {
                    points1.push_back(*iter1);
                    points2.push_back(*iter2);
                    iter1++;
                    iter2++;
                }
                else
                {
                    iter1++;
                    iter2++;
                }
            }
            //用F矩阵RANSAC剔除误匹配点
            point_filter(points1,points2);
            //points3用于划线显示，无实际作用

            //计算平均运动像素，当足够大的运动像素的时候返回

            for(auto iter2=points2.begin(),iter1=points1.begin();iter2!=points2.end();)
            {
                aver_x=aver_x+fabs(iter2->x-iter1->x);
                aver_y=aver_y+fabs(iter2->y-iter1->y);

                iter1++;
                iter2++;
            }
            aver_x=aver_x/(float)points1.size();
            aver_y=aver_y/(float)points1.size();
            dist=sqrt(aver_x*aver_x+aver_y*aver_y);

            cout<<"aver_x:"<<aver_x<<endl;
            cout<<"aver_y:"<<aver_y<<endl;
            cout<<"aver_k:"<<aver_k<<endl;
            cout<<"dist:"<<dist<<endl;
            //根据斜率去除误匹配点

            initFeatures_f[init_frame_count].assign(points2.begin(),points2.end());
            //设置 mask
            for (auto it : points2)
            {
                if (mask.at<uchar>(it) == 255)
                {
                    cv::circle(mask, it, 3, 0, -1);
                }
            }
            //todo 记录该特征点的追踪次数(初始化的时候不用，但是跟踪的过程中就需要了)
            Mat img_show2;
            img_show2=showMatch(init_first_img_f,initImg_f[init_frame_count],points1,points2);
            imshow("imshow2",img_show2);
        }


        vector<Point2f> feature_new;//需要补充的点

        if(init_frame_count==0)
        {
            goodFeaturesToTrack(cur_img_f,feature_new,500,0.05,10,mask);//todo 参数定义特征数量和最小距离
        }
        else
        {
            goodFeaturesToTrack(cur_img_f,feature_new,500-initFeatures_f[init_frame_count].size(),0.05,10,mask);//todo 参数定义特征数量和最小距离
        }

        cout<<"feature_new:"<<feature_new.size()<<endl;

        if(init_frame_count==0)
        {
            for(auto f:feature_new)
            {
                first_points_f.push_back(f);
            }
        }

            for(auto f:feature_new) //把新剔除的点添加到该帧对应的特征点中
            {
                initFeatures_f[init_frame_count].push_back(f);
                track_times[init_frame_count].push_back(1);  //将新加入点的跟踪次数设置为1

            }

        for(auto pt:initFeatures_f[init_frame_count])
        {
            circle(imshow1,pt,2,Scalar(0),2);
        }
        cout<<"第"<<init_frame_count<<"帧特征点数量："<<initFeatures_f[init_frame_count].size()<<endl;
        imshow("inshow1",imshow1);
        imshow("mask1",mask);
        waitKey(100);
        init_frame_count++;
        return false;
    }
    else
    {
        init_frame_flag=init_frame_count-1;
        return true;
    }
}

bool Feature_tracking::initialization()
{
    cout<<"Begin initialization..."<<endl;
    cout<<"first:"<<first_points_f.size()<<endl;
    cout<<"l:"<<initFeatures_f[init_frame_flag].size()<<endl;

    recoverRT();//通过第1帧和第l帧恢复相机的位姿，relative_R,relative_t
    //todo 设置第一帧和第l帧的位姿---SE3

    //todo 第一帧和第l帧三角化，恢复特征3D坐标（注意像素、相机、世界坐标系之间的转换），并去除畸变

    //todo 通过光流（之前的数据已经保存），pnp，三角化，恢复初始化这几帧 相机的RT，用两帧之间的优化，和局部BA优化

    cout<<"R:"<<relative_R<<endl;
    cout<<"t:"<<relative_t<<endl;
    waitKey(0);


//    R<<R_estimate.at<double>(0,0),R_estimate.at<double>(0,1),R_estimate.at<double>(0,2),
//            R_estimate.at<double>(1,0),R_estimate.at<double>(1,1),R_estimate.at<double>(1,2),
//            R_estimate.at<double>(2,0),R_estimate.at<double>(2,1),R_estimate.at<double>(2,2);
//    t<<t_estimate.at<double>(0,0),t_estimate.at<double>(1,0),t_estimate.at<double>(2,0);
//
//    SE3 pose_new(R,t);
//
//    Mat initial_pose=(Mat_<double>(3,4) <<
//                                        1,0,0,0,
//            0,1,0,0,
//            0,0,1,0);
//    Mat first_pose=(Mat_<double>(3,4)<<
//                                     R_estimate.at<double>(0,0),R_estimate.at<double>(0,1),R_estimate.at<double>(0,2),t_estimate.at<double>(0,0),
//            R_estimate.at<double>(1,0),R_estimate.at<double>(1,1),R_estimate.at<double>(1,2),t_estimate.at<double>(1,0),
//            R_estimate.at<double>(2,0),R_estimate.at<double>(2,1),R_estimate.at<double>(2,2),t_estimate.at<double>(2,0));
//
//    Mat points_position;
//
//    for(int j=0;j<pre_points.size();j++)
//    {
//        pre_points_cam.push_back(Camera::uv2camera(pre_points[j],camera->getK()));
//        cur_points_cam.push_back(Camera::uv2camera(cur_points[j],camera->getK()));
//    }
//
//    triangulatePoints(initial_pose,first_pose,pre_points_cam,cur_points_cam,points_position);
//    points_position_norm=normalization(points_position);//归一化坐标
//
//
//    pre_keypoints.clear();
//    pre_keypoints.assign(cur_keypoints.begin(),cur_keypoints.end());
//    pre_img=cur_img.clone();
//    pre_descriptor=cur_descriptor.clone();

}

void Feature_tracking::clearstate()
{
    R_estimate=(Mat_<double >(3,3)<<1.0,0,0,0,1.0,0,0,0,1.0);
    t_estimate=(Mat_<double >(3,1)<<0,0,0);
    good_matches.clear();
    matches.clear();
//    cur_keypoints.clear();
}

void Feature_tracking::initReset()
{
    cout<<"Initialization finshed, clear the state."<<endl;
}

void Feature_tracking::recoverRT()
{
    vector<uchar > status_l,status_k;
    vector<Point2f> points_l;
    vector<float> error_l;
    vector<KeyPoint> keypoints_l;
    Mat descriptor_first,desrciptor_l;

    cv::Ptr<ORB> orb_detect=cv::ORB::create(500,1.2f,8,31,0,2,ORB::HARRIS_SCORE,31,20);
    BFMatcher matcher(NORM_HAMMING);


    orb_detect->detect(init_first_img_f,first_keypoints_f);
    orb_detect->compute(init_first_img_f,first_keypoints_f,descriptor_first);
    orb_detect->detect(initImg_f[init_frame_flag],keypoints_l);
    orb_detect->compute(initImg_f[init_frame_flag],keypoints_l,desrciptor_l);


    matcher.match(descriptor_first,desrciptor_l,matches);//会将所有的特征进行匹配，所以总数是500

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
        orb_points_first.push_back(first_keypoints_f[m.queryIdx].pt);
        orb_points_l.push_back(keypoints_l[m.trainIdx].pt);
    }

//    calcOpticalFlowPyrLK(init_first_img_f,initImg_f[init_frame_flag],first_points_f,points_l,status_l,error_l,Size(21, 21), 3);
    point_filter(orb_points_first,orb_points_l);
    Mat img_show3;
    img_show3=showMatch(init_first_img_f,initImg_f[init_frame_flag],orb_points_first,orb_points_l);
    imshow("imshow3",img_show3);

    F_f=findFundamentalMat(orb_points_first,orb_points_l,FM_RANSAC,1.0,0.99,status_k);
    cout<<"F_f:"<<F_f<<endl;
    E_f=findEssentialMat(orb_points_first,orb_points_l,camera_k);

    recoverPose(E_f,orb_points_first,orb_points_l,camera_k,relative_R,relative_t);
    cv2eigen(relative_R,eigen_R);
    cv2eigen(relative_t,eigen_t);
//    Eigen::Quaterniond q=Eigen::Quaterniond(eigen_R);
//    cout<<"q:"<<q.coeffs()<<endl;
}