//
// Created by jh on 18-3-8.
//

#include "Feature_tracking.h"
//#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/opencv.hpp>


vector<Point3d> normalization(const Mat& points_position_)
{
    vector<Point3d> points_position_norm_;
    for(int j=0;j<points_position_.cols;j++)
    {
//        Point3d point_3d_norm_;
//        point_3d_norm_.x=points_position_.col(j).at<double>(0,0)/points_position_.col(j).at<double>(3,0);
//        point_3d_norm_.y=points_position_.col(j).at<double>(1,0)/points_position_.col(j).at<double>(3,0);
//        point_3d_norm_.z=points_position_.col(j).at<double>(2,0)/points_position_.col(j).at<double>(3,0);
        Mat x;
        x=points_position_.col(j);
        x /= x.at<float>(3,0);
        Point3d point_3d_norm_(x.at<float >(0,0),x.at<float >(1,0),x.at<float >(2,0));
        points_position_norm_.push_back(point_3d_norm_);
    }
    return points_position_norm_;
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
vector<Point2f> Feature_tracking::getFeaturepointsIn(uint64_t &frame_id)
{
    vector<Point2f> points;
    for(auto point:features_f)
    {
        for(int it=0;it<point->point_pre_frame.size();it++)
        {
            if(point->point_pre_frame[it].first==frame_id)
            {
                points.push_back(point->point_pre_frame[it].second);
            }

        }
    }
    return points;
}
vector<uint64_t > Feature_tracking::getFeatureIdIn(uint64_t &frame_id)
{
    vector<uint64_t > ids;
    for(auto point:features_f)
    {
        for(int it=0;it<point->point_pre_frame.size();it++)
        {
            if(point->point_pre_frame[it].first==frame_id)
            {
                ids.push_back(point->id_);
            }
        }
    }
    return ids;
}

void Feature_tracking::reduceVector(vector<uint64_t > &ids,vector<Point2f> &points1, vector<Point2f> &points2, vector<uchar> &status)
{
    int k=0;
    auto iter_id=ids.begin();
    for(auto iter2=points2.begin(),iter1=points1.begin();iter2!=points2.end();k++)
    {
        if(status[k]==0)
        {
            points1.erase(iter1);
            points2.erase(iter2);
            ids.erase(iter_id);
        }
        else
        {
            iter1++;
            iter2++;
            iter_id++;
        }
    }
    points1.resize(points2.size());
    ids.resize(points2.size());
}
void Feature_tracking::reduceVector(vector<Point3d> &points1, vector<uchar> &status)
{
    int k=0;

    for(auto iter1=points1.begin();iter1!=points1.end();k++)
    {
        if(status[k]==0)
        {
            points1.erase(iter1);

        }
        else
        {

            iter1++;
        }
    }
}
void Feature_tracking::reduceVector(vector<Point2f> &points1, vector<Point2f> &points2, vector<uchar> &status)
{
    int k=0;
    for(auto iter2=points2.begin(),iter1=points1.begin();iter2!=points2.end();k++)
    {
        if(status[k]==0)
        {
            points1.erase(iter1);
            points2.erase(iter2);
        }
        else
        {
            iter1++;
            iter2++;
        }
    }
    points1.resize(points2.size());
}


void Feature_tracking::addFeature(Point2f &point, uint64_t& frame_id)
{
    Feature::Ptr feature=Feature::creat(point);
    feature->addTrack(frame_id,point);
    feature->point_pre_camera.push_back(make_pair(frame_id,Camera::uv2camera(point,camera_k)));


    features_f.push_back(feature);
}

void Feature_tracking::solveCamPoseByPnP(int first,int second)//todo 用到了三角化的函数
{
    vector<Point2f> pre_points_f_copy,cur_points_f_copy;
    vector<Point2f> pre_points_f_copy_3d,cur_points_f_copy_3d;
    vector<Point3d> pre_points_norm_copy;
    vector<Point3f> pre_points_norm_copy_f;
    vector<uint64_t > pre_points_id_copy;
    vector<uint64_t > pre_points_id_copy_3d;
    pre_points_f_copy.clear();
    cur_points_f_copy.clear();
    pre_points_norm_copy.clear();
    pre_points_id_copy.clear();
    for(auto point:features_f)
    {
        for(int it=0;it<point->point_pre_frame.size();it++)
        {
            if(point->point_pre_frame[it].first==first)
            {
                pre_points_f_copy.push_back(point->point_pre_frame[it].second);
                pre_points_id_copy.push_back(point->id_);
            }
        }

    }

    Mat D,rvec_r;
    Mat rvec,tvec;
    Matrix3d rvec_eigen;
    Vector3d tvec_eigen;
    Mat inliers;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
    vector<uchar > status_solve;
    vector<float > errors_solve;
    rvec_eigen=init_Qs[second-1].toRotationMatrix();
    tvec_eigen=init_ts[second-1]; //为了对称，其实没必要
    eigen2cv(rvec_eigen,rvec);
    eigen2cv(tvec_eigen,tvec);
    Rodrigues(rvec,rvec_r);

    calcOpticalFlowPyrLK(initImg_f[first],initImg_f[second],pre_points_f_copy,cur_points_f_copy,status_solve,errors_solve,Size(21, 21), 3);
    reduceVector(pre_points_id_copy,pre_points_f_copy,cur_points_f_copy,status_solve);
//    reduceVector(pre_points_norm_copy,status_solve);
    //剔除空的向量
    point_filter(pre_points_id_copy,pre_points_f_copy,cur_points_f_copy);

    Feature::addTrack(features_f,pre_points_id_copy,second,cur_points_f_copy,camera_k);

    int num=0;
    for(auto id:pre_points_id_copy)
    {
        if(features_f[id]->if3D)
        {
            pre_points_f_copy_3d.push_back(features_f[id]->point_pre_frame[first].second);
            pre_points_norm_copy.push_back(features_f[id]->pose_world_);
            pre_points_id_copy_3d.push_back(id);
            cur_points_f_copy_3d.push_back(cur_points_f_copy[num]);
        }
        num++;
    }

//    for(auto po:pre_points_norm_copy)
//        pre_points_norm_copy_f.push_back(static_cast<Point3f>(po));

    solvePnPRansac(pre_points_norm_copy,cur_points_f_copy_3d,camera_k,D,rvec_r,tvec, true,100,5.0,0.99,inliers);

    cout<<"solvePnPRansac内点个数:"<<inliers.rows<<endl;

    Rodrigues(rvec_r,rvec);
    cv2eigen(rvec,rvec_eigen);
    cv2eigen(tvec,tvec_eigen);
    init_Qs[second]=Eigen::Quaterniond(rvec_eigen);
    init_ts[second]=tvec_eigen;

//    Mat img_show6;
//    img_show6=showMatch(initImg_f[first],initImg_f[second],pre_points_f_copy,cur_points_f_copy);
//    imshow("solvePNP",img_show6);
    cout<<"solved:"<<second<<"th camera pose"<<endl;
    cout<<"R"<<rvec<<endl;
    cout<<"t"<<tvec<<endl;
    solvePointsByTri(first,second,pre_points_id_copy,pre_points_f_copy,cur_points_f_copy);

}

void Feature_tracking::solvePointsByTri(int first, int second,vector<uint64_t >& pre_points_id_copy, vector<Point2f>& pre_points_f_copy,vector<Point2f>& cur_points_f_copy) //
{
    cout<<"Triangulation between-------------------------------------- "<<first<<"th and "<<second<<"th camera-----------------------------"<<endl;
    vector<Point3d> points_position_norm;
    vector<Point2f> pre_points_camera_copy,cur_points_camera_copy;
    vector<uint64_t > points_ids;
    ///仅三角化给定id的特征点
//    for(auto id:pre_points_id_copy)
//    {
//        uint64_t first_u=first;
//        uint64_t second_u=second;
//        pre_points_camera_copy.push_back(features_f[id]->pointIncamera(first_u));
//        cur_points_camera_copy.push_back(features_f[id]->pointIncamera(second_u));
//    }
    ///三角化两帧之中所有的特征点
    for(auto f:features_f)
    {
        uint64_t first_u=first;
        uint64_t second_u=second;
        if(f->TrackBy(first_u)&&f->TrackBy(second_u))
        {
            points_ids.push_back(f->id_);
            pre_points_camera_copy.push_back(f->pointIncamera(first_u));
            cur_points_camera_copy.push_back(f->pointIncamera(second_u));
        }

    }

    Mat points4D;
    Mat firstR,secondR,firstT,secondT;
    Matrix3d firstR_eigen,secondR_eigen;
    Vector3d firstT_eigen,secondT_eigen;
    firstR_eigen=init_Qs[first].toRotationMatrix();
    secondR_eigen=init_Qs[second].toRotationMatrix();
    firstT_eigen=init_ts[first];
    secondT_eigen=init_ts[second];
    eigen2cv(firstR_eigen,firstR);
    eigen2cv(secondR_eigen,secondR);
    eigen2cv(firstT_eigen,firstT);
    eigen2cv(secondT_eigen,secondT);

    Mat first_pose;
    first_pose=(Mat_<double>(3,4) <<
            firstR.at<double>(0,0),firstR.at<double>(0,1),firstR.at<double>(0,2),firstT.at<double>(0,0),
            firstR.at<double>(1,0),firstR.at<double>(1,1),firstR.at<double>(1,2),firstT.at<double>(1,0),
            firstR.at<double>(2,0),firstR.at<double>(2,1),firstR.at<double>(2,2),firstT.at<double>(2,0));
    Mat l_pose;
    l_pose = (Mat_<double>(3, 4) <<
            secondR.at<double>(0,0),secondR.at<double>(0,1),secondR.at<double>(0,2),secondT.at<double>(0,0),
            secondR.at<double>(1,0),secondR.at<double>(1,1),secondR.at<double>(1,2),secondT.at<double>(1,0),
            secondR.at<double>(2,0),secondR.at<double>(2,1),secondR.at<double>(2,2),secondT.at<double>(2,0));

    triangulatePoints(first_pose,l_pose,pre_points_camera_copy,cur_points_camera_copy,points4D);

    points_position_norm=normalization(points4D);//归一化,转换成非齐次坐标
    int k=0;
    ///三角化两帧之中所有的特征点
    for(auto po:points_position_norm)
    {
        if(po.z<0&&features_f[points_ids[k]]->quality_feature==Feature::bad)
        {
            features_f[points_ids[k]]->destoryFeature();
        }
        else if(po.z<0)
        {
            features_f[points_ids[k]]->quality_feature=Feature::bad;
        }
        else if(features_f[points_ids[k]]->if3D)
        {
            if(features_f[points_ids[k]]->track_times_>3)
                features_f[points_ids[k]]->quality_feature=Feature::good;
            features_f[points_ids[k]]->pose_world_=(po+features_f[points_ids[k]]->pose_world_)/2;
        }
        else
        {
            features_f[points_ids[k]]->pose_world_=po;
            features_f[points_ids[k]]->if3D=true;
        }
        k++;
    }
    ///三角化两帧之中特定的特征点
//    for(auto po:points_position_norm)
//    {
//        if(po.z<0&&features_f[pre_points_id_copy[k]]->quality_feature==Feature::bad)
//        {
//            features_f[pre_points_id_copy[k]]->destoryFeature();
//        }
//        else if(po.z<0)
//        {
//            features_f[pre_points_id_copy[k]]->quality_feature=Feature::bad;
//        }
//        else if(features_f[pre_points_id_copy[k]]->if3D)
//        {
//            if(features_f[pre_points_id_copy[k]]->track_times_>3)
//                features_f[pre_points_id_copy[k]]->quality_feature=Feature::good;
//            features_f[pre_points_id_copy[k]]->pose_world_=(po+features_f[pre_points_id_copy[k]]->pose_world_)/2;
//        }
//        else
//        {
//            features_f[pre_points_id_copy[k]]->pose_world_=po;
//            features_f[pre_points_id_copy[k]]->if3D=true;
//        }
//        k++;
//    }
    cout<<"Triangulation between---------------------------- "<<first<<"th and "<<second<<"th camera------------------finished!"<<endl;
}

//利用F矩阵剔除误匹配
void Feature_tracking::point_filter(vector<Point2f> &points1, vector<Point2f> &points2)     //todo 投影到归一化平面中进行畸变校正，然后再进行F矩阵筛选
{
    vector<uchar> status_F;
    Mat F;
    //根据F矩阵筛选
    F=findFundamentalMat(points1,points2,FM_RANSAC,1.0,0.99,status_F);
    reduceVector(points1,points2,status_F);

//    cout<<"F:"<<F<<endl;
    //根据斜率筛选
    for(auto iter2=points2.begin(),iter1=points1.begin();iter2!=points2.end();)
    {
        aver_k=aver_k+fabs((iter2->y-iter1->y)/(iter2->x+(float)640.0-iter1->x));
        aver_k_raw=aver_k_raw+(iter2->y-iter1->y)/(iter2->x+(float)640.0-iter1->x);
        iter1++;
        iter2++;
    }
    aver_k=aver_k/(float)points1.size();
    aver_k_raw=aver_k_raw/(float)points1.size();
    //todo 再根据斜率进行筛选

    for(auto iter3=points2.begin(),iter4=points1.begin();iter3!=points2.end();)
    {
        float k=(iter3->y-iter4->y)/(iter3->x+640-iter4->x);
        if(fabs(k)>1.5*aver_k||(k*aver_k_raw<0&&fabs(k)>0.5*aver_k) )  ///筛选特征点的位置
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
    cout<<"斜率筛选后:"<<points1.size()<<"   "<<points2.size()<<endl;
    points1.resize(points2.size());
}
void Feature_tracking::point_filter(vector<uint64_t > &ids,vector<Point2f> &points1, vector<Point2f> &points2)     //todo 投影到归一化平面中进行畸变校正，然后再进行F矩阵筛选
{
    vector<uchar> status_F;
    Mat F;
    //根据F矩阵筛选
    F=findFundamentalMat(points1,points2,FM_RANSAC,3.0,0.99,status_F);
    reduceVector(ids,points1,points2,status_F);

//    cout<<"F:"<<F<<endl;
    cout<<"F筛选后:"<<points1.size()<<"   "<<points2.size()<<endl;
    //根据斜率筛选
    for(auto iter2=points2.begin(),iter1=points1.begin();iter2!=points2.end();)
    {
        aver_k=aver_k+fabs((iter2->y-iter1->y)/(iter2->x+(float)640.0-iter1->x));
        aver_k_raw=aver_k_raw+(iter2->y-iter1->y)/(iter2->x+(float)640.0-iter1->x);
        iter1++;
        iter2++;
    }
    aver_k=aver_k/(float)points1.size();
    aver_k_raw=aver_k_raw/(float)points1.size();
    //todo 再根据斜率进行筛选
    auto iter_id=ids.begin();
    for(auto iter3=points2.begin(),iter4=points1.begin();iter3!=points2.end();)
    {
        float k=(iter3->y-iter4->y)/(iter3->x+640-iter4->x);
        if(fabs(k)>1.5*aver_k||(k*aver_k_raw<0&&fabs(k)>0.5*aver_k) )  ///筛选特征点的位置
        {
            points1.erase(iter4);
            points2.erase(iter3);
            ids.erase(iter_id);
        }
        else
        {
            iter_id++;
            iter4++;
            iter3++;
        }
    }
    cout<<"k筛选后:"<<points1.size()<<"   "<<points2.size()<<endl;
    points1.resize(points2.size());
    ids.resize(points2.size());
}

void Feature_tracking::point_filter(vector<uint64_t >& ids,vector<Point2f> &points1, vector<Point2f> &points2,
                                    vector<Point3d> &points_position_norm)
{
    vector<uchar> status_F;
    Mat F;
    //根据F矩阵筛选
    F=findFundamentalMat(points1,points2,FM_RANSAC,1.0,0.99,status_F);
    int k=0;
    auto iter_id=ids.begin();
    auto iter5=points_position_norm.begin();
    for(auto iter2=points2.begin(),iter1=points1.begin();iter2!=points2.end();k++)
    {
        if(status_F[k]==0)
        {
            points1.erase(iter1);
            points2.erase(iter2);
            points_position_norm.erase(iter5);
            ids.erase(iter_id);
        }
        else
        {
            iter1++;
            iter2++;
            iter5++;
            iter_id++;
        }
    }
    //根据斜率筛选
    for(auto iter2=points2.begin(),iter1=points1.begin();iter2!=points2.end();)
    {
        aver_k=aver_k+fabs((iter2->y-iter1->y)/(iter2->x+(float)640.0-iter1->x));
        aver_dist+=sqrt((iter2->y-iter1->y)*(iter2->y-iter1->y)+(iter2->x-iter1->x)*(iter2->x-iter1->x));
        iter1++;
        iter2++;
    }
    aver_k=aver_k/(float)points1.size();
    aver_dist=aver_dist/(float)points1.size();
    auto iter_id2=ids.begin();
    auto iter6=points_position_norm.begin();
    for(auto iter3=points2.begin(),iter4=points1.begin();iter3!=points2.end();)
    {
        if(fabs((iter3->y-iter4->y)/(iter3->x+640-iter4->x))>1.5*aver_k)   ///筛选特征点的位置
        {
            points1.erase(iter4);
            points2.erase(iter3);
            points_position_norm.erase(iter6);
            ids.erase(iter_id2);
        }
        else if(fabs(sqrt((iter3->y-iter4->y)*(iter3->y-iter4->y)+(iter3->x-iter4->x)*(iter3->x-iter4->x))-aver_dist)>0.3*aver_dist)
        {
            points1.erase(iter4);
            points2.erase(iter3);
            points_position_norm.erase(iter6);
            ids.erase(iter_id2);
        }
        else
        {
            iter4++;
            iter3++;
            iter6++;
            iter_id2++;
        }
    }
}

bool cmp(DMatch m1,DMatch m2)
{
    return m1.distance<m2.distance;
}

Feature_tracking::Feature_tracking(int width, int height)
        :width_(width),height_(height),init_frame_count(0),aver_x(0),aver_y(0),aver_k(0),aver_k_raw(0),min_init_dist(init_dist)
{

}

bool Feature_tracking::loadInitImage(const Mat& image,Frame::Ptr& frame)
{
    cout<<"loading..."<<init_frame_count<<"th image"<<endl;
    cur_frame_f=frame;

    //初始化变量
    vector<Point2f> empty_points;
    vector<uchar> empty_status;
    initFeatures_f.push_back(empty_points);//必须先插进去一个空的，否则无法按照编号插入
//    raw_Features_f.push_back(empty_points);
    raw_status_f.push_back(empty_status);

    vector<Point2f> pre_points,cur_points;
    vector<uint64_t > pre_points_ids;
    vector<uchar> status;
    vector<float> error;

    if (dist<min_init_dist)//数据太少，不能初始化
    {
        aver_x=0;
        aver_y=0;
        initImg_f.push_back(image);
        cur_img_f=image.clone();
        Mat imshow1=cur_img_f.clone();

        mask = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(255));

        if(init_frame_count!=0)
        {
            //设置上一帧的状态
            pre_img_f=initImg_f[init_frame_count-1].clone();
//            uint64_t frame_id=(uint64_t)(init_frame_count-1);
//            pre_points=getFeaturepointsIn(frame_id);
//            pre_points_ids=getFeatureIdIn(frame_id);

            uint64_t init=0;
            track_new(init,cur_frame_f->id_);//光流跟踪，第init帧和第frame->id_帧
        }
        if(init_frame_count==0)
        {
            vector<Point2f> feature_new;//需要补充的点
            goodFeaturesToTrack(cur_img_f,feature_new,500,0.1,20,mask);//todo 参数定义特征数量和最小距离

            cout<<"第0帧检测到特征点数量:"<<feature_new.size()<<endl;

            for(auto f:feature_new) //把新剔除的点添加到该帧对应的特征点中
            {
                Point2f f_nodistort;
                f_nodistort=Camera::removeDistort(f,camera_k1,camera_k2,camera_k3,camera_p1,camera_p2,camera_k);
                if((f_nodistort.x<5)||(f_nodistort.y<5)||(f_nodistort.x>475)||(f_nodistort.y>635))
                {
                    continue;
                }
                initFeatures_f[init_frame_count].push_back(f_nodistort);
                uint64_t frame_id=(uint64_t)init_frame_count;
                if(init_frame_count==0)
                {
                    addFeature(f_nodistort,frame_id);//为新检测到的特征点创建feature
                }
            }
        }

/* 每帧特征点画图
        for(auto pt:initFeatures_f[init_frame_count])
        {
            circle(imshow1,pt,2,Scalar(0),2);
        }
        cout<<"第"<<init_frame_count<<"帧特征点数量："<<initFeatures_f[init_frame_count].size()<<endl;
        imshow("inshow1",imshow1);
        imshow("mask1",mask);
        waitKey(0);
*/
        pre_frame_f=cur_frame_f;
        init_frame_count++;
        return false;
    }
    else
    {
        init_frame_flag=init_frame_count-1;
        init_l=init_frame_flag;
        initFeatures_f.resize(init_frame_flag+1);
        raw_status_f.resize(init_frame_flag+1);
// 打印初始化时所有的特征点
//        cout<<"所有的特征点："<<endl;
//        for(auto f:features_f)
//        {
//            for(uint64_t p=0;p<f->point_pre_frame.size();p++)
//            {
//                cout<<"feature "<<f->id_<<" in Frame "<<f->point_pre_frame[p].first<<" is "<<f->point_pre_frame[p].second<<" and camera frame "<<f->point_pre_camera[p].first<<" is "<<f->point_pre_camera[p].second<<endl;
//            }
//        }
//        waitKey(0);

        return true;
    }
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

bool Feature_tracking::recoverRT()   //todo 除了设置相机踪的移动的距离，再设定在x y轴上的移动距离
{

    vector<float> error_l;
    vector<uchar> status_l;
    vector<Point2f> points_first,points_l;
    points_first.assign(initFeatures_f[0].begin(),initFeatures_f[0].end());
    calcOpticalFlowPyrLK(initImg_f[0],initImg_f[init_l],points_first,points_l,status_l,error_l);

    reduceVector(points_first,points_l,status_l);
    point_filter(points_first,points_l);                            //todo 再用光流跟一下，补充一次特征点，要不然特征点太少了初始化的时候会失败

    if(points_first.size()<15)
    {
        init_l--;
        cout<<init_l<<endl;
        return false;
    }

//    Mat img_show3;
//    img_show3=showMatch(initImg_f[0],initImg_f[init_l],points_first,points_l);
//    imshow("solveRT",img_show3);

    E_f=findEssentialMat(points_first,points_l,camera_k);

    recoverPose(E_f,points_first,points_l,camera_k,relative_R,relative_t);
    cv2eigen(relative_R,eigen_R);
    cv2eigen(relative_t,eigen_t);
//    Eigen::Quaterniond q=Eigen::Quaterniond(eigen_R);
//    cout<<"q:"<<q.coeffs()<<endl;
    return true;
}
void Feature_tracking::track_new(uint64_t& pre_frame_id,uint64_t& cur_frame_id)
{
    vector<Point2f> pre_points,cur_points;
    vector<uint64_t > pre_points_ids;
    vector<uchar > status;
    vector<float> errors;

    pre_points=getFeaturepointsIn(pre_frame_id);
    pre_points_ids=getFeatureIdIn(pre_frame_id);

    cout<<pre_points.size()<<endl;
    calcOpticalFlowPyrLK(initImg_f[pre_frame_id],cur_img_f,pre_points,cur_points,status,errors,Size(21, 21), 3);


    reduceVector(pre_points_ids,pre_points,cur_points,status);
    //用F矩阵RANSAC剔除误匹配点

    point_filter(pre_points_ids,pre_points,cur_points);

    Feature::addTrack(features_f,pre_points_ids,cur_frame_id,cur_points,camera_k);


    for(auto iter2=cur_points.begin(),iter1=pre_points.begin();iter2!=cur_points.end();)
    {
        aver_x=aver_x+fabs(iter2->x-iter1->x);
        aver_y=aver_y+fabs(iter2->y-iter1->y);
        iter1++;
        iter2++;
    }
    aver_x=aver_x/(float)pre_points.size();
    aver_y=aver_y/(float)pre_points.size();
    dist=sqrt(aver_x*aver_x+aver_y*aver_y);

    cout<<"第"<<int(cur_frame_id)<<"平均运动距离dist（像素）:"<<dist<<endl;
    initFeatures_f[init_frame_count].assign(cur_points.begin(),cur_points.end());

    if(pre_points_ids.size()!=cur_points.size())
    {
        cout<<"error occurs,pre_points_ids.size()!=cur_points.size()"<<endl;
        waitKey(0);
    }
//    int k=0;
//    for(auto id:pre_points_ids)
//        features_f[id]->addTrack(init_frame_count,cur_points[k++]);

    Mat img_show2;
    img_show2=showMatch(initImg_f[pre_frame_id],initImg_f[init_frame_count],pre_points,cur_points);
    imshow("Tracking...",img_show2);
    waitKey(500);
}

void Feature_tracking::recoverStructure()
{
    vector<Point2f> pre_points_cam,cur_points_cam;
    init_Qs.resize(init_frame_flag+1);//设置大小
    init_ts.resize(init_frame_flag+1);

    //设置首帧和第l帧位姿（Q，t）
    init_Qs[0].setIdentity();
    init_ts[0].setZero();
    init_Qs[init_l]=Eigen::Quaterniond (eigen_R);
    init_ts[init_l]=Eigen::Vector3d (eigen_t);

    //转换成Mat形式便于三角化
    Mat first_pose;
    first_pose=(Mat_<double>(3,4) <<
            1,0,0,0,
            0,1,0,0,
            0,0,1,0);
    Mat l_pose;
    l_pose = (Mat_<double>(3, 4) <<
                relative_R.at<double>(0,0),relative_R.at<double>(0,1),relative_R.at<double>(0,2),relative_t.at<double>(0,0),
                relative_R.at<double>(1,0),relative_R.at<double>(1,1),relative_R.at<double>(1,2),relative_t.at<double>(1,0),
                relative_R.at<double>(2,0),relative_R.at<double>(2,1),relative_R.at<double>(2,2),relative_t.at<double>(2,0));
    cout<<"第l帧位姿，l_pose:"<<endl<<l_pose<<endl;

    //用最开始的光流信息进行跟踪，先进行拷贝
    vector<Point2f> pre_raw_Features_f,cur_raw_Features_f;
    vector<uint64_t > pre_raw_Features_id;
    //todo 代替raw_Features_f

    for(auto point:features_f)
    {
        uint64_t init=0;
        uint64_t finish=(uint64_t)init_frame_flag;
        if(point->TrackBy(init)&&point->TrackBy(finish))
        {
            pre_raw_Features_f.push_back(point->pointIn(init));
            pre_raw_Features_id.push_back(point->id_);
            cur_raw_Features_f.push_back(point->pointIn(finish));
        }
    }

    cout<<"共同观测点："<<pre_raw_Features_id.size()<<endl;
    //剔除+筛选
    point_filter(pre_raw_Features_id,pre_raw_Features_f,cur_raw_Features_f);
//    Mat imshow5;
//    imshow5=showMatch(initImg_f[0],initImg_f[init_frame_flag],pre_raw_Features_f,cur_raw_Features_f);
//    imshow("光流三角化0-l",imshow5);

    //变换到相机坐标系，才能进行三角化
    vector<Point3d> points_position_norm;//三角化恢复出的3D特征点,第一帧的相机中
    Mat points4D;
    for(int j=0;j<pre_raw_Features_f.size();j++)
    {
        pre_points_cam.push_back(Camera::uv2camera(pre_raw_Features_f[j],camera_k));
        cur_points_cam.push_back(Camera::uv2camera(cur_raw_Features_f[j],camera_k));
    }
/*
    for(auto pc:pre_points_cam)
    {
        cout<<"camera"<<":"<<pc.x<<","<<pc.y<<","<<endl;
    }
    */
    triangulatePoints(first_pose,l_pose,pre_points_cam,cur_points_cam,points4D);
/*
    for(int n=0;n<points4D.cols;n++)
    {
        cout<<points4D.col(n)<<endl;
        cout<<endl;
    }
*/
    points_position_norm=normalization(points4D);//归一化,转换成非齐次坐标
    int k=0;
    for(auto po:points_position_norm)
    {
        features_f[pre_raw_Features_id[k]]->pose_world_=po;
        features_f[pre_raw_Features_id[k]]->if3D=true;
        k++;
    }
cout<<"................................................finish RT and tri between two..............................................."<<endl;
//    waitKey(0);

    //用第一帧与其他帧之间进行pnp求解相机位姿
cout<<".......................................................Begin pnp and tri..............................................."<<endl;
    for(int num=1;num<=init_l;num++)
    {
       int init =0;
       solveCamPoseByPnP(init,num);   //根据第0帧位姿计算第1帧到第l-1帧的位姿，并三角化点
//        waitKey(0);
    }

//    uint64_t frame=0;
//    while(1)
//    {
//        cout<<(int) frame<<endl;
//        Mat imgtest=initImg_f[frame].clone();
//        for(auto f:features_f)
//        {
//
//            if(f->TrackBy(frame))
//            {
//                circle(imgtest,f->pointIn(frame),2,0,2);
//            }
//        }
//        imshow("test",imgtest);
//        waitKey(0);
//        frame++;
//    }

    int count=0;
    for(auto p:features_f)
    {
        if(p->if3D)
            count++;
    }
    cout<<"所有的特征点："<<endl;
    for(auto f:features_f)
    {
//        cout<<"feature "<<f->id_<<" times: "<<f->track_times_<<" in Frame "<<f->point_pre_frame.size()<<" in camera "<<f->point_pre_camera.size()<<endl;
        for(uint64_t p=0;p<f->point_pre_frame.size();p++)
        {

//            cout<<"feature "<<f->id_<<" is "<<f->quality_feature<<" in Frame "<<f->point_pre_frame[p].first<<" is "<<f->point_pre_frame[p].second<<" and camera frame "<<f->point_pre_camera[p].first<<" is "<<f->point_pre_camera[p].second<<" and his pose is "<<f->pose_world_<<endl;
        }
    }
    cout<<"具有深度的特征点数量："<<count<<endl;
//    waitKey(0);
  ///显示特征点
    //todo 再利用光流跟踪恢复更多特征点的深度，注意当深度恢复不一致时，要筛选
//    for(int num=1;num<=init_l;num++)
//    {
//        int init =0;
//        solveCamPoseByPnP(init,num);   //根据第0帧位姿计算第1帧到第l-1帧的位姿，并三角化点
//        waitKey(0);
//    }
    //todo 融合特征点，全部恢复之后完成与边恢复位姿一样

    for(uint64_t first=1;first<=init_l-1;first++)
    {
        vector<Point2f > pre_points_new;
        vector<Point2f > pre_points_new_copy;
        mask = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(255));
        for (auto it : initFeatures_f[first])
        {
            if (mask.at<uchar>(it) == 255)
            {
                cv::circle(mask, it, 1, 0, -1);
            }
        }
        goodFeaturesToTrack(initImg_f[first],pre_points_new,500-initFeatures_f[first].size(),0.1,20,mask);//todo 参数定义特征数量和最小距离
        int feature_number=features_f.size();     //第一个新加入的特征的id

        vector<Point2f>::iterator iter;

        for(iter=pre_points_new.begin();iter!=pre_points_new.end();) //把新剔除的点添加到该帧对应的特征点中,使用引用，直接进行变换
        {
            Point2f f=*iter;
            f=Camera::removeDistort(f,camera_k1,camera_k2,camera_k3,camera_p1,camera_p2,camera_k);
            if((f.x<5)||(f.y<5)||(f.x>475)||(f.y>635))
            {
                pre_points_new.erase(iter);
                continue;
            }
            initFeatures_f[first].push_back(f);
            uint64_t frame_id=(uint64_t)first;
            addFeature(f,frame_id);//为新检测到的特征点创建feature
            iter++;
        }

        int feature_count=pre_points_new.size();  //新加入特征的个数
        vector<uint64_t > ids;
        for(uint64_t id=feature_number;id<(feature_number+feature_count);id++)
        {
            ids.push_back(id);
        }
        for(uint64_t second=first+1;second<=init_l;second++)
        {
            cout<<"second:"<<second<<endl;
            vector<uint64_t > ids_copy=ids;
            pre_points_new_copy=pre_points_new;
            vector<Point2f > cur_points_new;
            vector<uchar > status;
            vector<float> errors;

            calcOpticalFlowPyrLK(initImg_f[first],initImg_f[second],pre_points_new_copy,cur_points_new,status,errors);
            reduceVector(ids_copy,pre_points_new_copy,cur_points_new,status);
            point_filter(ids_copy,pre_points_new_copy,cur_points_new);

            int l=0;
            for(auto id:ids_copy)
            {
                Point2f& f=cur_points_new[l++];
                cout<<"pre"<<f<<endl;
                f=Camera::removeDistort(f,camera_k1,camera_k2,camera_k3,camera_p1,camera_p2,camera_k);
                cout<<"cur"<<f<<endl;
                features_f[id]->addTrack(second,f);//todo 什么问题啊我去
                features_f[id]->point_pre_camera.push_back(make_pair(second,Camera::uv2camera(f,camera_k)));
                initFeatures_f[second].push_back(f);
            }

            solvePointsByTri(first,second,ids_copy,pre_points_new_copy,cur_points_new);
            Mat img;
            img= showMatch(initImg_f[first],initImg_f[second],pre_points_new_copy,cur_points_new);
            imshow("new",img);
            waitKey(500);
        }
    }
    cout<<"now we have features number: "<<features_f.size()<<endl;
        for(auto f:features_f)
        {
            for(uint64_t p=0;p<f->point_pre_frame.size();p++)
            {
                cout<<"feature "<<f->id_<<" in Frame "<<f->point_pre_frame[p].first<<" is "<<f->point_pre_frame[p].second<<" and camera frame "<<f->point_pre_camera[p].first<<" is "<<f->point_pre_camera[p].second<<" pose is "<<f->pose_world_<<endl;
            }
        }
//        waitKey(0);
    for(int first=0;first<init_l;first++)
    {
        mask = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(255));
        for (auto it : initFeatures_f[first])
        {

            if (mask.at<uchar>(it) == 255)
            {
                cv::circle(mask, it, 2, 0, -1);
            }

        }
//        imshow("mask",mask);
//        waitKey(0);
    }

    // TODO 融合相近的特征点

    //Feature::fuseSameFeature(features_f,init_l);




    for(int cam=0;cam<init_frame_flag+1;cam++)
    {
        cout<<"camera"<<cam<<":"<<endl;
        cout<<init_Qs[cam].toRotationMatrix()<<endl;
        cout<<init_ts[cam]<<endl;
    }
//    waitKey(0);
}
void Feature_tracking::fuseFeatures(uint64_t id)
{

}
bool Feature_tracking::initialization()
{
    cout<<"Begin initialization..."<<endl;

    bool RT=false;
    while(!RT)
    {
        RT=recoverRT();//通过第1帧和第l帧恢复相机的位姿，relative_R,relative_t todo 可以添加参数，设置第一帧与第几帧进行初始化
    }
    cout<<"R:"<<relative_R<<endl;
    cout<<"t:"<<relative_t<<endl;
    recoverStructure();

    //todo 设置第一帧和第l帧的位姿---SE3

    //todo 第一帧和第l帧三角化，恢复特征3D坐标（注意像素、相机、世界坐标系之间的转换），并去除畸变

    //todo 通过光流（之前的数据已经保存），pnp，三角化，恢复初始化这几帧 相机的RT，用两帧之间的优化，和局部BA优化

    waitKey(0);
}





















