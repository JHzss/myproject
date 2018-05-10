//
// Created by jh on 18-3-8.
//

#include "Feature_tracking.h"
#include "parameters.h"
//#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/opencv.hpp>

Feature_tracking::Feature_tracking(int width, int height)
        :width_(width),height_(height),init_frame_count(-1),aver_k(0),aver_k_raw(0),min_init_dist(init_dist),dist(0.0)
{

}

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
        iter2->x+=image_width;
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
/**
 * @brief 通过点和id添加特征
 * @param point 图像坐标点，未经过畸变矫正
 * @param frame_id 图像id
 */

void Feature_tracking::addFeature(Point2f &point, uint64_t& frame_id)
{
    Point2f point_nodistort;
    //removeDistort:先投影到归一化平面，再进行畸变校正，再反投影回图像平面
    point_nodistort=Camera::removeDistort(point,camera_k1,camera_k2,camera_p1,camera_p2,camera_k);
    if((point_nodistort.x<5)||(point_nodistort.y<5)||(point_nodistort.x>(cur_img_f.cols-5))||(point_nodistort.y>(cur_img_f.rows-5)))
    {
        return;
    }
//    initFeatures_f[init_frame_count].push_back(f_nodistort); 暂时注释
    Feature::Ptr feature=Feature::creat();
    feature->addTrack(frame_id,point,point_nodistort);
    features_f.push_back(feature);
}

void Feature_tracking::solveCamPoseByPnP(int first,int second)//todo 用到了三角化的函数
{

    vector<Point2f> pre_points_f_copy_3d,cur_points_f_copy_3d;
    vector<Point3d> pre_points_pose;
    vector<Point3f> pre_points_norm_copy_f;
    vector<uint64_t > pre_points_id_copy;
    vector<uint64_t > pre_points_id_copy_3d;

    Mat D,rvec_rodrigues;
    Mat rvec,tvec;
    Matrix3d rvec_eigen;
    Vector3d tvec_eigen;
    Mat inliers;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);

    rvec_eigen=Q_s[second-1].toRotationMatrix();
    tvec_eigen=P_s[second-1]; //为了对称，其实没必要
    eigen2cv(rvec_eigen,rvec);
    eigen2cv(tvec_eigen,tvec);
    Rodrigues(rvec,rvec_rodrigues);

    uint64_t second_u=second;
    vector<Point2f> cur_points;
    for(auto feature:features_f)
    {
        if(feature->if3D&&feature->TrackBy(second_u))
        {
            pre_points_pose.push_back(feature->pose_world_);
            cur_points.push_back(feature->point_pre_frame_nodistort[second_u].second);
        }
    }
    cout<<"3D点个数:"<<pre_points_pose.size()<<endl;
    solvePnPRansac(pre_points_pose,cur_points,camera_k,D,rvec_rodrigues,tvec, true,100,5.0,0.99,inliers);

    cout<<"solvePnPRansac内点个数:"<<inliers.rows<<endl;

    Rodrigues(rvec_rodrigues,rvec);
    cv2eigen(rvec,rvec_eigen);
    cv2eigen(tvec,tvec_eigen);
    Q_s[second]=Eigen::Quaterniond(rvec_eigen);
    P_s[second]=tvec_eigen;

//    Mat img_show6;
//    img_show6=showMatch(allImg_f[first],allImg_f[second],pre_points_f_copy,cur_points_f_copy);
//    imshow("solvePNP",img_show6);
    cout<<"solved:"<<second<<"th camera pose"<<endl;
    cout<<"R"<<rvec<<endl;
    cout<<"t"<<tvec<<endl;
}

void Feature_tracking::solvePointsByTri(int first, int second,vector<uint64_t >& pre_points_id_copy, vector<Point2f>& pre_points_f_copy,vector<Point2f>& cur_points_f_copy) //
{
    cout<<"Triangulation between-------------------------------------- "<<first<<"th and "<<second<<"th camera-----------------------------"<<endl;
    vector<Point3d> points_position_norm;
    vector<Point2f> pre_points_camera_copy,cur_points_camera_copy;
    vector<uint64_t > points_ids;

    uint64_t first_u=first;
    uint64_t second_u=second;
    points_ids.assign(pre_points_id_copy.begin(),pre_points_id_copy.end());
    for(auto id:points_ids)
    {
        pre_points_camera_copy.push_back(features_f[id]->pointIncamera(first_u));
        cur_points_camera_copy.push_back(features_f[id]->pointIncamera(second_u));
    }
    Mat points4D;
    Mat firstR,secondR,firstT,secondT;
    Matrix3d firstR_eigen,secondR_eigen;
    Vector3d firstT_eigen,secondT_eigen;
    firstR_eigen=Q_s[first].toRotationMatrix();
    secondR_eigen=Q_s[second].toRotationMatrix();
    firstT_eigen=P_s[first];
    secondT_eigen=P_s[second];
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

//    ///三角化两帧之中所有的特征点
    for(auto po:points_position_norm)
    {
            features_f[points_ids[k]]->pose_world_=po;
            features_f[points_ids[k]]->if3D=true;
        k++;
    }
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
    /*
    for(auto iter2=points2.begin(),iter1=points1.begin();iter2!=points2.end();)
    {
        aver_k=aver_k+fabs((iter2->y-iter1->y)/(iter2->x+(float)image_width-iter1->x));
        aver_k_raw=aver_k_raw+(iter2->y-iter1->y)/(iter2->x+(float)image_width-iter1->x);
        iter1++;
        iter2++;
    }
    aver_k=aver_k/(float)points1.size();
    aver_k_raw=aver_k_raw/(float)points1.size();
    //todo 再根据斜率进行筛选

    for(auto iter3=points2.begin(),iter4=points1.begin();iter3!=points2.end();)
    {
        float k=(iter3->y-iter4->y)/(iter3->x+image_width-iter4->x);
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
     */
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
    /*
    //根据斜率筛选
    for(auto iter2=points2.begin(),iter1=points1.begin();iter2!=points2.end();)
    {
        aver_k=aver_k+fabs((iter2->y-iter1->y)/(iter2->x+(float)image_width-iter1->x));
        aver_k_raw=aver_k_raw+(iter2->y-iter1->y)/(iter2->x+(float)image_width-iter1->x);
        iter1++;
        iter2++;
    }
    aver_k=aver_k/(float)points1.size();
    aver_k_raw=aver_k_raw/(float)points1.size();
    //todo 再根据斜率进行筛选
    auto iter_id=ids.begin();
    for(auto iter3=points2.begin(),iter4=points1.begin();iter3!=points2.end();)
    {
        float k=(iter3->y-iter4->y)/(iter3->x+image_width-iter4->x);
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
     */
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
        aver_k=aver_k+fabs((iter2->y-iter1->y)/(iter2->x+(float)image_width-iter1->x));
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
        if(fabs((iter3->y-iter4->y)/(iter3->x+image_width-iter4->x))>1.5*aver_k)   ///筛选特征点的位置
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

void Feature_tracking::clearstate()
{
    R_estimate=(Mat_<double >(3,3)<<1.0,0,0,0,1.0,0,0,0,1.0);
    t_estimate=(Mat_<double >(3,1)<<0,0,0);

}

void Feature_tracking::initReset()
{
    cout<<"Initialization finshed, clear the state."<<endl;
}

bool Feature_tracking::recoverRT()   //todo 除了设置相机踪的移动的距离，再设定在x y轴上的移动距离
{

    vector<Point2f> points_first,points_l;
    vector<Point2f> points_first_nodistort,points_l_nodistort;
    uint64_t id_l=init_frame_count;
    for(auto feature:features_f)
    {
        if(feature->TrackBy(id_l))
        {
            points_first.push_back(feature->point_pre_frame[0].second);
            points_l.push_back(feature->point_pre_frame[id_l].second);
            points_first_nodistort.push_back(feature->point_pre_frame_nodistort[0].second);
            points_l_nodistort.push_back(feature->point_pre_frame_nodistort[id_l].second);
        }
    }
    point_filter(points_first_nodistort,points_l_nodistort);
    //todo 在初始化时特征点太少时需要进一步处理
//    if(points_first.size()<5)
//      {}

    Mat img_show3;
    img_show3=showMatch(allImg_f[0],allImg_f[id_l],points_first,points_l);
    imshow("solveRT",img_show3);

    ROS_INFO_STREAM("WE use "<<points_first.size()<<" points to calc relative RT");

    E_f=findEssentialMat(points_first_nodistort,points_l_nodistort,camera_k);//todo 计算矩阵应该用畸变前还是畸变后
    recoverPose(E_f,points_first_nodistort,points_l_nodistort,camera_k,relative_R,relative_t);

    cv2eigen(relative_R,eigen_R);
    cv2eigen(relative_t,eigen_t);
    Eigen::Quaterniond q=Eigen::Quaterniond(eigen_R);
    cout<<"q:"<<q.coeffs()<<endl;
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

    cout<<"pre image have "<<pre_points.size()<<" points"<<endl;
    calcOpticalFlowPyrLK(allImg_f[pre_frame_id],cur_img_f,pre_points,cur_points,status,errors,Size(21, 21), 3);

    reduceVector(pre_points_ids,pre_points,cur_points,status);
    //用F矩阵RANSAC剔除误匹配点
    point_filter(pre_points_ids,pre_points,cur_points);

    Feature::addTrack(features_f,pre_points_ids,cur_frame_id,cur_points,camera_k);

    // 计算视差
    float dx=0,dy=0;
    for(auto id:pre_points_ids)
    {
        dx+=fabs(features_f[id]->point_pre_frame_nodistort[0].second.x-features_f[id]->point_pre_frame_nodistort[cur_frame_id].second.x);
        dy+=fabs(features_f[id]->point_pre_frame_nodistort[0].second.y-features_f[id]->point_pre_frame_nodistort[cur_frame_id].second.y);
    }
    dx/=pre_points_ids.size();
    dy/=pre_points_ids.size();
    dist=sqrt(dx*dx+dy*dy);

    cout<<"第"<<int(cur_frame_id)<<"平均运动距离dist（像素）:"<<dist<<endl;
    Mat img_show2;
    img_show2=showMatch(allImg_f[pre_frame_id],allImg_f[init_frame_count],pre_points,cur_points);
    imshow("Tracking...",img_show2);
//    waitKey(0);
}
/**
 *  通过 1-2,2-3,3-4，……，init_l 光流跟踪恢复更多的特征点
 */
void Feature_tracking::recoverMoreFeatures()
{
    for(uint64_t first=1;first<init_l;first++)
    {
        cout<<"begin to extract "<<first<<"'s features "<<endl;
        vector<Point > pre_points_old;
        vector<Point2f > pre_points_new;
        mask = cv::Mat(image_height, image_width, CV_8UC1, cv::Scalar(255));
        for(auto point:features_f)
        {
            if(point->TrackBy(first))
                pre_points_old.push_back(point->pointIn_raw(first));
        }
        cout<<"have old features "<<pre_points_old.size()<<endl;
        for (auto it : pre_points_old)
        {
            if (mask.at<uchar>(it) == 255)
                circle(mask, it, 2, Scalar(0), -1);
        }
        goodFeaturesToTrack(allImg_f[first],pre_points_new,150-pre_points_old.size(),0.1,30,mask);//todo 参数定义特征数量和最小距离

        cout<<"have new features "<<pre_points_new.size()<<endl;
        for(auto feature_new:pre_points_new)
            addFeature(feature_new,first);

        vector<Point2f > points_in_first;
        vector<uint64_t > points_in_first_id;
        vector<Point2f > points_in_next;
        vector<uchar > status;
        vector<float> errors;
        uint64_t second=first+1;

        for(auto point:features_f)
        {
            for(int it=0;it<point->point_pre_frame.size();it++)
            {
                if((point->point_pre_frame[it].first==first)&&(!point->TrackBy(second)))
                {
                    points_in_first.push_back(point->point_pre_frame[it].second);
                    points_in_first_id.push_back(point->id_);
                }

            }
        }
        calcOpticalFlowPyrLK(allImg_f[first],allImg_f[second],points_in_first,points_in_next,status,errors);
        reduceVector(points_in_first_id,points_in_first,points_in_next,status);
        point_filter(points_in_first_id,points_in_first,points_in_next);
        int it=0;
        for(auto id:points_in_first_id)
        {
            Point2f point_nodistort;
            //removeDistort:先投影到归一化平面，再进行畸变校正，再反投影回图像平面
            point_nodistort = Camera::removeDistort(points_in_next[it], camera_k1, camera_k2, camera_p1, camera_p2,
                                                    camera_k);
            if ((point_nodistort.x < 5) || (point_nodistort.y < 5) || (point_nodistort.x > (cur_img_f.cols - 5)) ||
                (point_nodistort.y > (cur_img_f.rows - 5))) {
                continue;
            }
            features_f[id]->addTrack(second, points_in_next[it], point_nodistort);
            it++;
        }
        Mat img_show4;
        img_show4=showMatch(allImg_f[first],allImg_f[second],points_in_first,points_in_next);
        imshow("recoverMoreFeatures...",img_show4);
//        waitKey(0);
    }
}
/**
 * 将没有深度的特征点恢复出特征点
 */
void Feature_tracking::recover3dPoseOfFeature()
{
    for(int pre=1;pre<init_l;pre++)
    {
        vector<uint64_t >pre_points_id;
        vector<Point2f> pre_points,cur_points;
        uint64_t pre_u=pre;
        uint64_t cur_u=pre_u+1;
        for(auto feature:features_f)
        {
            if((feature->TrackBy(pre_u))&&(feature->TrackBy(cur_u))&&(!feature->if3D))
            {
                pre_points.push_back(feature->pointIn_nodistort(pre_u));
                pre_points_id.push_back(feature->id_);
                cur_points.push_back(feature->pointIn_nodistort(cur_u));
            }
        }
        cout<<"pre_points_id to solve 3D"<<pre_points_id.size()<<endl;
        solvePointsByTri(pre_u,cur_u,pre_points_id,pre_points,cur_points);
    }

}
bool Feature_tracking::optimize()
{
    double c_rotation[init_l+1][4];//存放c_Quat的w x y z
    double c_translation[init_l+1][3];//存放c_Translation的x y z
    ceres::Problem problem;
    ceres::LocalParameterization* quater= new ceres::QuaternionParameterization();

    for(int i=0;i<=init_l;i++)
    {
        c_rotation[i][0]=Q_s[i].w();
        c_rotation[i][1]=Q_s[i].x();
        c_rotation[i][2]=Q_s[i].y();
        c_rotation[i][3]=Q_s[i].z();
        c_translation[i][0]=P_s[i].x();
        c_translation[i][1]=P_s[i].y();
        c_translation[i][2]=P_s[i].z();
        problem.AddParameterBlock(c_rotation[i],4,quater);
        problem.AddParameterBlock(c_translation[i],3);
        if(i==init_l||i==0)
        {
            problem.SetParameterBlockConstant(c_rotation[i]);
            problem.SetParameterBlockConstant(c_translation[i]);
        }
    }
    double position[features_f.size()][3];
    for(int i=0;i<features_f.size();i++)
    {
        if(features_f[i]->if3D)
        {
            for(int j=0;j<features_f[i]->point_pre_camera.size();j++)
            {
                position[i][0]=features_f[i]->pose_world_.x;
                position[i][1]=features_f[i]->pose_world_.y;
                position[i][2]=features_f[i]->pose_world_.z;

                int id=features_f[i]->point_pre_camera[j].first;//帧id
                ceres::CostFunction* cost_function=ReprojectionError::creat(features_f[i]->point_pre_camera[j].second.x,features_f[i]->point_pre_camera[j].second.y);
                problem.AddResidualBlock(cost_function,NULL,c_rotation[id],c_translation[id],position[i]);
            }
        }
    }
    ceres::Solver::Options options;
    options.linear_solver_type=ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout=true;
    options.max_solver_time_in_seconds=0.2;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout<<summary.BriefReport()<<endl;
    if(summary.termination_type==ceres::CONVERGENCE||summary.final_cost<5e-03)
    {
        cout<<"Optimize success!"<<endl;
    }
    else
    {
        cout<<"Optimize fail!"<<endl;
        return false;
    }

    for(int i=0;i<=init_l;i++)
    {
        Q_s[i].w()=c_rotation[i][0];
        Q_s[i].x()=c_rotation[i][1];
        Q_s[i].y()=c_rotation[i][2];
        Q_s[i].z()=c_rotation[i][3];
        P_s[i].x()=c_translation[i][0];
        P_s[i].y()=c_translation[i][1];
        P_s[i].z()=c_translation[i][2];
    }
    for(int i=0;i<features_f.size();i++)
    {
        if(features_f[i]->if3D)
        {
            cout<<"error before:"<<i<<" "<<features_f[i]->pose_world_.x<<" "<<features_f[i]->pose_world_.y<<" "<<features_f[i]->pose_world_.z<<endl;
            cout<<"error after:"<<i<<" "<<position[i][0]<<" "<<position[i][1]<<" "<<position[i][2]<<endl;
            cout<<"error term:"<<i<<" "<<position[i][0]-features_f[i]->pose_world_.x<<" "<<position[i][1]-features_f[i]->pose_world_.y<<" "<<position[i][2]-features_f[i]->pose_world_.z<<endl;
            if(fabs(position[i][0]-features_f[i]->pose_world_.x)>5||fabs(position[i][1]-features_f[i]->pose_world_.y)>5||fabs(position[i][2]-features_f[i]->pose_world_.z)>10)
            {
                features_f[i]->if3D=false;
                cout<<"remove this point----------------------------------------------------------"<<endl;
                continue;
            }
            features_f[i]->pose_world_.x=position[i][0];
            features_f[i]->pose_world_.y=position[i][1];
            features_f[i]->pose_world_.z=position[i][2];
        }
    }
}


void Feature_tracking::recoverStructure()
{
    vector<Point2f> pre_points_cam,cur_points_cam;
    Q_s.resize(init_frame_flag+1);//设置大小
    P_s.resize(init_frame_flag+1);
    //设置首帧和第l帧位姿（Q，P）
    Q_s[0].setIdentity();
    P_s[0].setZero();
    Q_s[init_l]=Eigen::Quaterniond (eigen_R);
    P_s[init_l]=Eigen::Vector3d (eigen_t);
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
    vector<Point2f> pre_Features,cur_Features;
    vector<uint64_t > pre_Features_id;

    for(auto point:features_f)
    {
        uint64_t init=0;
        uint64_t finish=init_frame_flag;
        cout<<"finish"<<finish<<endl;
        if(point->TrackBy(finish))
        {
            pre_Features_id.push_back(point->id_);
            pre_points_cam.push_back(point->pointIncamera(init));
            cur_points_cam.push_back(point->pointIncamera(finish));
        }
    }
    cout<<"共同观测点："<<pre_Features_id.size()<<endl;
    //剔除+筛选
//    Mat imshow5;
//    imshow5=showMatch(allImg_f[0],allImg_f[init_frame_flag],pre_Features,cur_Features);
//    imshow("光流三角化0-l",imshow5);

    //变换到相机坐标系归一化平面，才能进行三角化
    vector<Point3d> points_position_norm;//三角化恢复出的3D特征点,第一帧的相机中
    Mat points4D;//三角化得到的是其次坐标

    triangulatePoints(first_pose,l_pose,pre_points_cam,cur_points_cam,points4D);

    points_position_norm=normalization(points4D);//归一化,转换成非齐次坐标
    int k=0;
    for(auto po:points_position_norm)
    {
        features_f[pre_Features_id[k]]->pose_world_=po;
        features_f[pre_Features_id[k]]->if3D=true;
        k++;
    }
    /*
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
            cout<<"feature "<<f->id_<<" is "<<f->quality_feature<<" in Frame "<<f->point_pre_frame[p].first<<" is "<<f->point_pre_frame[p].second<<" and camera frame "<<f->point_pre_camera[p].first<<" is "<<f->point_pre_camera[p].second<<" and his pose is "<<f->pose_world_<<endl;
        }
    }
    cout<<"具有深度的特征点数量："<<count<<endl;
    waitKey(0);*/

    cout<<"................................................finish RT and tri between two..............................................."<<endl;
    cout<<".......................................................Begin pnp ..............................................."<<endl;
    for(int num=1;num<init_l;num++)
    {
       int init =0;
       solveCamPoseByPnP(init,num);   //根据第0帧位姿计算第1帧到第l-1帧的位姿，并三角化点
    }
    cout<<"................................................. pnp finished..............................................."<<endl;
    cout<<"................................................. Begin recoverMoreFeatures..............................................."<<endl;
    recoverMoreFeatures();
    cout<<"................................................. recoverMoreFeatures finished..............................................."<<endl;
    cout<<"................................................. Begin recover3dPoseOfFeature..............................................."<<endl;
    recover3dPoseOfFeature();
    cout<<"................................................. recover3dPoseOfFeature finished..............................................."<<endl;
    /*
    for(auto f:features_f)
    {
        for(uint64_t p=0;p<f->point_pre_frame.size();p++)
        {
            cout<<"feature "<<f->id_<<" in Frame "<<f->point_pre_frame[p].first<<" is "<<f->point_pre_frame[p].second<<" and camera frame "<<f->point_pre_camera[p].first<<" is "<<f->point_pre_camera[p].second<<" pose is "<<f->pose_world_<<endl;
        }
    }
    waitKey(0);
    for(int cam=0;cam<init_frame_flag+1;cam++)
    {
        cout<<"camera"<<cam<<":"<<endl;
        cout<<Q_s[cam].toRotationMatrix()<<endl;
        cout<<P_s[cam]<<endl;
    }
    waitKey(0);
     */
}
void Feature_tracking::fuseFeatures(uint64_t id)
{

}
/**
 * @brief
 * 1.calc the relative RT between image 0 and image l
 * @return if successful
 */
bool Feature_tracking::initialization()
{
    bool RT=false;
    while(!RT)
    {
        //todo 增加初始化时RT的精度
        RT=recoverRT();//通过第1帧和第l帧恢复相机的位姿，relative_R,relative_t todo 可以添加参数，设置第一帧与第几帧进行初始化
    }
    cout<<"------------------------------finish solve R T...----------------------------------"<<endl;
    waitKey(0);
    recoverStructure();
    cout<<"------------------------------finish recoverStructure...----------------------------------"<<endl;
    waitKey(0);
    //todo 局部BA优化
    bool optimize_state;
    optimize_state=optimize();//优化，并将优化后位置变化很大的特征点if3D置为false
    cout<<"optimize_state:"<<optimize_state<<endl;
    cout<<"------------------------------finish optimize features and camera pose...----------------------------------"<<endl;
    waitKey(0);

    return optimize_state;//优化成功则视为初始化成功
}
/**
 * @brief
 * 1.Load image and extract features
 * 2.add features to features_f
 * 3.track to initialization
 * @param image
 * @param frame
 * @return if the distance is enough to initialization
 */
bool Feature_tracking::loadInitImage(const Mat& image,Frame::Ptr& frame)
{
    init_frame_count++;
    cout<<"loading init frame...init_frame_count:"<<init_frame_count<<"th image"<<endl;

        cur_frame_f=frame;

        allImg_f.push_back(image);
        cur_img_f=image.clone();
        Mat imshow1=cur_img_f.clone();
        mask = cv::Mat(image_height, image_width, CV_8UC1, cv::Scalar(255));

        if(init_frame_count!=0)
        {
            //设置上一帧的状态
            pre_img_f=allImg_f[init_frame_count-1].clone();
            uint64_t pre=init_frame_count-1;
            track_new(pre,cur_frame_f->id_);//光流跟踪，第init帧和第frame->id_帧
        }
        if(init_frame_count==0)
        {
            vector<Point2f> feature_new;//需要补充的点
            goodFeaturesToTrack(cur_img_f,feature_new,150,0.1,30,mask);//todo 参数定义特征数量和最小距离

            cout<<"第0帧检测到特征点数量:"<<feature_new.size()<<endl;
            for(auto f:feature_new) //把新剔除的点添加到该帧对应的特征点中
            {
                uint64_t frame_id=(uint64_t)init_frame_count;
                addFeature(f,frame_id);//为新检测到的特征点创建feature
            }
//            for(auto pt:feature_new)
//            {
//                circle(imshow1,pt,2,Scalar(0),2);
//            }
//            cout<<"第"<<init_frame_count<<"帧特征点数量："<<feature_new.size()<<endl;
//            imshow("inshow1",imshow1);
//            imshow("mask1",mask);
//            waitKey(0);
        }
    cout<<"dist:"<<dist<<endl;
        pre_frame_f=cur_frame_f;
    if(dist<init_dist)
        return false;
    else
    {
        //todo 设置状态量,如果初始化的判定需要改的话就需要改这个


        init_frame_flag=init_frame_count;
        init_l=init_frame_flag;
/* 打印初始化时所有的特征点
        cout<<"所有的特征点："<<endl;
        for(auto f:features_f)
        {
            for(uint64_t p=0;p<f->point_pre_frame.size();p++)
            {
                cout<<"feature "<<f->id_<<" in Frame "<<f->point_pre_frame[p].first<<" is "<<f->point_pre_frame[p].second<<" and camera frame "<<f->point_pre_camera[p].first<<" is "<<f->point_pre_camera[p].second<<endl;
            }
        }
        waitKey(0);
        */
        return true;
    }
}





















