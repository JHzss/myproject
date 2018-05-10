//
// Created by jh on 18-3-12.
//

#include "Feature.h"
#include "parameters.h"
uint64_t Feature::next_id_=0;
Feature::Feature() :quality_feature(normal),track_times_(0),if3D(false),id_(next_id_++)
{
    cout<<"creat feature"<<id_<<endl;
//this->pose_world_=Point3d(0,0,-1);
}
/**
 *
 * @param features 系统所有的特征点
 * @param pre_points_ids points的id
 * @param frame_id  本帧的id
 * @param point 本帧中的特征点
 * @param k 相机内参矩阵
 */
void Feature::addTrack(vector<Feature::Ptr>& features,vector<uint64_t> &pre_points_ids, uint64_t frame_id, vector<Point2f> &point,Mat& k)
{
    int it=0;
    for(uint64_t num:pre_points_ids)
    {
        Point2f point_nodistort;
        point_nodistort=Camera::removeDistort(point[it],camera_k1,camera_k2,camera_p1,camera_p2,k);
        features[num]->addTrack(frame_id,point[it],point_nodistort);
        it++;
    }
}

void Feature::addTrack(uint64_t frame_id, Point2f &point,Point2f &point_nodistort)
{
    point_pre_frame_nodistort.push_back(make_pair(frame_id,point_nodistort));
    point_pre_camera.push_back(make_pair(frame_id,Camera::uv2camera(point_nodistort,camera_k)));//去畸变之后的相机平面下的坐标
    point_pre_frame.push_back(make_pair(frame_id,point));
    track_times_++;
}
bool Feature::TrackBy(uint64_t &frame_id)
{
    for(auto tr:this->point_pre_frame)
    {
        if(tr.first==frame_id)
            return true;
    }
    return false;
}
Point2f Feature::pointIn_nodistort(uint64_t &frame_id)
{
    for(auto tr:this->point_pre_frame_nodistort)
    {
        if(tr.first==frame_id)
            return tr.second;
    }
}

Point2f Feature::pointIncamera(uint64_t &frame_id)
{
    for(auto tr:this->point_pre_camera)
    {
        if(tr.first==frame_id)
            return tr.second;
    }
}
Point2f Feature::pointIn_raw(uint64_t &frame_id)
{
    for(auto tr:this->point_pre_frame)
    {
        if(tr.first==frame_id)
            return tr.second;
    }
}

//清空观测量
void Feature::destoryFeature()
{
    this->quality_feature=bad;
    this->pose_world_= Point3d(0,0,-1);
    this->if3D=false;
    this->point_pre_frame.clear();
    this->point_pre_camera.clear();
    this->track_times_=0;

}

//todo 还没有完成
bool judge(const pair<uint64_t ,Point2f> a,const pair<uint64_t ,Point2f> b)
{
    return a.second.x>b.second.x; //从大到小排列
}
bool judgeid(const pair<uint64_t ,uint64_t> a,const pair<uint64_t ,uint64_t> b)
{
    return a.first>b.first; //从大到小排列
}
bool judgeuv(const pair<uint64_t,Point> a,const pair<uint64_t,Point> b)
{
    return a.first<b.first; //从小到大排列
}
bool judgecam(const pair<uint64_t,Point2f> a,const pair<uint64_t,Point2f> b)
{
    return a.first<b.first; //从小到大排列
}
//
void Feature::fuseSameFeature(vector<Feature::Ptr> &features,uint64_t l)
{

    vector<pair<uint64_t ,uint64_t>> same_f;//遍历所有的帧得到的所有相同的特征点
    for(uint64_t i=0;i<=l;i++)
    {
//        vector<Feature::Ptr> features_tmp;
        vector<pair<uint64_t ,Point2f>> pairs;  //特征id，uv组的pair
        vector<Point2f> points;  //第i帧所有的uv
        vector<uint64_t > ids;   //uv对应的特征id
        uint64_t id;  //临时变量
        for(auto &f:features)//遍历所有的特征
        {
            if(f->TrackBy(i))//找到第i帧中所有的特征点
            {
                pairs.push_back(make_pair(f->id_,f->pointIn_nodistort(i)));
//                features_tmp.push_back(f);
            }
        }
        sort(pairs.begin(),pairs.end(),judge);
        float pre_x=pairs.begin()->second.x;
        float pre_y=pairs.begin()->second.y;
        uint64_t pre_id=pairs.begin()->first;
        vector<pair<uint64_t ,Point2f>>::iterator iter;
        for(iter=pairs.begin()+1;iter!=pairs.end();iter++)//找到所有的特征点相近的特征点
        {
            if(((*iter).second.x-pre_x)<2.0)
            {
                if(((*iter).second.y-pre_y)<2.0)
                {
                    uint64_t cur_id=(*iter).first;
                    same_f.push_back(make_pair(pre_id,cur_id));
                }
                else
                {
                    pre_x=(*iter).second.x;
                    pre_y=(*iter).second.y;
                    pre_id=(*iter).first;
                }
            }
            else
            {
                pre_x=(*iter).second.x;
                pre_y=(*iter).second.y;
                pre_id=(*iter).first;
            }
        }
    }

    sort(same_f.begin(),same_f.end(),judgeid);//从大到小排序  56/45/34/23，依次向前合并

    for(auto &pair:same_f)
    {
        uint64_t pre=pair.first;
        uint64_t cur=pair.second;
        for(int n=0;n<features[cur]->point_pre_frame.size();n++)
        {
            features[cur]->point_pre_frame[n].first=pre;
            features[cur]->point_pre_camera[n].first=pre;
            features[cur]->quality_feature=bad;
        }
        features[pre]->point_pre_frame.insert(features[pre]->point_pre_frame.end(),features[cur]->point_pre_frame.begin(),features[cur]->point_pre_frame.end());
        features[pre]->point_pre_camera.insert(features[pre]->point_pre_camera.end(),features[cur]->point_pre_camera.begin(),features[cur]->point_pre_camera.end());
        features[pre]->pose_world_=(features[pre]->pose_world_+features[cur]->pose_world_)/2;
        features[cur]->point_pre_frame.clear();
        features[cur]->point_pre_camera.clear();
    }

    for(auto &f:features)//按照观测帧排序
    {
        sort(f->point_pre_frame.begin(),f->point_pre_frame.begin(),judgeuv);
        sort(f->point_pre_frame.begin(),f->point_pre_frame.begin(),judgeuv);
    }

//    for(auto &f:features)
//    {
//        vector<pair<uint64_t ,Point>>::iterator iter;
//        vector<pair<uint64_t ,Point2f>>::iterator iter1;
//        for( iter=f->point_pre_frame.begin();iter!=f->point_pre_frame.end();)
//        {
//            if((*iter).first==f->(*(iter)).first)
//            {
//                (*iter).second=((*iter).second+f->(*(iter+1)).second)/2;
//                f->point_pre_frame.erase(iter);
//                continue;
//            }
//            iter++;
//        }
//        for( iter1=f->point_pre_camera.begin();iter1!=f->point_pre_camera.end();)
//        {
//            if((*iter1).first==f->(*(iter1+1)).first)
//            {
//                (*iter1).second=((*iter1).second+f->(*(iter1+1)).second)/2;
//                f->point_pre_camera.erase(iter1);
//                continue;
//            }
//            iter1++;
//        }
//    }

}