//
// Created by jh on 18-3-12.
//

#include "Feature.h"
uint64_t Feature::next_id_=0;
Feature::Feature(Point2f &p) :quality_feature(normal),track_times_(0),if3D(false),id_(next_id_++)
{


}
void Feature::addTrack(vector<Feature::Ptr>& features,vector<uint64_t> &pre_points_ids, uint64_t frame_id, vector<Point2f> &point,Mat& k)
{
    int it=0;
    for(uint64_t num:pre_points_ids)
    {
        features[num]->point_pre_frame.push_back(make_pair(frame_id,point[it]));
        features[num]->point_pre_camera.push_back(make_pair(frame_id,Camera::uv2camera(point[it],k)));
        it++;
    }
}


void Feature::addTrack(uint64_t frame_id, Point2f &point)
{
    this->point_pre_frame.push_back(make_pair(frame_id,point));
    this->track_times_++;
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
Point2f Feature::pointIn(uint64_t &frame_id)
{
    for(auto tr:this->point_pre_frame)
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

void Feature::fuse_feature()
{

}
void Feature::removeRepeatFeature()
{

}
