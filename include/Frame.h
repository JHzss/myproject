//
// Created by jh on 18-3-9.
//

#ifndef MYPROJECT_FRAME_H
#define MYPROJECT_FRAME_H

#include "myheader.h"
#include "Camera.h"
#include "Feature.h"

class Feature;
class Camera;


class Frame {
public:
    typedef shared_ptr<Frame> Ptr;
    uint64_t id_;
    static uint64_t next_id_;
    const double timestamps_;
    vector<Feature::Ptr> featuresInThisFrame_;//Frame中的特征点
    vector<uint64_t > features_id_lists_;
    vector<Point2d> points_cam_;//Frame中的特征点转换到相机坐标系
    vector<Point3d> points_world_;//Frame中的特征点转换到世界坐标系
    SE3 T_c2w;//相机位姿
    SE3 T_w2c;
    const Camera::Ptr camara_;//该Frame所在的相机
    const Mat img_;
    Eigen::Matrix3d R_;
    Vector3d t_;

public:
    Frame(const Mat& img, const double& timeStamps,const Camera::Ptr &cam);
    void addFeatureInFrame(Feature::Ptr& feature);
    void eraseFeatureInFrame(Feature::Ptr& feature);

    void setPose(const SE3& pose);//T_w2c,T_c2w,R,t
    int getFeatureNumber();
    vector<Point2d> getPoints_cam(const Camera::Ptr &cam);//利用提取出的特征点计算其在相机坐标系下的坐标
    vector<Point3d> getPoints_world(const Camera::Ptr &cam);//利用提取出的特征点计算其在世界坐标系下的坐标

    inline static Frame::Ptr creat(const Mat& img, const double& timeStamps, const Camera::Ptr &cam) { return Frame::Ptr(new Frame(img, timeStamps,cam));}

private:
    Feature::quality_ checkFeatureQuality(const Feature::Ptr& feature){ return feature->quality_feature;}//todo 类的枚举类型怎么用
};


#endif //MYPROJECT_FRAME_H
