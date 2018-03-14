//
// Created by jh on 18-3-12.
//

#ifndef MYPROJECT_MAP_H
#define MYPROJECT_MAP_H

#include "myheader.h"
#include "MapPoint.h"
#include "KeyFrame.h"


class Map {  //主要是用于显示的
public:
    typedef std::shared_ptr<Map> Ptr;
    vector<MapPoint::Ptr> mapPointsInMap_;
    vector<KeyFrame::Ptr> keyFramesInMap_;
    void addMapPoint(const MapPoint::Ptr& mp);
    void eraseMapPoint(const MapPoint::Ptr& mp);
    void addKeyFrame(const KeyFrame::Ptr& kf);
    void eraseKeyFrame(const KeyFrame::Ptr& kf);

};


#endif //MYPROJECT_MAP_H
