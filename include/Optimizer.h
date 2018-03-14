//
// Created by jh on 18-3-12.
//

#ifndef MYPROJECT_OPTIMIZER_H
#define MYPROJECT_OPTIMIZER_H

#include "myheader.h"
#include "KeyFrame.h"
#include "Map.h"
#include "Feature.h"
#include "MapPoint.h"

class Optimizer
{
public:
    static void twoViewsBA();//两个关键帧
    static void motionBA();  //两个相邻帧
    static void windowBA();//两个关键帧
    static void globalBA();//两个关键帧


};


#endif //MYPROJECT_OPTIMIZER_H
