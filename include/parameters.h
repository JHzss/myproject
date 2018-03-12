//
// Created by jh on 18-3-8.
//
#pragma once

#ifndef MYPROJECT_PARAMETERS_H
#define MYPROJECT_PARAMETERS_H

#include "myheader.h"

class Parameters
{
public:
    typedef shared_ptr<Parameters> Ptr;
    double camera_fx;
    double camera_fy;
    double camera_cx;
    double camera_cy;
    double camera_depth;

    int number_of_features;

public:
void ReadParameters(const FileStorage& filename);
};


#endif //MYPROJECT_PARAMETERS_H
