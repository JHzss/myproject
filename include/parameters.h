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
    friend class Feature_tracking;
//    friend float get_init_dist();

    typedef shared_ptr<Parameters> Ptr;
    static Parameters::Ptr creat(const string& configFile){ return Parameters::Ptr(new Parameters(configFile));}
    double camera_fx;
    double camera_fy;
    double camera_cx;
    double camera_cy;
    double camera_depth;
    Mat camera_k;
    int number_of_features;

    int image_width;
    int image_height;
    int slideWindowsize;
    float init_dist;

    void ReadParameters(const FileStorage& filename);
private:
     Parameters(const string& configFile);

};


#endif //MYPROJECT_PARAMETERS_H
