//
// Created by jh on 18-3-8.
//
#include "myheader.h"
#include "parameters.h"
Parameters::Parameters(const string &configFile)
{
    FileStorage config(configFile.c_str(),FileStorage::READ);
    ReadParameters(config);
    camera_k=(Mat_<double>(3,3)<< camera_fx,0,camera_cx,0,camera_fy,camera_cy,0,0,1.0);
//    cout<<camera_depth<<endl;
}

void Parameters::ReadParameters(const FileStorage& filename)
{
    camera_fx=filename["camera.fx"];
    camera_fy=filename["camera.fy"];
    camera_cx=filename["camera.cx"];
    camera_cy=filename["camera.cy"];

    camera_k1=filename["camera.k1"];
    camera_k2=filename["camera.k2"];
    camera_k3=filename["camera.k3"];
    camera_p1=filename["camera.p1"];
    camera_p2=filename["camera.p2"];

    number_of_features=filename["number_of_features"];
    init_dist=filename["init_dist"];

    image_width=filename["image.width"];
    image_height=filename["image.height"];
    slideWindowsize=filename["slideWindowsize"];
}

