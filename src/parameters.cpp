//
// Created by jh on 18-3-8.
//
#include "myheader.h"
#include "parameters.h"
void Parameters::ReadParameters(const FileStorage& filename)
{
    camera_fx=filename["camera.fx"];
    camera_fy=filename["camera.fy"];
    camera_cx=filename["camera.cx"];
    camera_cy=filename["camera.cy"];
    camera_depth=filename["camera.depth_scale"];
    number_of_features=filename["number_of_features"];
}
