//
// Created by jh on 18-3-6.
//
#include "myheader.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/opencv.hpp>

#include "Camera.h"
#include "parameters.h"
#include "Read_dataset.h"
#include "Feature_tracking.h"

int main(int argc,char** argv)
{
//    if(argc!=3)
//    {
//        cout<<"usage:node  data_file_name"<<endl;
//        return 1;
//    }
//    string data_file=argv[1];
//
//    cv::Point2d aa(1,2);
//    aa /= 2.0;
//    const int N = 10;
//    aa /= N;
//
//    Parameters::Ptr param;
//    string parameters_file=argv[2];
//    FileStorage setting_file(parameters_file.c_str(),FileStorage::READ);
//    param->ReadParameters(setting_file);
//    Camera::Ptr camera;
//    camera->creat(param);
////    cout<<param.camera_fx<<param.camera_fy<<endl;
//    Read_dataset dataset;
//    dataset.read_from_dataset(argv[1]);
//    Feature_tracking tracker;
////    camera.K_=Camera::getK(param);
//
//    for(int i=0;i<dataset.rgb_files.size();i++)
//    {
//        tracker.clearstate();
//        tracker.loadImage(dataset.rgb_files[i]);
//        if(tracker.init_flag)
//        {
//          tracker.initialization(param,camera);
//        }
//            drawMatches(tracker.pre_img,tracker.pre_keypoints,tracker.cur_img,tracker.cur_keypoints,tracker.good_matches,tracker.img_match);
//            cout<<tracker.camera_pose.rotationMatrix()<<endl;
//            cout<<tracker.camera_pose.translation()<<endl;
//            imshow("match",tracker.img_match);
//            waitKey(0);
//        return 0;
//    }
//
//    destroyAllWindows();
    return 0;

}