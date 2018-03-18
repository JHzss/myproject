//
// Created by jh on 18-3-14.
//

#include "myheader.h"
#include "Camera.h"
#include "parameters.h"
#include "Read_dataset.h"

int main(int argc,char** argv)
{
    if(argc!=3)
    {
        cout<<"usage:node  data_file_name"<<endl;
        return 1;
    }
    //string data_file=argv[1];
    string parameters_file=argv[2];
    Parameters::Ptr parameter=Parameters::creat(parameters_file);
    Camera::Ptr mono_cam=Camera::creat(parameter);
    cout<<mono_cam->getK()<<endl;
    Read_dataset::Ptr dataset=Read_dataset::creat(argv[1]);
    cout<<dataset->depth_files.size()<<endl;
    return 0;
}