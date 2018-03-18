//
// Created by jh on 18-3-12.
//
#include "myheader.h"
#include "Camera.h"
#include "parameters.h"
#include "Read_dataset.h"
#include "System.h"
void process()
{


}

int main(int argc,char** argv)
{
    if(argc!=3)
    {
        cout<<"usage:node  data_file_name config_file_name"<<endl;
        return 1;
    }
    //string data_file=argv[1];
    Read_dataset::Ptr dataset=Read_dataset::creat(argv[1]);
//    cout<<dataset->depth_files.size()<<endl;
    System::Ptr system=System::creat(argv[2]);
//    cout<<system->camera_s->getK()<<endl;
    for(int i=0;i<dataset->rgb_files.size();i++)
    {
        cout<<"i:"<<i<<endl;
        Mat image=imread(dataset->rgb_files[i],IMREAD_GRAYSCALE);
        double timestamp=dataset->rgb_times[i];
        system->Image_process(image,timestamp);
    }




    return 0;
}
