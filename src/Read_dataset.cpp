//
// Created by jh on 18-3-8.
//

#include "Read_dataset.h"
#include "myheader.h"

Read_dataset::Read_dataset(const string &dataset)
{
    read_from_dataset(dataset);
}

void Read_dataset::read_from_dataset(const string& dataset_filename )
{
    string data_file=dataset_filename;
    ifstream fin(data_file+"/associate.txt");
    while(!fin.eof())
    {
        fin>>rgb_time>>rgb_name>>depth_time>>depth_name;
        rgb_times.push_back(atof(rgb_time.c_str()));
        depth_times.push_back(atof(depth_time.c_str()));
        rgb_files.push_back(data_file+"/"+rgb_name);
        depth_files.push_back(data_file+"/"+depth_name);
    }
}