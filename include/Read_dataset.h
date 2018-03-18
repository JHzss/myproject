//
// Created by jh on 18-3-8.
//

#ifndef MYPROJECT_READ_DATASET_H
#define MYPROJECT_READ_DATASET_H

#include "myheader.h"


class Read_dataset
{
public:
    Read_dataset(const string& dataset);
    typedef shared_ptr<Read_dataset> Ptr;
    vector<string> rgb_files,depth_files;
    vector<double > rgb_times,depth_times;
    inline static Read_dataset::Ptr creat(const string& dataset){ return Read_dataset::Ptr(new Read_dataset(dataset));}
public:
    void read_from_dataset(const string& dataset_filename );

private:
    string rgb_time,rgb_name,depth_time,depth_name;

};


#endif //MYPROJECT_READ_DATASET_H
