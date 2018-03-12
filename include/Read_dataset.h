//
// Created by jh on 18-3-8.
//

#ifndef MYPROJECT_READ_DATASET_H
#define MYPROJECT_READ_DATASET_H

#include "myheader.h"


class Read_dataset
{
public:
    typedef shared_ptr<Read_dataset> Ptr;
    vector<string> rgb_files,depth_files;
    vector<double > rgb_times,depth_times;
public:
    void read_from_dataset(const string& dataset_filename );

private:
    string rgb_time,rgb_name,depth_time,depth_name;

};


#endif //MYPROJECT_READ_DATASET_H
