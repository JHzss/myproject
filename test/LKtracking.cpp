//
// Created by jh on 18-3-5.
//

#include "myheader.h"


#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
using namespace cv;


int main(int argc,char** argv)
{
    if(argc!=2)
    {
        cout<<"usage:node  data_file_name"<<endl;
    }

    string data_file=argv[1];
    string rgb_time,rgb_name,depth_time,depth_name;
    vector<string> rgb_files,depth_files;
    vector<double > rgb_times,depth_times;
    ifstream fin(data_file+"/associate.txt");
    while(!fin.eof())
    {
        fin>>rgb_time>>rgb_name>>depth_time>>depth_name;
        rgb_times.push_back(atof(rgb_time.c_str()));
        depth_times.push_back(atof(depth_time.c_str()));
        rgb_files.push_back(data_file+"/"+rgb_name);
        depth_files.push_back(data_file+"/"+depth_name);
    }


    Mat rgb_img_cur,depth_img_cur,rgb_img_pre,depth_img_pre;

    //vector<KeyPoint> pre_keypoints,cur_keypoints;
    //Mat pre_descriptors,cur_descriptors;
    //list<KeyPoint> orb_keypoints;
    vector<unsigned char> status;
    vector<float> errors;

    vector<KeyPoint> orb_keypoints_pre,orb_keypoints_cur;
    cv::Ptr<ORB> orb_detector=cv::ORB::create(100,1.2f,8,31,0,2,ORB::HARRIS_SCORE,31,20);

    vector<Point2f> points1_,points2_;

    vector<string> image_names;
    string name1="/home/jh/桌面/slambook/1.png";
    string name2="/home/jh/data-orb/data/rgbd_dataset_freiburg1_desk/rgb/1305031452.791720.png";
    image_names.push_back(name1);
    image_names.push_back(name2);

    for(int i=0;i<2;i++)
    {

        float aver_x=0,aver_y=0;
        rgb_img_cur=imread(image_names[i],CV_LOAD_IMAGE_GRAYSCALE);

        if(i==0)
        {
            orb_detector->detect(rgb_img_cur,orb_keypoints_pre);
            cout<<orb_keypoints_pre.size()<<endl;
            rgb_img_pre=rgb_img_cur.clone();
//            orb_keypoints_pre=orb_keypoints_cur;

            continue;
        }
        orb_keypoints_cur.clear();



        if(i==1)
        {
            for(auto m:orb_keypoints_pre)
            {
                points1_.push_back(m.pt);
            }
        }

        cout<<points1_.size();
//        for(KeyPoint& m:orb_keypoints_cur)
//        {
//            points2.push_back(m.pt);
//        }

        cv::calcOpticalFlowPyrLK(rgb_img_pre,rgb_img_cur,points1_,points2_,status,errors);

        Mat img_show;
        hconcat(rgb_img_pre,rgb_img_cur,img_show);


        cout<<img_show.rows<<endl;
        int k=0;

        list<cv::Point2f> points1,points2;
        list<cv::Point2f> points1__,points2__;
        for(auto iter2=points2_.begin(),iter1=points1_.begin();iter2!=points2_.end();k++)
        {
            if(status[k]!=0)
            {
                points1.push_back(*iter1);
                points2.push_back(*iter2);
                iter1++;
                iter2++;
            }
            else
            {
                iter1++;
                iter2++;
            }
        }
        cout<<"points size:"<<points1.size()<<endl;
        for(auto iter2=points2.begin(),iter1=points1.begin();iter2!=points2.end();)
        {
            aver_x=aver_x+fabs(iter2->x-iter1->x);
            aver_y=aver_y+fabs(iter2->y-iter1->y);
            iter1++;
            iter2++;
        }
        aver_x=aver_x/(float)points1_.size();
        aver_y=aver_y/(float)points1_.size();
        cout<<aver_x<<"   "<<aver_y<<endl;
        points1_.clear();
        for(auto iter2=points2.begin(),iter1=points1.begin();iter2!=points2.end();)
        {
            if(fabs(iter2->x-iter1->x)<2*aver_x&&fabs(iter2->y-iter1->y)<2*aver_y)
            {
                points1__.push_back(*iter1);
                points1_.push_back(*iter2);
                points2__.push_back(*iter2);
                iter1++;
                iter2++;
            }
            else
            {
                iter1++;
                iter2++;
            }

        }
        cout<<"points-- size:"<<points1__.size()<<endl;
        for(auto iter2=points2__.begin();iter2!=points2__.end();)
        {
            iter2->x+=640;
            iter2++;
        }
        cv::RNG rng(time(0));

        while(points1__.size()!=0)
        {
            line(img_show,points1__.front(),points2__.front(),cv::Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255)),1);
            points1__.pop_front();
            points2__.pop_front();
        }
        rgb_img_pre=rgb_img_cur.clone();
        orb_keypoints_pre=orb_keypoints_cur;

        imshow("img_show",img_show);
        waitKey(0);
    }
    destroyAllWindows();

    return 0;

}