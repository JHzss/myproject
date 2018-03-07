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
    cv::Ptr<ORB> orb_detector=cv::ORB::create(500,1.2f,8,31,0,2,ORB::HARRIS_SCORE,31,20);

    vector<Point2f> points1_,points2_;

    for(int i=0;i<10;i++)
    {

        rgb_img_cur=imread(rgb_files[i]);
        depth_img_cur=imread(depth_files[i]);

        if(i==0)
        {
            orb_detector->detect(rgb_img_cur,orb_keypoints_pre);
            cout<<orb_keypoints_pre.size()<<endl;
            rgb_img_pre=rgb_img_cur.clone();
//            orb_keypoints_pre=orb_keypoints_cur;

            continue;
        }
        orb_keypoints_cur.clear();



        for(auto m:orb_keypoints_pre)
        {
            points1_.push_back(m.pt);
        }
        cout<<points1_.size();
//        for(KeyPoint& m:orb_keypoints_cur)
//        {
//            points2.push_back(m.pt);
//        }

        cv::calcOpticalFlowPyrLK(rgb_img_pre,rgb_img_cur,points1_,points2_,status,errors);
        Mat img_show=rgb_img_cur.clone();
        cout<<img_show.rows<<endl;
        int k=0;

        list<cv::Point2f> points1,points2;
        for(auto iter2=points2_.begin(),iter1=points1_.begin();iter2!=points2_.end();k++)
        {
            if(status[k]!=0)
            {
                points1.push_back(*iter1);
                points2.push_back(*iter2);
                iter1++;
                iter2++;
            }
            iter1++;
            iter2++;
        }
        while(points1.size()!=0)
        {
            line(img_show,points1.front(),points2.front(),cv::Scalar(0,240,0),2);
            points1.pop_front();
            points2.pop_front();
        }
        rgb_img_pre=rgb_img_cur.clone();
        orb_keypoints_pre=orb_keypoints_cur;

        imshow("img_show",img_show);
        waitKey(200);
    }
    destroyAllWindows();

    return 0;

}