//
// Created by comoer on 2021/10/11.
//
/*
 * essential matrix
 */
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace std;
using namespace cv;

string PATH_DATA = "/home/data/";
void find_match(const Mat &img1,const Mat &img2,vector<Point2f>& lpt,vector<Point2f>& rpt)
{
    Mat gray1,gray2;
    // using gray image to compute
    if(img1.dims == 3)
    {
        cvtColor(img1,gray1,COLOR_BGR2GRAY);
    }
    else{
        gray1 = img1;
    }
    if(img2.dims == 3)
    {
        cvtColor(img2,gray1,COLOR_BGR2GRAY);
    }
    else{
        gray2 = img2;
    }
    //create orb detector
    Ptr<ORB> orb = ORB::create();
    // create the container of Key points
    vector<KeyPoint> feature_points1,feature_points2;
    // do Orient_FAST detect Keypoint
    orb->detect(gray1,feature_points1);
    orb->detect(gray2,feature_points2);
    // compute the descriptors
    Mat descriptor1,descriptor2;
    orb->compute(gray1,feature_points1,descriptor1);
    orb->compute(gray2,feature_points2,descriptor2);
    //do matching
    //create Matcher
    BFMatcher matcher(NORM_HAMMING); //O(N^2)
    vector<DMatch> pairs;
    matcher.match(descriptor1,descriptor2,pairs);
    //You can also filter the match to generate
    double min_dist = 100000;
    // compute the minimum of the distance
    for(const DMatch&m:pairs)
    {
        if(m.distance < min_dist) min_dist = m.distance;
    }
    // filter
    for(const DMatch&m:pairs)
    {
        if(m.distance < max(min_dist*1.5,20.))
        {
            lpt.push_back(feature_points1[m.queryIdx].pt);
            rpt.push_back(feature_points2[m.trainIdx].pt);
        }
    }
}

int main(int argc,char** argv)
{
    // camera matrix
    FileStorage params(PATH_DATA + "camera.yaml",FileStorage::READ);
    Mat K = params["K"].mat();

    Mat img1 = imread(PATH_DATA + "stereo-data/0_orig.jpg");
    Mat img2 = imread(PATH_DATA + "stereo-data/1_orig.jpg");
    vector<Point2f> left_pts;
    vector<Point2f> right_pts;
    find_match(img1,img2,left_pts,right_pts);

    // find the Essential Matrix using RANSAC
    Mat E = findEssentialMat(left_pts,right_pts,K,RANSAC);
    cout<<"Essential Matrix is \n"<<E<<endl;
    Mat R,t;
    recoverPose(E,left_pts,right_pts,K,R,t);
    cout<<"R is \n"<<R<<endl;
    cout<<"t (unknown unit) is \n"<<t<<endl;

    return 0;
}

