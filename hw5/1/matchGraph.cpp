#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>

using namespace cv;
using namespace std;

string PATH_VIDEO = "/home/data/video/";
void jly_showMatches(const Mat &img,const Mat &img2){
    cv::namedWindow("show", 0);
    cv::Mat gray, gray2;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);
    Ptr<ORB> orb =ORB::create();
    vector<KeyPoint> feature_points,feature_points2;
    // do Orient_FAST detect Keypoint
    orb->detect(gray,feature_points);
    orb->detect(gray2,feature_points2);
    // compute the descriptors
    Mat descriptor1,descriptor2;
    orb->compute(gray,feature_points,descriptor1);
    orb->compute(gray2,feature_points2,descriptor2);
    //do matching
    //create Matcher
    BFMatcher matcher(NORM_HAMMING); //O(N^2)
    vector<DMatch> pairs;
    matcher.match(descriptor1,descriptor2,pairs);
    printf("DMatch contains the matched points like (%d in img1,%d in img2) their distance is %.3f (in Hamming Norm).\n"
           ,pairs[0].queryIdx,pairs[0].trainIdx,pairs[0].distance);
    Mat canvas;
    drawMatches(img,feature_points,img2,feature_points2,pairs,canvas);
    imshow("show",canvas);
    waitKey(0);
    //You can also filter the match to generate
    vector<DMatch> good;
    double min_dist = 100000;
    // compute the minimum of the distance
    for(const DMatch&m:pairs)
    {
        if(m.distance < min_dist) min_dist = m.distance;
    }
    // filter
    for(const DMatch&m:pairs)
    {
        if(m.distance < max(min_dist*2,30.))
        {
            good.push_back(m);
        }
    }
    drawMatches(img,feature_points,img2,feature_points2,good,canvas);
    imwrite("../keypointss/good_matches/good_match.jpg",canvas);
    imshow("show",canvas);
    waitKey(0);
    // draw the keypoint
    drawKeypoints(img,feature_points,canvas,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imwrite("../keypoints.jpg",canvas);
    imshow("show",canvas);
    waitKey(0);
}
int main(){
    cv::VideoCapture cap_left(PATH_VIDEO+"left_10.mp4");
    cv::VideoCapture cap_right(PATH_VIDEO+"right_10.mp4");
    cv::Mat src_left;
    cv::Mat src_right;
    cv::Mat src_lf;
    cv::namedWindow("src_lf",0);
    while(cap_left.read(src_left) && cap_right.read(src_right)){
        // cv::hconcat(src_left, src_right, src_lf);
        // // cv::imshow("src_left", src_left);
        // // cv::imshow("src_right", src_right);
        // cv:imshow("src_lf", src_lf);
        
        jly_showMatches(src_left, src_right);
    }
    return 0;
}