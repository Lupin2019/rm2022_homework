#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

using namespace std;
using namespace cv;

string PATH_VIDEO = "/home/data/video/";

int main(){
    cv::VideoCapture cap_left(PATH_VIDEO+"left_10.mp4");
    cv::VideoCapture cap_right(PATH_VIDEO+"right_10.mp4");
    cv::Mat src_left;
    cv::Mat src_right;
    cv::Mat src_lf;
    cv::namedWindow("src_lf",0);
    while(cap_left.read(src_left) && cap_right.read(src_right)){
        cv::hconcat(src_left, src_right, src_lf);
        // cv::imshow("src_left", src_left);
        // cv::imshow("src_right", src_right);
        cv:imshow("src_lf", src_lf);
        cv::waitKey(50);
    }
    return 0;
}