#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/opencv.hpp>
#include <string>
using namespace cv;

Mat camera_matrix( 3, 3, CV_32FC1, Scalar::all( 0 ));
Mat dist_coeffs(1, 5, CV_32FC1, Scalar::all( 0 ));


int main(){
    camera_matrix = (Mat_<double>(3,3) << 1095.813935769838, 0, 1067.804871518417, 
    0, 1097.557970551979, 617.9762181262952,
    0, 0, 1);
    dist_coeffs = (Mat_<double>(1,5) << -0.05619281664155706, 0.09789783658435036, 0.0007822181639836854, -0.001785154696106422, -0.04337279910667968);
    namedWindow("src",0);
    namedWindow("distort",0);
    
    std::string path_src = "/home/nvidia/gitee/rb_vm_ubuntu/hw2/calib2/";
    Mat show;
    for (int i = 0; i < 20; i++){ 
        cv::Mat src0 = cv::imread(path_src+ std::__cxx11::to_string(i).append("_orig.jpg"));
        undistort(src0, show, camera_matrix, dist_coeffs);
        if(i == 0){
            imwrite("/home/nvidia/gitee/rb_vm_ubuntu/hw2/calib2_undistorted/1_undistored.jpg",show);
        }
        imshow("src",src0);
        imshow("distort",show);
        waitKey(2000);
    }
    return 0;
}