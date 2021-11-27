#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
// #include <opencv4/opencv2/opencv.hpp>

using namespace std;
using namespace cv;

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
    // imwrite("../keypointss/good_matches/good_match.jpg",canvas);
    imshow("show",canvas);
    
    // draw the keypoint
    drawKeypoints(img,feature_points,canvas,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // imwrite("../keypoints.jpg",canvas);
    imshow("show",canvas);
    
}


int main(){

    // camera matrix
    FileStorage params("/home/data/camera.yaml",FileStorage::READ);
    Mat K = params["K"].mat();

    cv::VideoCapture cap_left(PATH_VIDEO+"left_10.mp4");
    cv::VideoCapture cap_right(PATH_VIDEO+"right_10.mp4");
    cv::Mat src_left;
    cv::Mat src_right;
    cv::Mat res_left, res_right;
    // cv::namedWindow("src_lf",0);
    cap_left.read(src_left);
    cap_right.read(src_right);
    cv::Rect roi =  cv::selectROI(src_left);
    // // 322 x 247 from (809, 580
    // cv::Rect roi(809, 580, 322, 247);
    // cout << roi[0] << endl;
    cout << roi << endl;
    src_left(roi).copyTo(res_left);
    src_right(roi).copyTo(res_right);
    cv::namedWindow("res_left", WINDOW_NORMAL);
    cv::namedWindow("res_right", WINDOW_NORMAL);
    cv::imshow("res_left", res_left);
    cv::imshow("res_right", res_right);
    cv::waitKey(1000);
    int n = 0;

    // E H 
    cv::namedWindow("show", 0);
    cv::Mat gray, gray2;
    Ptr<ORB> orb =ORB::create();
    vector<KeyPoint> feature_points,feature_points2;
    Mat descriptor1,descriptor2;
    //do matching
    //create Matcher
    BFMatcher matcher(NORM_HAMMING); //O(N^2)
    vector<DMatch> pairs;
    Mat canvas;
    //You can also filter the match to generate
    vector<DMatch> good;
    double min_dist = 100000;
    // compute the minimum of the distance
    


    while(cap_left.read(src_left) && cap_right.read(src_right)){
        n += 1;
        cout << n << "\n" << endl;
        
        
        src_left(roi).copyTo(res_left);
        src_right(roi).copyTo(res_right);
        
        
        cv::cvtColor(res_left, gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(res_right, gray2, cv::COLOR_BGR2GRAY);
            // do Orient_FAST detect Keypoint
        orb->detect(gray,feature_points);
        orb->detect(gray2,feature_points2);
        // compute the descriptors
        orb->compute(gray,feature_points,descriptor1);
        orb->compute(gray2,feature_points2,descriptor2);
        matcher.match(descriptor1,descriptor2,pairs);
        printf("DMatch contains the matched points like (%d in img1,%d in img2) their distance is %.3f (in Hamming Norm).\n"
            ,pairs[0].queryIdx,pairs[0].trainIdx,pairs[0].distance);
        // drawMatches(res_left,feature_points,res_right,feature_points2,pairs,canvas);
        // imshow("show0",canvas);
        double min_dist = 100000;
        for(const DMatch&m:pairs)
        {
            if(m.distance < min_dist) min_dist = m.distance;
        }
        // filter
        for(const DMatch&m:pairs)
        {
            if(m.distance < max(min_dist*2,70.))
            {
                cout << n << "\n" << endl;
                good.push_back(m);
            }
        }
        drawMatches(res_left,feature_points,res_right,feature_points2,good,canvas);
        // imwrite("../keypointss/good_matches/good_match.jpg",canvas);
        imshow("show1",canvas);
        
        #if 0
        // draw the keypoint
        drawKeypoints(res_left,feature_points,canvas,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        // imwrite("../keypoints.jpg",canvas);
        imshow("show2",canvas);
        #endif 

        cv::imshow("res_left", res_left);
        cv::imshow("res_right", res_right);
        // ly_showMatches(res_left, res_right);
        cv::waitKey(0);
    }
    cv::destroyAllWindows();
    return 0;
}