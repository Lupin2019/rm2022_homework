#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

string PATH_DATA = "/home/data/";


int main(){
    FileStorage camera(PATH_DATA + "stereo.yaml",FileStorage::READ);
    Mat K_0 = camera["K_0"].mat();
    Mat C_0 = camera["C_0"].mat();
    Mat K_1 = camera["K_1"].mat();
    Mat C_1 = camera["C_1"].mat();

    FileStorage params(PATH_DATA + "extrinsics.yml",FileStorage::READ);
    Mat R = params["R"].mat(); // 3 * 3
    Mat T = params["T"].mat(); // 3 * 1

    Mat imgL, imgR;
    Mat grayL, grayR;
    Mat roiL, roiR;
    Mat canvas;

    Ptr<ORB> orb =ORB::create(); 
    vector<KeyPoint> feature_pointsL,feature_pointsR; 
    Mat descriptorL,descriptorR;
    BFMatcher matcher(NORM_HAMMING); //O(N^2)
    vector<DMatch> pairs;
    vector<Point2f> lpt, rpt;
    vector<Point2f> undistort_lpts,undistort_rpts;

    // start here : 
    cv::VideoCapture cap_left(PATH_DATA+"video/left_10.mp4");
    cv::VideoCapture cap_right(PATH_DATA+"video/right_10.mp4");

    // 先各加载一张, 并且选取ROI
    cap_left.read(imgL);
    cap_right.read(imgR);
    Rect roi = selectROI(imgL);
    cout << roi << endl; 
    
    // 图像初始化
    namedWindow("out", WINDOW_NORMAL);

    // video save
    VideoWriter writer("../output.avi",
        VideoWriter::fourcc('M', 'J', 'P', 'G'), 50, 
        cv::Size(imgL.cols+imgR.cols, imgL.rows), false);
    
    float distance = 0;

    while(cap_left.read(imgL) && cap_right.read(imgR)){
        imgL(roi).copyTo(roiL);
        imgR(roi).copyTo(roiR); 
        // roiR = imgR;

        if(roiL.dims == 3)
        {
            cvtColor(roiL, grayL, COLOR_BGR2GRAY);
        }
        else{
            grayL = roiL;
        }
        if(roiR.dims == 3)
        {
            cvtColor(roiR, grayR, COLOR_BGR2GRAY);
        }
        else{
            grayR = roiR;
        }

        // do Orient_FAST detect Keypoint
        orb->detect(grayL,feature_pointsL);
        orb->detect(grayR,feature_pointsR);
        // compute the descriptors
        orb->compute(grayL,feature_pointsL,descriptorL);
        orb->compute(grayR,feature_pointsR,descriptorR);
        matcher.match(descriptorL,descriptorR,pairs);

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
            if(m.distance < max(min_dist*1.5,10.))
            {
                lpt.push_back(feature_pointsL[m.queryIdx].pt);
                rpt.push_back(feature_pointsR[m.trainIdx].pt);
            }
        }
        // TRANSATION
        for (int i = 0; i < feature_pointsL.size(); i++){
            feature_pointsL[i].pt.x += roi.x;
            feature_pointsL[i].pt.y += roi.y;
        }
        for (int i = 0; i < feature_pointsR.size(); i++){
            feature_pointsR[i].pt.x += roi.x;
            feature_pointsR[i].pt.y += roi.y;
        }
        // draw 
        rectangle(imgL, roi, Scalar(0,255,0), 3);
        rectangle(imgR, roi, Scalar(0,255,0), 3);
        drawMatches(imgL, feature_pointsL, imgR, feature_pointsR, pairs, canvas);
        
        /*create translation Matrix*/
        
        Mat T1 = Mat::eye(3,4,CV_64F);

        Mat T2 = Mat(3,4,CV_64F);

        R.convertTo(T2(Rect(0,0,3,3)),CV_64F);

        T.convertTo(T2(Rect(3,0,1,3)),CV_64F);

        /* 变换到归一化相机坐标系*/

        vector<Point2f> undistort_lpts,undistort_rpts;

        undistortPoints(lpt,undistort_lpts,K_0,C_0);

        undistortPoints(rpt,undistort_rpts,K_1,C_1);

        Mat results;

        triangulatePoints(T1,T2,undistort_lpts,undistort_rpts,results);

        /* results is a 4*N matrix*/
        distance = 0;
        for(int i = 0;i<results.cols;++i)

        {
            float D = results.at<float>(3,i);

            /* 通过归一化，得到第一个相机坐标系下各点坐标*/

            Point3f p_3d(results.at<float>(0,i)/D,results.at<float>(1,i)/D,results.at<float>(2,i)/D);
            distance += p_3d.z;
            
        }   
        distance /= results.cols;
        cout << distance << "\n" << endl;
        putText(canvas, to_string(distance) ,roi.tl() ,FONT_HERSHEY_PLAIN,
        5, Scalar(0,0,255), 3, 4);
        // writer << canvas;
        imshow("out", canvas);
        imwrite("test_three2.jpg", canvas);
        cv::waitKey(50);
        break;
    }
    // writer.release();
    return 0;
}