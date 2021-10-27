#include <iostream> 
#include <opencv2/opencv.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/calib3d/calib3d.hpp> 
#include <opencv2/core/core.hpp> 
using namespace cv; 
bool findCorners(const cv::Mat &src, std::vector<cv::Point2f> &corners) 
{ 
    std::vector<cv::Point2f> pts; 
    corners.clear(); 
    bool flag = cv::findChessboardCorners(src, {9, 6}, pts); 
    if (!flag) 
        return false; 
    corners.push_back(pts[0]); 
    corners.push_back(pts[9 - 1]); 
    corners.push_back(pts[pts.size() - 9]); 
    corners.push_back(pts[pts.size() - 1]); 
    return true; 
}
int main() 
{ 
    cv::Mat src; 
    cv::Mat camera_matrix; 
    cv::Mat distort_matrix; 
    cv::FileStorage reader("/home/nvidia/gitee/rb_vm_ubuntu/hw3/2/data/parameter.txt", cv::FileStorage:: READ);
    
    reader["C"] >> camera_matrix; reader["D"] >> distort_matrix;
    std::string path0 = "/home/nvidia/gitee/rb_vm_ubuntu/hw3/2/data/chess/";
    for (int i = 0; i <= 40; i++) 
    { 
        src = imread(path0 + std::__cxx11::to_string(i).append(".jpg")); 
        std::vector<cv::Point2f> corners; 
        bool flag = findCorners(src, corners); 
        imshow("Opencv Demo", src); 
        cv::waitKey(100); 
        if (flag == false) 
        { 
            std::cout << "failed to find all corners\n"; 
            continue; 
        }
        std::vector<cv::Point3f> dst; 
        dst.push_back({0, 0, 0}); 
        dst.push_back({8 * 1, 0, 0}); 
        dst.push_back({0, 5 * 1, 0}); 
        dst.push_back({8 * 1, 5 * 1, 0}); 
        cv::Mat rvec, tvec; 
        cv::solvePnP(dst, corners, camera_matrix, distort_matrix, rvec, tvec); 
        std::cout << "t:" << std::endl << -tvec << std::endl << std::endl;
        cv::Mat drawer; drawer = src.clone(); 
        for (int j = 0; j < 4; j++) 
            cv::circle(drawer, corners[j], 2, {0, 255, 0}, 2); 
        cv::imshow("corners", drawer); 
        cv::waitKey(5); 
    }
    return 0; 
}
