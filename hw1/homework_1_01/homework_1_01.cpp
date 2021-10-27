#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/opencv.hpp>
#include <iostream> 
#include <assert.h> 
#include <vector>
using namespace std;
using namespace cv;

int main(int argc, char ** argv) { 
    Mat src = imread("/home/nvidia/Robomaster/images/apple.jpg"); // 读取图片
    assert(src.channels() == 3); // 检测是否为三通道彩色图片
    Mat hsv_img; 
    cvtColor(src, hsv_img, COLOR_BGR2HSV); // 将颜色空间从BGR转为HSV 
    
    Mat hsv_part1, hsv_part2; 
    inRange(hsv_img, Scalar(0, 167, 97), Scalar(23, 255, 255), hsv_part1); // trackbar试出来的
    inRange(hsv_img, Scalar(158, 109, 36), Scalar(180, 255, 255), hsv_part2); // trackbar试出来的
    Mat ones_mat = Mat::ones(Size(src.cols, src.rows), CV_8UC1); // 并集
    Mat hsv_result = 255 * (ones_mat - (ones_mat - hsv_part1 / 255).mul(ones_mat - hsv_part2 / 255)); // 对hsv_part1的结果和hsv_part2的结果取并集 
    
    cv::imshow("hsv", hsv_result); 

    // 先腐蚀后膨胀消除噪点
    Mat erode_dilate, element_erode, element_dilate;
    element_erode = getStructuringElement(MORPH_RECT, Size(15, 15));
    element_dilate = getStructuringElement(MORPH_RECT, Size(20, 20));
    erode(hsv_result, erode_dilate, element_erode, Point(-1, -1), 1, 0);
    dilate(erode_dilate,erode_dilate, element_dilate);

    cv::imshow("erode_dilate", erode_dilate); 

    //正外接矩形
    vector<vector<Point>> contours;
    vector<Vec4i> hierarcy;
    findContours(erode_dilate, contours, hierarcy, RETR_EXTERNAL, CHAIN_APPROX_NONE); //查找轮廓
    vector<Rect> boundRect(contours.size()); //定义外接矩形集合
    int x0=0, y0=0, w0=0, h0=0;
    for(int i=0; i<contours.size(); i++)
    {
        boundRect[i] = boundingRect((Mat)contours[i]); //查找每个轮廓的外接矩形
        // drawContours(src, contours, i, Scalar(0, 0, 255), 2, 8);  //绘制轮廓
        x0 = boundRect[i].x;  //获得第i个外接矩形的左上角的x坐标
        y0 = boundRect[i].y; //获得第i个外接矩形的左上角的y坐标
        w0 = boundRect[i].width; //获得第i个外接矩形的宽度
        h0 = boundRect[i].height; //获得第i个外接矩形的高度
        rectangle(src, Point(x0, y0), Point(x0+w0, y0+h0), Scalar(0, 255, 0), 2, 8); //绘制第i个外接矩形
    }


    imshow("result", src);
    cv::waitKey(0); 
    return 0; 
}
