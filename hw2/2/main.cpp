#include "CameraApi.h" //相机SDK的API头文件

#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/opencv.hpp>


// 多线程内容
#include <thread>
#include <list>
#include <mutex>

using namespace cv;


// 标定程序所需的变量及函数 ------ start
const int board_w = 9, board_h = 6;
const int board_n = board_w * board_h;
Size board_size(9, 6);
Mat gray_img, drawn_img;
std::vector< Point2f > point_pix_pos_buf;
std::vector< std::vector<Point2f> > point_pix_pos;

int found, successes = 0;
const int min_successes = 5; // 最少成功次数
bool ACK_stop = false; // 手动确认停止采集更多图片 PS： 如果没有达到最少成功样本，子线程不会停
Size img_size;

int cnt = 0;
int k = 0, n = 0;
std::list<Mat> L_frames;
Mat src0;
std::mutex g_mutex;
int key_down = 0;
void consumer_action_partB();
// 标定程序所需的变量及函数 ------ end

// Logitech C170 使用 ------ start
cv::VideoCapture capture;
// Logitech C170 使用 ------ end

void consumer()
{
    while (!ACK_stop || successes < min_successes)
    {
        // ACK_stop = false; // 首先重置“确认停止”键
        if (L_frames.size() > 0)
        {
            g_mutex.lock();
            src0 = L_frames.front();
            L_frames.pop_front();
            g_mutex.unlock();
            if (!cnt)
            {
                img_size.width = src0.cols;
                img_size.height = src0.rows;
            }
            found = findChessboardCorners( src0, board_size, point_pix_pos_buf);
            if ( found && point_pix_pos_buf.size() == board_n )
            {
                successes++;
                cvtColor( src0, gray_img, COLOR_BGR2GRAY );
                find4QuadCornerSubpix(gray_img, point_pix_pos_buf, Size(5, 5));
                point_pix_pos.push_back( point_pix_pos_buf );
                drawn_img = src0.clone();
                drawChessboardCorners( drawn_img, board_size, point_pix_pos_buf, found );
                imshow("corners", drawn_img);
                waitKey(50);
            }else 
                std::cout << "\t but failed to found all chess board corners in this image" << std::endl;
            point_pix_pos_buf.clear();
            cnt++;

        }else{
            // std::cout << "consumer waiting for new frame\n" << std::endl;
            NULL;
        }
    }
    consumer_action_partB();
}

void consumer_action_partB()
{
    std::cout << successes << " useful chess boards" << std::endl;

    Size square_size(10,10);
    std::vector< std::vector< Point3f > > point_grid_pos;
    std::vector< Point3f > point_grid_pos_buf;
    std::vector< int > point_count;
    Mat camera_matrix( 3, 3, CV_32FC1, Scalar::all( 0 ));
    Mat dist_coeffs(1, 5, CV_32FC1, Scalar::all( 0 ));
    std::vector< Mat > rvecs;
    std::vector< Mat > tvecs;

    for (int i = 0; i < successes; i++)
    {
        for ( int j = 0; j < board_h; j++)
        {
            for (int k = 0; k < board_w; k++)
            {
                Point3f pt;
                pt.x = k * square_size.width;
                pt.y = j * square_size.height;
                pt.z = 0;
                point_grid_pos_buf.push_back( pt );
            }
        }
        point_grid_pos.push_back( point_grid_pos_buf);
        point_grid_pos_buf.clear();
        point_count.push_back( board_h * board_w );
    }
    std::cout << "work until here\n" << std::endl;
    std::cout << calibrateCamera( point_grid_pos, point_pix_pos, img_size,
    camera_matrix, dist_coeffs, rvecs, tvecs) << std::endl;
    std::cout << camera_matrix << std::endl << dist_coeffs << std::endl;
}

int main()
{   
    capture.open(0);
    // 开启子线程
    std::thread t1(consumer);
    // 初始化视频窗口
    namedWindow("Opencv Demo", 0);
    namedWindow("corners", 0);
    cv::Mat matImage;
    // 显示视频
    while(capture.read(matImage)){
        imshow("Opencv Demo", matImage);

        key_down = waitKey(5);
        
        if (key_down == 49)
        {   
            std::cout << key_down <<std::endl;
            g_mutex.lock();
            L_frames.push_back(matImage);
            g_mutex.unlock();
            std::cout << "采集了1张图片\n" << std::endl;
        }else if(key_down == 50){
            std::cout << key_down <<std::endl;
            g_mutex.lock();
            ACK_stop = true;
            g_mutex.unlock();
        }
    }
    destroyAllWindows;
    t1.join();
    return 0;
}

