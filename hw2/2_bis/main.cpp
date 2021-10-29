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

unsigned char           * g_pRgbBuffer;     //处理后数据缓存区

// 标定程序所需的变量及函数 ------ start
const int board_w = 11, board_h = 8;
const int board_n = board_w * board_h;
Size board_size(11, 8);
Mat gray_img, drawn_img;
std::vector< Point2f > point_pix_pos_buf;
std::vector< std::vector<Point2f> > point_pix_pos;

int found, successes = 0;
const int min_successes = 10; // 最少成功次数
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
            if (! cnt)
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

    int                     iCameraCounts = 1;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList;
    int                     hCamera;
    tSdkCameraCapbility     tCapability;      //设备描述信息
    tSdkFrameHead           sFrameInfo;
    BYTE*			        pbyBuffer;
    int                     iDisplayFrames = 10000;
    // IplImage *iplImage = NULL;
    int                     channel=3;

    CameraSdkInit(1);

    //枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
    #if 0
	printf("state = %d\n", iStatus);

	printf("count = %d\n", iCameraCounts);
    #endif
    //没有连接设备
    if(iCameraCounts==0){
        return -1;
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);

    //初始化失败
	printf("state = %d\n", iStatus);
    if(iStatus!=CAMERA_STATUS_SUCCESS){
        return -1;
    }

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera,&tCapability);

    //
    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    //g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(hCamera);

    /*其他的相机参数设置
    例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
         CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
         CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
         更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
    */

    if(tCapability.sIspCapacity.bMonoSensor){
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }

    // 开启子线程
    std::thread t1(consumer);
    // 初始化视频窗口
    namedWindow("Opencv Demo", 0);
    namedWindow("corners", 0);
    //循环显示1000帧图像
    while(iDisplayFrames--)
    {
        if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
		{
		    CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);
		    
		    cv::Mat matImage(
					Size(sFrameInfo.iWidth,sFrameInfo.iHeight), 
					sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
					g_pRgbBuffer
					);
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
            
            //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
			//否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
			CameraReleaseImageBuffer(hCamera,pbyBuffer);

		}
    }
    destroyAllWindows;
    CameraUnInit(hCamera);
    //注意，现反初始化后再free
    free(g_pRgbBuffer);
    t1.join();

    return 0;
}

