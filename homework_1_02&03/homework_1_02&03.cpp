#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <iostream> 
#include <vector> 
#include <assert.h> 

using namespace std;
using namespace cv;

void dfs(cv::Mat &drawer, 
    const std::vector< std::vector<cv::Point> > &contours, 
    const std::vector< cv::Vec4i > &hierachy, 
    const int &id, 
    const int &depth) { 
    if (id == -1) 
        return; 
    static cv::Scalar COLOR_LIST[3] = { {220, 20, 20}, {20, 220, 20}, {20, 20, 220} }; 
    cv::drawContours(drawer, contours, id, COLOR_LIST[depth % 3], 1); 
    for (int i = hierachy[id][2]; i + 1; i = hierachy[i][0]) { 
        dfs(drawer, contours, hierachy, i, depth + 1); // 向内部的子轮廓 递归 
    } 
}

#if 0

cv::Point2f no_child(const std::vector< std::vector<cv::Point> > &contours, int id){
    std::vector<cv::Point> contour = contours[id];
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contour, center,radius);
    return center;
}


void unique_child(const std::vector< std::vector<cv::Point> > &contours, int id){

}


int count_children(const std::vector< cv::Vec4i > &hierachy, const int id){
    int n = 0;
    for (int i = id; i + 1; i = hierachy[i][2]){
        n+=1;
    } 
    printf("the layer %d, has %d child/children.\n", id, n);
    return n;
}

void census(const std::vector< std::vector<cv::Point> > &contours, const std::vector< cv::Vec4i > &hierachy, const int id){
    cv::Point2f center;
    // 遍历最外层轮廓
    for (int i = 0; i + 1; i = hierachy[i][0]) {

    }
    
    if (hierachy[id][2] == -1){
        center = no_child(contours, id);
    }
    else{

    }
}
#endif

double Point2PointDist(const cv::Point2f& a, const cv::Point2f& b)
{
    double res = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    return res;
}

cv::Mat find_draw(const cv::Mat &src) { 
    cv::Mat hsv;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV); // 将颜色空间从BGR转为HSV 
    cv::Mat hsv_part1, hsv_part2; 
    cv::inRange(hsv, cv::Scalar(0, 43, 46), cv::Scalar(25, 255, 255), hsv_part1); 
    cv::inRange(hsv, cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255), hsv_part2); // 提取红色和橙色 
    cv::Mat ones_mat = cv::Mat::ones(cv::Size(src.cols, src.rows), CV_8UC1); 
    cv::Mat hsv_result = 255 * (ones_mat - (ones_mat - hsv_part1 / 255).mul(ones_mat - hsv_part2 / 255)); // 对hsv_part1的结果和hsv_part2的结果取并集 
    // //定义核
	// cv::Mat element_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4 , 4));  
    // cv::Mat element_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));  
	// //进行形态学操作
    // cv::Mat morpho;
	// // cv::morphologyEx(hsv_result, morpho, cv::MORPH_OPEN, element_erode);
    // cv::erode(hsv_result, morpho,element_erode);
    // cv::dilate(morpho, morpho,element_dilate);

    std::vector< std::vector<cv::Point> > contour; 
    std::vector< cv::Vec4i > hierachy; 
    cv::findContours( hsv_result, contour, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE); 
    // get_depth(hierachy);
    // cv::Mat drawer = cv::Mat::zeros(cv::Size(src.cols, src.rows), CV_8UC3); 
    cv::Mat drawer = src;
    double s_min = 100000000000; 
    int center_id = 0;
    int big_ids[5] = {-1,-1,-1,-1,-1};
    int flag = 0;
    // 遍历最外层， 先找找中心点, 以及大区域的标号
    for (int i = 0; i + 1; i = hierachy[i][0]){
        double s = contourArea(contour[i]);
        if (s < s_min && s > 150){
            s_min = s;
            center_id = i;
        }
        if (s > 2400){
            big_ids[flag] = i;
            // cv::drawContours(drawer, contour,i, Scalar(0,255,0), 2);
            flag += 1;
            flag = flag % 5;
        } 
    }
    float radius;
    cv::Point2f center;
    cv::minEnclosingCircle(contour[center_id], center, radius);
    // cv::minEnclosingCircle(Mat(contour[center_id]), center, radius);
    cv::circle(drawer, Point(center), static_cast<int>(radius), Scalar(255,0,255), 5);
    
    // 再找 图报面积小于最小外接举行
    for (int i = 0; i < 5; i++){
        int id = big_ids[i];
        if (id != -1){
            
            // cv::drawContours(drawer, contour,id, Scalar(0,255,255), 2);
            
            // 返回自身距离中心最近的点
            cv::RotatedRect box = cv::minAreaRect(contour[id]);
            cv::Point2f rect[4];
            box.points(rect);
            Point2f far_mid_point;
            double dis_max = 10000000000;
            for (int k = 0; k < 4; k++){
                // 中点：
                cv::Point2f mid  = (rect[k] + rect[(k+1)%4])/2;
                // cv::Point2f mid2  = (rect[k+1] + rect[(k+2)%4])/2;
                double dis = Point2PointDist(mid, center);
                if (dis < dis_max){
                    dis_max = dis;
                    far_mid_point = mid;
                }
                // line(drawer, mid, mid2, Scalar(0, 0, 255), 2, 8);
            }


            // 返回子轮廓距离中心最远的那一个
            int far_child_id = -1;
            double far_child_distance = 100000000000;
            int shot = 0;
            for (int j = hierachy[id][2]; j + 1; j= hierachy[j][0]) { 
                
                if(j != -1){
                    shot += 1;
                    double distance = cv::pointPolygonTest(contour[j], center, true);
                    if (distance < far_child_distance){
                        
                        far_child_distance = distance;
                        far_child_id = j;
                    }
                }
            }
            // far_child_id = 3;
            if (far_child_id != -1 && shot > 1){
                // cv::drawContours(drawer, contour, far_child_id, Scalar(0,255,0), 2);
                // vector<Point> contour_new;
                // contour_new.assign(contour[far_child_id].begin(), contour[far_child_id].end());
                
                vector<vector<cv::Point>> convex(1);
                vector<cv::Point> contourmax = contour[far_child_id];
                contourmax.push_back(far_mid_point);
                cv::convexHull(contourmax, convex[0]);
                cv::drawContours(drawer, convex, -1, Scalar(0,255,0), 2);
                // cv::drawContours(drawer, convex, 0, Scalar(0,255,0), 2);
                // cv::convexHull(contour[far_child_id].push_back(far_mid_point), convex, false)
                // cv::drawContours(drawer, convex, 0, Scalar(0,255,0), 2);

            }
            else if(far_child_id != -1 && shot <= 1){
                // cv::drawContours(drawer, contour, far_child_id, Scalar(0,255,255), 2);
                // vector<Point> contour_new;
                // contour_new.assign(contour[far_child_id].begin(), contour[far_child_id].end());
                // contour_new.push_back(far_mid_point);
                vector<vector<cv::Point>> convex(1);
                vector<cv::Point> contourmax = contour[far_child_id];
                contourmax.push_back(far_mid_point);
                // [far_child_id].push_back(far_mid_point)
                cv::convexHull(contourmax, convex[0]);
                cv::drawContours(drawer, convex, -1, Scalar(0,255,255), 2);
            }

        }
    }
    return drawer; 
}


int main() { 

    // // 保存
    // cv::VideoCapture capture("/home/nvidia/Robomaster/videos/video2.mp4"); 
    // cv::Mat src;
    // capture >> src; 
    // assert(!src.empty()); 
    // cv::VideoWriter writer("/home/nvidia/Downloads/2_output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(src.cols, src.rows), true); // 需要注意的是，这里由于输出灰度图片，参数isColor为false，如果输出彩色图片 则应为true 
    // while (true) { 
    //     // cv::Mat output;
    //     // assert(src.channels() == 3); // 检测是否为三通道彩色图片 
    //     cv::Mat output = find_draw(src);
    //     writer << output; 
    //     capture >> src; 
    //     if (src.empty()) 
    //         break; 
    //     // cv::imshow("src", output);
    //     // // cv::imshow("src", src); // 这里是显示图片的语句，第一个参数为显示 窗口的名字，第二个参数为需要显示的图片 
    //     // cv::waitKey(50); 
    // }
    // writer.release(); 

    // 即看
    cv::VideoCapture capture("/home/nvidia/Robomaster/videos/video2.mp4"); 
    cv::Mat src; 
    while (capture.read(src)) {
        cv::Mat output = find_draw(src); 
        cv::imshow("src", output);
        // 这里是显示图片的语句，第一个参数为显示 窗口的名字，第二个参数为需要显示的图片 
        cv::waitKey(50); 
    }
    return 0;

}
