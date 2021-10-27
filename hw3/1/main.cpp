#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
using namespace cv;

void doPerspectiveTransform( Mat input, Mat& output ) {
    std::vector<Point2f> srcQuad( 4 ), dstQuad( 4 );
    output = input.clone();

    srcQuad[0].x = 602, srcQuad[1].x = 605, srcQuad[2].x = 769, srcQuad[3].x = 763;
    srcQuad[0].y = 332, srcQuad[1].y = 384, srcQuad[2].y = 386, srcQuad[3].y = 332;

    dstQuad[0].x = 0, dstQuad[1].x = 0, dstQuad[2].x = 320, dstQuad[3].x = 320;
    dstQuad[0].y = 0, dstQuad[1].y = 120, dstQuad[2].y = 120, dstQuad[3].y = 0;

    Mat warp_matrix = getPerspectiveTransform( srcQuad, dstQuad );

    warpPerspective( input, output, warp_matrix, Size( 320, 120 ) );
}


int main() {
    Mat img = imread("../../../datapack/car.jpg");
    Mat result;

    doPerspectiveTransform( img, result );

    namedWindow( "input" );
    namedWindow( "output" );

    imshow( "input", img );
    imshow( "output", result );

    waitKey( 0 );

    destroyWindow( "input" );
    destroyWindow( "output" );
    return 0;
}