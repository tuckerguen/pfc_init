//
// Created by tucker on 5/21/20.
//

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <iostream>
using namespace cv;
using namespace cv::xfeatures2d;

Mat src, src_gray;
Mat dst, detected_edges;
int lowThreshold = 0;
const int max_lowThreshold = 100;
const int ratio = 3;
const int kernel_size = 3;
const char* window_name = "Edge Map";

int minHessian = 400;


static void CannyThreshold(int, void*)
{
    // blur( src_gray, detected_edges, Size(3,3) );
    GaussianBlur( src_gray, detected_edges, Size(3,3), 1);
    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );


    //Do SURF feature detection
    Ptr<SURF> detector = SURF::create( minHessian );
    std::vector<KeyPoint> keypoints;
    detector->detect( detected_edges, keypoints );
    //-- Draw keypoints
    Mat img_keypoints;
    drawKeypoints( detected_edges, keypoints, img_keypoints );
    imshow( window_name, img_keypoints );
}

int main()
{
    src = imread( "../imgs/raw_r_marked.png", IMREAD_COLOR ); //00 Load an image
    if( src.empty() )
    {
        std::cout << "Could not open or find the image!\n" << std::endl;
        return -1;
    }
    dst.create( src.size(), src.type() );
    cvtColor( src, src_gray, COLOR_BGR2GRAY );
    namedWindow( window_name, WINDOW_AUTOSIZE );
    createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

    CannyThreshold(0, 0);
    waitKey(0);
    return 0;
}