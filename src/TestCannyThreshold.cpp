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
int high_threshold = 115;
const int max_lowThreshold = 255;
const int ratio = 3;
const int kernel_size = 3;
const char* window_name = "Edge Map";


int low_H = 0, low_S = 0, low_V = 0;
int high_H = 4, high_S = 0, high_V = 140;

int minHessian = 400;


static void CannyThreshold(int, void*)
{
    // blur( src_gray, detected_edges, Size(3,3) );
    // inRange(src, Scalar(lowThreshold, lowThreshold, lowThreshold), Scalar(high_threshold, high_threshold, high_threshold), colorFiltered);
  
    // GaussianBlur( colorFiltered, detected_edges, Size(3,3), 0);
    Mat img_HSV;
    cvtColor(src, img_HSV, COLOR_BGR2HSV);
    inRange(img_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), detected_edges);
    GaussianBlur( detected_edges, detected_edges, Size(3,3), 0);
    Canny( detected_edges, detected_edges, 8, 8*3, kernel_size );
    Rect r(168, 92, 58, 35);
    detected_edges = detected_edges(r);

    //Do SURF feature detection
    // Ptr<SURF> detector = SURF::create( minHessian );
    // std::vector<KeyPoint> keypoints;
    // detector->detect( detected_edges, keypoints );
    // //-- Draw keypoints
    // Mat img_keypoints;
    // drawKeypoints( detected_edges, keypoints, img_keypoints );
    imshow( window_name, detected_edges );
}

int main()
{
    src = imread( "../imgs/raw_l_b.png", IMREAD_COLOR ); //00 Load an image
    if( src.empty() )
    {
        std::cout << "Could not open or find the image!\n" << std::endl;
        return -1;
    }
    dst.create( src.size(), src.type() );
    // cvtColor( src, src_gray, COLOR_BGR2GRAY );
    namedWindow( window_name, WINDOW_AUTOSIZE );
    // createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );
    // createTrackbar( "Max Threshold:", window_name, &high_threshold, max_lowThreshold, CannyThreshold );
    createTrackbar("Low H", window_name, &low_H, 180, CannyThreshold);
    createTrackbar("High H", window_name, &high_H, 170, CannyThreshold);
    createTrackbar("Low S", window_name, &low_S, 255, CannyThreshold);
    createTrackbar("High S", window_name, &high_S, 255, CannyThreshold);
    createTrackbar("Low V", window_name, &low_V, 255, CannyThreshold);
    createTrackbar("High V", window_name, &high_V, 255, CannyThreshold);

    CannyThreshold(0, 0);
    waitKey(0);
    return 0;
}