//
// Created by tucker on 5/21/20.
//

#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;
int main( int argc, char* argv[] )
{
    CommandLineParser parser( argc, argv, "{@input | box.png | input image}" );
    Mat src = imread( "../imgs/raw_r_marked.png", IMREAD_GRAYSCALE );
    if ( src.empty() )
    {
        cout << "Could not open or find the image!\n" << endl;
        cout << "Usage: " << argv[0] << " <Input image>" << endl;
        return -1;
    }

    Mat detected_edges;
    GaussianBlur( src, detected_edges, Size(3,3), 1);
    Canny( detected_edges, detected_edges, 8, 24, 3 );



    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create( minHessian );
    std::vector<KeyPoint> keypoints;
    detector->detect( detected_edges, keypoints );
    //-- Draw keypoints
    Mat img_keypoints;
    drawKeypoints( detected_edges, keypoints, img_keypoints );
    //-- Show detected (drawn) keypoints
    imshow("SURF Keypoints", img_keypoints );
    waitKey();
    return 0;
}