//
// Created by tucker on 5/19/20.
//
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <iostream>
#include <stdio.h>
#include "NeedleMatch.hpp"

using namespace std;
using namespace cv;

string image_window = "Source Image";
string result_window = "Result Window";
string template_window = "Template";

//A template match
struct match {
    double maxVal; //max matching value assigned by opencv templatematch()
    double angle; //angle matched at
    double scale; //scale matched at
    Point maxLoc; //location of match
    Mat templ; //template used to match
};

int match_method;
int max_Trackbar = 5;

int main(int argc, char** argv){
    namedWindow( image_window, WINDOW_AUTOSIZE );
    namedWindow( result_window, WINDOW_AUTOSIZE );
    namedWindow( template_window,  WINDOW_AUTOSIZE );

    Mat raw_img, raw_templ, img_hsv, templ_hsv;
    Mat *img, *templ, result, *iTempl;

    // match match bestMatch = {
    //     -DBL_MAX,
    //     0.0,
    //     0.0,
    //     Point(0, 0),
    //     templ,
    // };
    
    size_t img_size = img.total() * img.elemSize();
    size_t templ_size = templ.total() * templ.elemSize();
    size_t iTempl_size = iTempl.total() * iTempl.elemSize();
    //Add necessary mats to unified memory
    cudaMallocManaged(&img, img_size);
    cudaMallocManaged(&templ, templ_size);
    cudaMallocManaged(&iTempl, iTempl_size);

    //pick block size
    int blockSize = 256;
    int numBlocks = (360 + blockSize - 1) / blockSize;
    
    //CPU image preproc operations
    raw_img = imread("../../imgs/raw_l_c.png", IMREAD_COLOR);
    raw_templ = imread("../../imgs/raw_l_b.png", IMREAD_COLOR);
    Rect r(168, 92, 58, 35);
    raw_templ = raw_templ(r);
    cvtColor(raw_img, img_hsv, COLOR_BGR2HSV);
    inRange(img_hsv, Scalar(0, 0, 0), Scalar(5, 0, 140), img);
    cvtColor(raw_templ, templ_hsv, COLOR_BGR2HSV);
    inRange(templ_hsv, Scalar(0, 0, 0), Scalar(5, 0, 140), iTempl);

    //Run Rotations and matching on the GPU
    run<<<numBlocks, blockSize>>>();

    imshow(template_window, templ);

    waitKey();

    return 0;
}

__global__ run(template){
    int index = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;
    for(int i = index; i < 360; i+= stride){
        RotTemplate(i);
        MatchImageToTemplate();
    }
}

__device__ void RotTemplate(double angle){
    // get rotation matrix for rotating the image around its center in pixel coordinates
    Point2f center((iTempl.cols-1)/2.0, (iTempl.rows-1)/2.0);
    Mat rot = getRotationMatrix2D(center, angle, 1.0);
    // determine bounding rectangle, center not relevant
    Rect2f bbox = RotatedRect(Point2f(), iTempl.size(), angle).boundingRect2f();
    // adjust transformation matrix
    rot.at<double>(0,2) += bbox.width/2.0 - iTempl.cols/2.0;
    rot.at<double>(1,2) += bbox.height/2.0 - iTempl.rows/2.0;

    warpAffine(iTempl, templ, rot, bbox.size());
}

__device__ void MatchImageToTemplate(){
    Mat img_display;
    img_display = img.clone();

    /// Create the result matrix
    int result_cols =  img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;

    result.create( result_rows, result_cols, CV_32FC1 );

    /// Do the Matching and Normalize
    matchTemplate( img, templ, result, TM_CCOEFF );
    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;

    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if( match_method  == TM_SQDIFF || match_method == TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
    else
    { matchLoc = maxLoc; }

    /// Show me what you got
    rectangle( img, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(255), 2, 8, 0 );
    rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );

    imshow( image_window, img );
    imshow( result_window, result );

    return;
}