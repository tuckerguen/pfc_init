//
// Created by tucker on 5/19/20.
//
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

Mat img, templ, result;
char* image_window = "Source Image";
char* result_window = "Result Window";
char* template_window = "Template";

int match_method;
int max_Trackbar = 5;

void Match(int, void*);
void RotTemplate(double angle, const Mat &src, Mat &dst);

int main(int argc, char** argv){
    namedWindow( image_window, WINDOW_AUTOSIZE );
    namedWindow( result_window, WINDOW_AUTOSIZE );
    namedWindow(template_window,  WINDOW_AUTOSIZE);

    img = imread("imgs/vessel_l_c.png", IMREAD_COLOR);
    Mat iTempl;
    iTempl = imread("imgs/vessel_l_a.png", IMREAD_COLOR);
    Rect r(280, 200, 115, 65);
    iTempl = iTempl(r);

    for(int i = 0; i < 360; ++i){
        RotTemplate(-90, iTempl, templ);
        Match(0, 0);
    }
    



//    char* trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
//    createTrackbar( trackbar_label, image_window, &match_method, max_Trackbar, Match );



    imshow(template_window, templ);

//    namedWindow("img", WINDOW_AUTOSIZE);
//    imshow("img", img);
//    namedWindow("templ", WINDOW_AUTOSIZE);
//    imshow("templ", templ);
//    namedWindow("result", WINDOW_AUTOSIZE);
//    imshow("result", result);
    waitKey();

    return 0;
}

void RotTemplate(double angle, const Mat &src, Mat &dst){
    // get rotation matrix for rotating the image around its center in pixel coordinates
    Point2f center((src.cols-1)/2.0, (src.rows-1)/2.0);
    Mat rot = getRotationMatrix2D(center, angle, 1.0);
    // determine bounding rectangle, center not relevant
    Rect2f bbox = RotatedRect(Point2f(), src.size(), angle).boundingRect2f();
    // adjust transformation matrix
    rot.at<double>(0,2) += bbox.width/2.0 - src.cols/2.0;
    rot.at<double>(1,2) += bbox.height/2.0 - src.rows/2.0;

    warpAffine(src, dst, rot, bbox.size());
}

void Match( int, void*){
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
    rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(255), 2, 8, 0 );
    rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );

    imshow( image_window, img_display );
    imshow( result_window, result );

    return;
}