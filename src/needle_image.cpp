#include "needle_image.h"
#include "pfc_initializer_constants.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace std;

// constructor
NeedleImage::NeedleImage(string path)
{
    //Load needle image at path
    raw = cv::imread(path, cv::IMREAD_COLOR);
    
    if (!raw.data)
    {
        cerr << "Loading image failed" << endl;
        exit(0);
    }

    //preprocess
    filterRaw();    
}

//Filters the image for needle
void NeedleImage::filterRaw()
{
    // temp mat
    cv::Mat img_HSV;
    // convert to HSV
    cv::cvtColor(raw, img_HSV, cv::COLOR_BGR2HSV);
    // filter by HSV values 
    cv::inRange(img_HSV, cv::Scalar(pfc::low_h, pfc::low_s, pfc::low_v), cv::Scalar(pfc::high_h, pfc::high_s, pfc::high_v), image);
}

//TODO: Make this a member function
//rotate an image by angle degrees
void rotate(const cv::Mat &src, cv::Mat &dst, double angle)
{
    // get center of original img
    cv::Point2d center((src.cols - 1) / 2.0, (src.rows - 1) / 2.0);

    // get rotation matrix for rotating the image around its center in pixel coordinates
    cv::Mat rot = getRotationMatrix2D(center, angle, 1.0);
    // calculate dimensions of rotated image, center not relevant
    cv::Rect2d bbox = cv::RotatedRect(cv::Point2d(), src.size(), angle).boundingRect2f();

    // apply translation to rotation matrix to shift center 
    rot.at<double>(0, 2) += bbox.width / 2.0 - src.cols / 2.0;

    rot.at<double>(1, 2) += bbox.height / 2.0 - src.rows / 2.0;
    //apply matrix transformation
    warpAffine(src, dst, rot, bbox.size());
}