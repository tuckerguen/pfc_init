#ifndef NEEDLE_IMAGE
#define NEEDLE_IMAGE

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

#endif