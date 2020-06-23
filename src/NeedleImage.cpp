#ifndef NEEDLE_IMAGE
#define NEEDLE_IMAGE

#include "NeedleImage.hpp"
#include "PfcInitConstants.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace std;

NeedleImage::NeedleImage(string path)
{
    //Load camera image to match
    cv::Mat img_HSV;
    raw = cv::imread(path, cv::IMREAD_COLOR);
    
    if (!raw.data)
    {
        cerr << "Loading image failed" << endl;
        exit(0);
    }

    // Filter by color for needle
    cv::cvtColor(raw, img_HSV, cv::COLOR_BGR2HSV);
    cv::inRange(img_HSV, cv::Scalar(pfc::low_h, pfc::low_s, pfc::low_v), cv::Scalar(pfc::high_h, pfc::high_s, pfc::high_v), image);
    cv::namedWindow("segmented image: " + path, cv::WINDOW_AUTOSIZE);
}

#endif