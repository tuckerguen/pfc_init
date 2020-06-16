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
    cv::inRange(img_HSV, cv::Scalar(low_h, low_s, low_v), cv::Scalar(high_h, high_s, high_v), image);
}