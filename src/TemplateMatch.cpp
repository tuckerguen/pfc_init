#include "TemplateMatch.hpp"
#include <iostream>

using namespace std;

void TemplateMatch::printMatchSummary()
{
    cout << "Pixel Location: (" << rect.x << ", " << rect.y << ")" << endl;
    cout << "Size: " << "width: " << rect.width << ", height: " << rect.height << endl;
    cout << "Yaw: degrees = " << getAngleDegrees() << ", radians = " << getAngleRadians() << endl;
    cout << "Scale: " << scale << endl;
}

void TemplateMatch::drawOnImage(cv::Mat& img, const cv::Scalar& color)
{
    rectangle(img, rect, color, line_weight, line_type, shift);
}