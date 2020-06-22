#ifndef TEMPLATE_MATCH
#define TEMPLATE_MATCH

#include "TemplateMatch.hpp"
#include <iostream>

using namespace std;

void TemplateMatch::printMatchSummary(string name)
{
    cout << "----------------------------------------------------------------------" << endl;
    cout << "Match Summary: " + name << endl;
    cout << "----------------------------------------------------------------------" << endl;
    cout << "Pixel Location: (" << rect.x << ", " << rect.y << ")" << endl;
    cout << "Size: " << "width: " << rect.width << ", height: " << rect.height << endl;
    cout << "Yaw: degrees = " << getAngleDegrees() << ", radians = " << getAngleRadians() << endl;
    cout << "Scale: " << scale << endl;
}

void TemplateMatch::drawOnImage(cv::Mat& img, const cv::Scalar& color)
{
    rectangle(img, rect, color, pfc::line_weight, pfc::line_type, pfc::shift);
}

#endif