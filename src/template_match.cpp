#include <iostream>
#include "template_match.h"

using namespace std;

// Format and print details of the match
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

// Draws the match bounding rectangle on the image
void TemplateMatch::drawOnImage(cv::Mat& img, const cv::Scalar& color)
{
    rectangle(img, rect, color, pfc::line_weight, pfc::line_type, pfc::shift);
}