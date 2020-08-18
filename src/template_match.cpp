#include <iostream>
#include "template_match.h"

using namespace std;

// Format and print details of the match
void TemplateMatch::printMatchSummary(const string& name)
{
    // Compact Print
    printf("%s: [BB:(%d,%d,%d,%d), rot=%f, scl=%f]\n", name.c_str(), rect.x, rect.y, rect.width, rect.height, getAngleDegrees(), scale);
}

// Draws the match bounding rectangle on the image
void TemplateMatch::drawOnImage(cv::Mat& img, const cv::Scalar& color)
{
    rectangle(img, rect, color, pfc::line_weight, pfc::line_type, pfc::shift);
}