#ifndef NEEDLE_TEMPLATE_H
#define NEEDLE_TEMPLATE_H

#include <opencv2/core.hpp>
#include <string>
#include "TemplateMatch.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "PfcInitConstants.hpp"

using namespace std;
//TODO: reconsider if we need a separate object for this. The use of match over scale and rotation is ugly
class NeedleTemplate
{
public:
    int origin_offset_x;
    int origin_offset_y;
    cv::Rect2i initialRect;
    cv::Mat templ;

    NeedleTemplate(const std::string& path, const cv::Rect2i& rect, int origin_offset_x, int origin_offset_y, double rotation);
    NeedleTemplate() : 
        origin_offset_x(52), origin_offset_y(4), initialRect(pfc::initial_rect), templ(cv::imread(pfc::templ_path, cv::IMREAD_COLOR)) 
    {
        cout << "Initializing needle template with default constructor. This may not be the template you want" << endl;
    }

    //TODO: Pack parameters into a MatchingParameters struct or something
    TemplateMatch matchOverScaleAndRotation(const cv::Mat& img);
};

//TODO: Move this to a different file? Probably doesn't make sense here
void match(const cv::Mat &img, const cv::Mat& templ, TemplateMatch *bestMatch, double angle, double scale);
void rotate(double degrees, const cv::Mat &src, cv::Mat &dst);


#endif