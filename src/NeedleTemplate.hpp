#ifndef NEEDLE_TEMPLATE_H
#define NEEDLE_TEMPLATE_H

#include <opencv2/core.hpp>
#include <string>
#include "TemplateMatch.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

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
        origin_offset_x(52), origin_offset_y(4), initialRect(287,205,105,56), templ(cv::imread("../imgs/raw/0_l_c_fatty.png", cv::IMREAD_COLOR)) 
    {
        cout << "Initializing needle template with default constructor. This may not be the template you want" << endl;
    }

    //TODO: Pack parameters into a MatchingParameters struct or something
    TemplateMatch matchOverScaleAndRotation(const cv::Mat& img, double min_scale, double max_scale, double scale_increment, double min_rotation, double max_rotation, double rotation_increment);
};

//TODO: Move this to a different file? Probably doesn't make sense here
void match(const cv::Mat &img, const cv::Mat& templ, TemplateMatch *bestMatch, double angle, double scale);
void rotate(double degrees, const cv::Mat &src, cv::Mat &dst);


#endif