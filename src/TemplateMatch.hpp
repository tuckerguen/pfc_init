#ifndef TEMPLATE_MATCH_H
#define TEMPLATE_MATCH_H

#include <opencv2/imgproc.hpp>
#include "PfcInitConstants.hpp"

//A template match
class TemplateMatch 
{
public:
    double angle; //angle matched at    
    double score; //max matching value assigned by opencv templatematch()
    double scale; //scale matched at
    cv::Rect rect; //location and size of match
    cv::Mat result; //result image from opencv templatematch()
    cv::Mat templ;
    cv::Mat needle_origin;

    TemplateMatch(double angle, double score, double scale) : 
        angle(angle), score(score), scale(scale), rect(0,0,0,0), result(), templ() 
    {} 

    TemplateMatch() : 
        angle(0), score(-DBL_MAX), scale(1), rect(0,0,0,0), result() 
    {} 

    double getAngleDegrees() { return angle; }
    double getAngleRadians() { return angle * pfc::deg2rad; }

    void printMatchSummary(std::string name);    
    void drawOnImage(cv::Mat& img, const cv::Scalar& color);
};

#endif