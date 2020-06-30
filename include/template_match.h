#ifndef TEMPLATE_MATCH_H
#define TEMPLATE_MATCH_H

#include <opencv2/imgproc.hpp>
#include "pfc_initializer_constants.h"

//A template match
class TemplateMatch 
{
public:
    /**
     * @brief Angle (degrees) of rotation of the matching template (relative to original)
     */
    double angle;

    /**
     * @brief Match quality value returned by cv::matchTemplate()
     */    
    double score;

    /**
     * @brief Scale of matching template (relative to original)
     */
    double scale;

    /**
     * @brief Location of match and template dimensions at match
     */
    cv::Rect rect;

    /**
     * @brief Result image returned by cv::matchTemplate()
     */
    cv::Mat result; 

    /**
     * @brief Template used for match
     */
    cv::Mat templ;

    /**
     * @brief 2x1 point represents (x,y) of needle origin in matched template
     */
    cv::Mat needle_origin;

    /**
     * @brief Constructor
     * 
     * @param angle Angle (degrees) template was rotated at match
     * @param score Match quality value given to match by cv::matchTemplate()
     * @param scale Scale template was scaled by at match
     */
    TemplateMatch(double angle, double score, double scale) : 
        angle(angle), score(score), scale(scale), rect(0,0,0,0), result(), templ() 
    {} 

    /**
     * @brief Constructor
     * 
     * @param angle  Angle (degrees) template was rotated at match
     * @param score  Match quality value given to match by cv::matchTemplate()
     * @param scale  Scale template was scaled by at match
     * @param rect   Rectangle representing bounds of template in matched image
     * @param result Result image returned by cv::matchTemplate()
     * @param templ  Template used in match
     */
    TemplateMatch(double angle, double score, double scale, cv::Rect2i rect, cv::Mat result, cv::Mat templ) : 
        angle(angle), score(score), scale(scale), rect(rect), result(result), templ(templ) 
    {} 

    /**
     * @brief Default constructor (angle=0, score=-DBL_MAX, scale=1, rect=(0,0,0,0))
     */
    TemplateMatch() : 
        angle(0), score(-DBL_MAX), scale(1), rect(0,0,0,0), result() 
    {} 

    /**
     * @brief Returns the match angle in degrees
     */
    double getAngleDegrees() { return angle; }
    
    /**
     * @brief Returns the match angle in radians
     */
    double getAngleRadians() { return angle * pfc::deg2rad; }

    /**
     * @brief Format and print details of the match
     * 
     * @param name Title to be printed with the match (probably "right" or "left")
     */
    void printMatchSummary(const std::string& name);

    /**
     * @brief Draws the match bounding rectangle on the image
     * 
     * @param img Image to draw the match onto
     * @param color Color of the rectangle
     */
    void drawOnImage(cv::Mat& img, const cv::Scalar& color);
};

#endif