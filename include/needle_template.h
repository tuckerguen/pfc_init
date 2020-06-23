#ifndef NEEDLE_TEMPLATE_H
#define NEEDLE_TEMPLATE_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "template_match.h"
#include "pfc_initializer_constants.h"
#include "needle_image.h"

using namespace std;

/**
 * @brief Needle image used as template in template matching
 */
class NeedleTemplate : public NeedleImage
{
public:
    /**
     * @brief the pixel location of the needle's 3D origin
     */
    cv::Point2d origin;

    /**
     * @brief the initial rect used to crop the raw image to template size
     */
    cv::Rect2i initialRect;

    /**
     * @brief max/min rotation and scale parameters to speed up testing (temporary implementation, only for testing)
     */
    pfc::match_params params;

    /**
     * @brief needle template constructor
     * 
     * @param raw image path
     * @param rectangle used to crop the raw image
     * @param 3D origin of needle
     * @param initial rotation of raw image to align final template with ground truth 0 rotation
     * @param min/max rotation and scale parameters
     */
    NeedleTemplate(const std::string& path, const cv::Rect2i& rect, cv::Point2d origin, double rotation, pfc::match_params iparams);

    /**
     * @brief default constructor (origin=(52,9), rect=(287,205,105,56), img_path="../imgs/raw/0_l_c_fatty.png")
     */
    NeedleTemplate() : 
        NeedleImage(pfc::templ_path), origin(52,9), initialRect(pfc::initial_rect)
        {
            cout << "Initializing needle template with default constructor" << endl;
        }

    /**
     * @brief the deconstructor
     */
    ~NeedleTemplate()
    {};


    //TODO: Pack parameters into a MatchingParameters struct or something
    TemplateMatch matchOverScaleAndRotation(const cv::Mat& base_img);
};

//TODO: Move this to a different file? Probably doesn't make sense here
void match(const cv::Mat &img, const cv::Mat& templ, TemplateMatch *bestMatch, double angle, double scale);
void rotate(double degrees, const cv::Mat &src, cv::Mat &dst);


#endif