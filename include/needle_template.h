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
class NeedleTemplate
{
public:
    /**
     * @brief Pixel location of the needle's 3D origin
     */
    cv::Point2d origin;

    /**
     * @brief the image of the template
     */
    cv::Mat image;

    /**
     * @brief Max/min rotation and scale parameters to speed up testing (temporary implementation, only for testing)
     */
    pfc::match_params params;

    int resolution;

    bool left;


    void GenerateTemplate(float z, float a, float b, float y);

    /**
     * @brief Needle template constructor
     * 
     */
    NeedleTemplate(pfc::match_params params, double z, double a, double b, double y, int resolution, bool left)
    : params(params), resolution(resolution), left(left)
    {
        GenerateTemplate(z, a, b, y);
    }

    NeedleTemplate(pfc::match_params params, bool left)
    : params(params), resolution(params.resolution), left(left)
    {
        GenerateTemplate(params.min_z, params.min_yaw, params.pitch_range.start, params.roll_range.start);
    }

    // /**
    //  * @brief Default constructor
    //  */
    // NeedleTemplate()
    // {
    //     GenerateTemplate(0.15, 0, 0, 0, 10, true);
    // }

    /**
     * @brief Deconstructor
     */
    ~NeedleTemplate()
    {};
};

cv::Point2d CalcUVPoint(const cv::Mat& p, const cv::Mat& transform, const cv::Mat& projection);


#endif