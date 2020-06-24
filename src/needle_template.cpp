#ifndef NEEDLE_TEMPLATE
#define NEEDLE_TEMPLATE

#include "needle_template.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <iostream>
#include "pfc_initializer_constants.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/cudaimgproc.hpp>

using namespace std;

// constructor
NeedleTemplate::NeedleTemplate(const string& path, const cv::Rect2i& rect, cv::Point2d origin, double rotation, pfc::match_params iparams)
: NeedleImage(path), origin(origin)
{
    // set params
    params = iparams;

    //Crop both raw and preprocessed template images
    raw = raw(rect);
    image = image(rect);

    // rotate template to align with ground truth 0 degree rotation
    rotate(rotation, image, image);
    rotate(rotation, raw, raw);

    // Center initial rect at top left of template image
    initialRect = cv::Rect2i(0, 0, image.cols, image.rows);
}

//TODO: Make this a member function
//rotate an image by angle degrees
void rotate(double angle, const cv::Mat &src, cv::Mat &dst)
{
    // get center of original img
    cv::Point2d center((src.cols - 1) / 2.0, (src.rows - 1) / 2.0);

    // get rotation matrix for rotating the image around its center in pixel coordinates
    cv::Mat rot = getRotationMatrix2D(center, angle, 1.0);
    // calculate dimensions of rotated image, center not relevant
    cv::Rect2d bbox = cv::RotatedRect(cv::Point2d(), src.size(), angle).boundingRect2f();

    // apply translation to rotation matrix to shift center 
    rot.at<double>(0, 2) += bbox.width / 2.0 - src.cols / 2.0;

    rot.at<double>(1, 2) += bbox.height / 2.0 - src.rows / 2.0;
    //apply matrix transformation
    warpAffine(src, dst, rot, bbox.size());
}

#endif