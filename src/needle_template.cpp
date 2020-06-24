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
    cout << origin << endl;
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

TemplateMatch NeedleTemplate::matchOverScaleAndRotation(const cv::Mat& img)
{
    TemplateMatch bestMatch(params.min_rotation, -DBL_MAX, params.min_scale);
    double scale = params.min_scale / 100.0;
 
    for (int i = 0; i < ceil((params.max_scale - params.min_scale) / params.scale_increment); ++i)
    {
        scale += ((double)params.scale_increment) / 100.0;

        cv::Mat resized, rot_templ;

        //Use inter-linear in all cases (is faster than inter_area, similar results)
        cv::resize(image, resized, cv::Size(), scale, scale, cv::INTER_LINEAR);

        for (double rot_angle = params.min_rotation; rot_angle < params.max_rotation; rot_angle += params.rotation_increment)
        {
            //Rotate template
            rotate(rot_angle, resized, rot_templ);
            //Match rotated template to image
            match(img, rot_templ, &bestMatch, rot_angle, scale);
        }
    }

    return bestMatch;
}

void match(const cv::Mat &img, const cv:: Mat& templ, TemplateMatch *bestMatch, double angle, double scale)
{
    /// Create the result matrix
    cv::Mat result;
    int result_cols = img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;
    result.create(result_rows, result_cols, CV_32FC1);
    
    //Match using TM_CCOEFF_NORMED
    cv::matchTemplate(img, templ, result, cv::TM_CCOEFF_NORMED);

    /// Localizing the best match with minMaxLoc
    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;
    cv::Point matchLoc;
    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

    // if (angle>135 && angle < 160)
        // cout << angle << ": " << maxVal << endl;

    //If the new match is better than our previous best, record it
    if (bestMatch->score < maxVal)
    {
        bestMatch->score = maxVal;
        if(angle > 180.0)
            bestMatch->angle = 360 - angle;
        else
            bestMatch->angle = -angle;
        bestMatch->scale = scale;
        bestMatch->rect.x = maxLoc.x;
        bestMatch->rect.y = maxLoc.y;
        bestMatch->rect.width = templ.cols;
        bestMatch->rect.height = templ.rows;
        bestMatch->result = result;
        bestMatch->templ = templ;
    }
}

#endif