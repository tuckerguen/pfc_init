#include "NeedleTemplate.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <iostream>
#include "PfcInitConstants.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/cudaimgproc.hpp>

using namespace std;

//inital: "../imgs/raw/0_l_c_fatty.png", 287, 205, 105, 56, 180

NeedleTemplate::NeedleTemplate(const string& path, const cv::Rect2i& rect, int origin_offset_x, int origin_offset_y, double rotation)
: initialRect(rect), origin_offset_x(origin_offset_x), origin_offset_y(origin_offset_y)
{
    //Load camera image to match
    cv::Mat raw, img_HSV, segmented;
    raw = cv::imread(path, cv::IMREAD_COLOR);
    if (!raw.data)
    {
        cerr << "Loading image: " << path << " failed" << endl;
        exit(0);
    }

    //Crop template image to just needle
    raw = raw(initialRect);

    // Filter by HSV for needle
    cvtColor(raw, img_HSV, cv::COLOR_BGR2HSV);
    inRange(img_HSV, cv::Scalar(low_h, low_s, low_v), cv::Scalar(high_h, high_s, high_v), segmented);
    rotate(rotation, segmented, templ);
}

void rotate(double degrees, const cv::Mat &src, cv::Mat &dst)
{
    /// get rotation matrix for rotating the image around its center in pixel coordinates
    cv::Point2d center((src.cols - 1) / 2.0, (src.rows - 1) / 2.0);

    cv::Mat rot = getRotationMatrix2D(center, degrees, 1.0);
    /// determine bounding rectangle, center not relevant
    cv::Rect2d bbox = cv::RotatedRect(cv::Point2d(), src.size(), degrees).boundingRect2f();
    /// adjust transformation matrix
    rot.at<double>(0, 2) += bbox.width / 2.0 - src.cols / 2.0;
    rot.at<double>(1, 2) += bbox.height / 2.0 - src.rows / 2.0;

    warpAffine(src, dst, rot, bbox.size());
}

TemplateMatch NeedleTemplate::matchOverScaleAndRotation(const cv::Mat& img, double min_scale, double max_scale, double scale_increment, double min_rotation, double max_rotation, double rotation_increment)
{
    TemplateMatch bestMatch(min_rotation, -DBL_MAX, min_scale);

    double scale = min_scale / 100.0;
    for (int i = 0; i < ceil((max_scale - min_scale) / scale_increment); ++i)
    {
        scale += ((double)scale_increment) / 100.0;
        cv::Mat resized, rot_templ;

        //Use inter-linear in all cases (is faster than inter_area, similar results)
        cv::resize(templ, resized, cv::Size(), scale, scale, cv::INTER_LINEAR);

        for (double rot_angle = min_rotation; rot_angle < max_rotation; rot_angle += rotation_increment)
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
    }
}