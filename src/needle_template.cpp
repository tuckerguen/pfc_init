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
    rotate(image, image, rotation);
    rotate(raw, raw, rotation);

    // Center initial rect at top left of template image
    initialRect = cv::Rect2i(0, 0, image.cols, image.rows);
}