#include <opencv2/core.hpp>
#include <string>
#include <opencv2/cudaimgproc.hpp>
#include "needle_template.h"
#include "pfc_initializer_constants.h"

using namespace std;

// constructor
NeedleTemplate::NeedleTemplate(const string& path, const cv::Rect2i& rect, const cv::Point2d& origin, double rotation, pfc::match_params iparams)
: NeedleImage(path), origin(origin), params(iparams)
{
    //Crop both raw and preprocessed template images
    raw = raw(rect);
    image = image(rect);

    // rotate template to align with ground truth 0 degree rotation
    rotate(image, image, rotation);
    rotate(raw, raw, rotation);

    // Center initial rect at top left of template image
    initialRect = cv::Rect2i(0, 0, image.cols, image.rows);
}