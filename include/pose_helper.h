#include <opencv2/core.hpp>
#include "template_match.h"
#include "needle_template.h"

/**
 * @brief returns 3D location of point given two pixel space points from endoscope stereo camera
 */
cv::Point3d deProjectPoints(const cv::Mat& p_l, const cv::Mat& p_r);

/**
 * @brief returns coordinate location of needle in template after rotation and scaling transformation
 */
cv::Mat getRotatedOrigin(double angle, double scale, NeedleTemplate* templ);
