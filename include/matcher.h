#include "needle_template.h"
#include "template_match.h"

/**
 * @brief Template match template on base image over range of scales and rotations
 * 
 * @param base_img Image to match template to
 * @param templ Needle template object with template to match with and scale/rotation parameters
 */
TemplateMatch matchOverScaleAndRotation(const cv::Mat& base_img, const NeedleTemplate* templ);


/**
 * @brief Run template match and store results if this match is better than bestMatch
 * 
 * @param img The base image to match template to
 * @param templ The template to match onto the image
 * @param bestMatch The current best match (by score)
 * @param angle Angle of rotation or template (degrees)
 * @param scale Scale of template
 */
void matchAndCompare(const cv::Mat &img, const cv::Mat& templ, TemplateMatch *bestMatch, double angle, double scale);


