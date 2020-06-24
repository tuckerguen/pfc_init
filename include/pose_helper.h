#ifndef POSE_HELPER_H
#define POSE_HELPER_H

#include <opencv2/core.hpp>
#include "template_match.h"
#include "needle_template.h"
#include "needle_pose.h"

/**
 * @brief returns 3D location of point given two pixel space points from endoscope stereo camera
 * 
 * @param p_l Location of point in left image
 * @param p_r Location of point in right image
 */
cv::Point3d deProjectPoints(const cv::Mat& p_l, const cv::Mat& p_r);

/**
 * @brief returns coordinate location of needle in template after rotation and scaling transformation
 * 
 * @param angle Template was rotated by
 * @param scale Template was scaled to
 * @param templ Template used for match (gives initial origin and template size)
 */
cv::Mat getRotatedOrigin(double angle, double scale, NeedleTemplate* templ);

/**
 * @brief draws needle origin on image, given match info and rotated/scaled template
 * 
 * @param img Image to draw the origin on
 * @param match Match object used to match template to img
 * @param color Color of the origin point to be drawn
 * @param templ The needle template used in matching
 */
void drawNeedleOrigin(cv::Mat& img, TemplateMatch* match, cv::Scalar color, NeedleTemplate* templ);

#endif