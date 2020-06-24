#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <opencv2/sfm/triangulation.hpp>
#include "template_match.h"
#include "pose_helper.h"


using namespace std;

cv::Point3d deProjectPoints(const cv::Mat& p_l, const cv::Mat& p_r)
{
    cv::Mat P_r = (cv::Mat_<double>(3, 4) << 662.450355616388, 0.0, 320.5, -3.31225177808194, 
                                     0.0, 662.450355616388, 240.5, 0.0,
                                     0.0, 0.0, 1.0, 0.0);

    cv::Mat P_l = (cv::Mat_<double>(3, 4) << 662.450355616388, 0.0, 320.5, 0.0, 
                                     0.0, 662.450355616388, 240.5, 0.0, 
                                     0.0, 0.0, 1.0, 0.0);

    cv::Mat results;

    vector<cv::Mat> points;
    points.push_back(p_l);
    points.push_back(p_r);

    vector<cv::Mat> projections;
    projections.push_back(P_l);
    projections.push_back(P_r);

    cv::sfm::triangulatePoints(points, projections, results);

    cv::Point3d result;

    result.x = results.at<double>(0);
    result.y = results.at<double>(1);
    result.z = results.at<double>(2);

    return result;
}

cv::Mat getRotatedOrigin(double angle, double scale, NeedleTemplate* templ)
{
    // "Correct angle" is clockwise, we rotate counter clockwise
    angle = -angle;

    // Original template size
    double cols = scale * templ->raw.cols;
    double rows = scale * templ->raw.rows;

    //Get center of original image
    cv::Point2d center((cols-1)/2.0, (rows-1)/2.0);
    //Get rotation matrix given center, angle, and scale
    cv::Mat rot = getRotationMatrix2D(center, angle, 1.0);
    //Compute what the size of the rotated image will be
    cv::Rect2d bbox = cv::RotatedRect(cv::Point2d(), cv::Size(cols, rows), angle).boundingRect2f();
    
    // Add translation to rotation matrix to shift the center of the image to the correct location
    rot.at<double>(0, 2) += bbox.width / 2.0 - cols / 2.0;
    rot.at<double>(1, 2) += bbox.height / 2.0 - rows / 2.0;

    //Scale the original origin to account for scale of template
    cv::Mat original_origin = (cv::Mat_<double>(3,1) << scale * templ->origin.x, scale * templ->origin.y, 1);
    cv::Mat final_origin = rot * original_origin;
    return final_origin;
}