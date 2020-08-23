#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include "pfc_initializer_constants.h"

using namespace std;
cv::Mat GenerateTemplate(float z, float a, float b, float y, int resolution, bool left);

int main(int argc, char* argv[])
{
    // for(int i = 9; i < 18; i ++){
        cv::Mat templ_l = GenerateTemplate(0.177,pfc::deg2rad*176.5, 45, 0, 10, true);
        cv::Mat templ_r = GenerateTemplate(0.177, pfc::deg2rad*176.5, 45, 0, 10, false);

        cv::namedWindow("left");
        cv::namedWindow("right");
        cv::imshow("left", templ_l);
        cv::imshow("right", templ_r);
        cv::waitKey(0);
    // }
    
}

cv::Mat GenerateTemplate(float z, float a, float b, float y, int resolution, bool left)
{
    // Array of needle points
    cv::Point2d needle_arc[resolution + 2];

    double radius = 0.0128;

    // Projection matrices
    cv::Mat projection = left ? pfc::P_l : pfc::P_r;

    // Transformation Matrix
    // https://en.wikipedia.org/wiki/Rotation_matrix#General_rotations
    cv::Mat transform = (cv::Mat_<double>(4,4) << 
        cos(a)*cos(b), cos(a)*sin(b)*sin(y)-sin(a)*cos(y), cos(a)*sin(b)*cos(y)+sin(a)*sin(y),  0,
        sin(a)*cos(b), sin(a)*sin(b)*sin(y)+cos(a)*cos(y), sin(a)*sin(b)*cos(y)-cos(a)*sin(y),  0,
              -sin(b),                      cos(b)*sin(y),                      cos(b)*cos(y),  z,
                    0,                                  0,                                  0,  1); 
    
    // To determine bounding box of needle for cropping
    double leftmost=640, rightmost=0, upmost=480, downmost=0;
    double boundary = 5;

    cv::Mat needle_arc_origin = (cv::Mat_<double>(4,1) <<
        0.0,
        0.0,
        0.0,
        1.0
    );

    //Transform based on needle origin location
    cv::Mat needle_arc_origin_tf = transform * needle_arc_origin;

    //Transform into u, v
    cv::Mat needle_arc_origin_uv = projection * needle_arc_origin_tf;

    double u = needle_arc_origin_uv.at<double>(0) / needle_arc_origin_uv.at<double>(2);
    double v = needle_arc_origin_uv.at<double>(1) / needle_arc_origin_uv.at<double>(2);

    needle_arc[0].x = u;
    needle_arc[0].y = v; 

    // cout << projection << endl << transform << endl;


    for(int i = 1; i <= resolution; i++)
    {
        double turn_amt = i * M_PI / resolution;

        cv::Mat needle_arc_pt = (cv::Mat_<double>(4,1) <<
            radius * cos(turn_amt),
            radius * sin(turn_amt),
            0.0,
            1.0
        );

        //Transform based on needle origin location
        cv::Mat needle_arc_pt_tf = transform * needle_arc_pt;

        //Transform into u, v
        cv::Mat needle_arc_pt_uv = projection * needle_arc_pt_tf;

        double u = needle_arc_pt_uv.at<double>(0) / needle_arc_pt_uv.at<double>(2);
        double v = needle_arc_pt_uv.at<double>(1) / needle_arc_pt_uv.at<double>(2);

        needle_arc[i].x = u;
        needle_arc[i].y = v; 

        // Track the needle boundaries
        if(u < leftmost)
            leftmost = u;
        if(u > rightmost)
            rightmost = u;
        if(v < upmost)
            upmost = v;
        if(v > downmost)
            downmost = v;
    }

    // Draw into template
    cv::Mat templ(cv::Size(640, 480), CV_64FC1, cv::Scalar(0));
    
    for(int i = 2; i <= resolution; i++)
    {
        cv::Point2d p1 = needle_arc[i-1];
        cv::Point2d p2 = needle_arc[i];

        cv::line(templ, p1, p2, cv::Scalar(255), 1, 8, 0);
    }
    cv::circle(templ, needle_arc[0], 2, cv::Scalar(255),1,8, 0);

    // Crop image to only include the needle
    upmost = floor(upmost); leftmost = floor(leftmost); 
    downmost = ceil(downmost);  rightmost=ceil(rightmost);

    cv::Range rows(upmost-boundary, downmost+boundary);
    cv::Range cols(leftmost-boundary, rightmost+boundary);
    templ = templ(rows, cols);
    cout << templ.cols << ":, " << templ.rows << endl;

    return templ;
}