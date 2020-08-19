#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include "pfc_initializer_constants.h"

using namespace std;

int main(int argc, char* argv[])
{
    int resolution = stoi(argv[6]);

    cv::Point2d needle_arc_l[resolution + 1];
    cv::Point2d needle_arc_r[resolution + 1];
    
    double radius = stod(argv[1]);

    Eigen::Matrix<double, 3, 4> projection;
    Eigen::Matrix4d needle_transformation;

    // Projection matrices
    Eigen::Matrix<double, 3, 4>P_l;
    Eigen::Matrix<double, 3, 4> P_r;
    
    P_l << 662.450355616388, 0.0, 320.5, 0.0, 
            0.0, 662.450355616388, 240.5, 0.0, 
            0.0, 0.0, 1.0, 0.0;

    P_r << 662.450355616388, 0.0, 320.5, -3.31225177808194, 
            0.0, 662.450355616388, 240.5, 0.0,
            0.0, 0.0, 1.0, 0.0;   

    // Transformation parameters
    // Location. For generating template, it can be centered, we only 
    // need to use z axis to control scale of the needle
    float tz = stod(argv[5]);
    // Rotation. a=yaw, b=pitch=, y=roll
    float a = stod(argv[2])*pfc::deg2rad, b = stod(argv[3])*pfc::deg2rad, y = stod(argv[4])*pfc::deg2rad;
    // Assign to transformation matrix
    // https://en.wikipedia.org/wiki/Rotation_matrix#General_rotations
    needle_transformation << cos(a)*cos(b), cos(a)*sin(b)*sin(y)-sin(a)*cos(y), cos(a)*sin(b)*cos(y)+sin(a)*sin(y),  0,
                             sin(a)*cos(b), sin(a)*sin(b)*sin(y)+cos(a)*cos(y), sin(a)*sin(b)*cos(y)-cos(a)*sin(y),  0,
                                   -sin(b),                      cos(b)*sin(y),                      cos(b)*cos(y), tz,
                                         0,                                  0,                                  0,  1; 
    
    cout << needle_transformation << endl;

    for(int i = 0; i <= resolution; i++)
    {
        double turn_amt = i * M_PI / resolution;

        Eigen::Vector4d needle_arc_pt = Eigen::Vector4d(
            radius * cos(turn_amt),
            radius * sin(turn_amt),
            0.0,
            1.0
        );

        //Transform based on needle origin location
        Eigen::Vector4d needle_arc_pt_tf = needle_transformation * needle_arc_pt;

        //Transform into u, v
        Eigen::Vector3d needle_arc_pt_uv_l = P_l * needle_arc_pt_tf;
        Eigen::Vector3d needle_arc_pt_uv_r = P_r * needle_arc_pt_tf;

        double u_l = needle_arc_pt_uv_l.x() / needle_arc_pt_uv_l.z();
        double v_l = needle_arc_pt_uv_l.y() / needle_arc_pt_uv_l.z();

        double u_r = needle_arc_pt_uv_r.x() / needle_arc_pt_uv_r.z();
        double v_r = needle_arc_pt_uv_r.y() / needle_arc_pt_uv_r.z();

        needle_arc_l[i].x = u_l;
        needle_arc_l[i].y = v_l; 

        needle_arc_r[i].x = u_r;
        needle_arc_r[i].y = v_r; 

        cout << u_l << ", " << v_l <<  endl;
        cout << u_r << ", " << v_r <<  endl;
    }

    // Draw into templates
    cv::Mat templ_l(cv::Size(640, 480), CV_64FC1, cv::Scalar(0));
    cv::Mat templ_r(cv::Size(640, 480), CV_64FC1, cv::Scalar(0));
    
    for(int i = 1; i <= resolution; i++)
    {
        cv::Point2d p_l1 = needle_arc_l[i-1];
        cv::Point2d p_l2 = needle_arc_l[i];

        cv::Point2d p_r1 = needle_arc_r[i-1];
        cv::Point2d p_r2 = needle_arc_r[i];

        cv::line(templ_l, p_l1, p_l2, cv::Scalar(255), 1, 8, 0);
        cv::line(templ_r, p_r1, p_r2, cv::Scalar(255), 1, 8, 0);
    }

    cv::namedWindow("left");
    cv::namedWindow("right");
    cv::imshow("left", templ_l);
    cv::imshow("right", templ_r);
    cv::waitKey(0);
}