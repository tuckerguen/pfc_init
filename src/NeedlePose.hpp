#ifndef NEEDLE_POSE_H
#define NEEDLE_POSE_H

#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Dense>
#include "PfcInitConstants.hpp"
#include <iostream>
using namespace std;

class NeedlePose 
{
    Eigen::Vector3f orientation;
public:
    cv::Point3d location;
    
    NeedlePose(cv::Point3d location, Eigen::Vector3f orientation) :
        location(location), orientation(orientation) 
    {}

    NeedlePose():
        location(cv::Point3d(0,0,0)), orientation(Eigen::Vector3f(0,0,0))
    {}

    void setOrientation(Eigen::Vector3f new_orientation) { orientation = new_orientation;}
    void setOrientation(Eigen::Quaternionf q) { 
        orientation = q.toRotationMatrix().eulerAngles(0, 1, 2);
        orientation = orientation * rad2deg;
    }
    Eigen::Vector3f getEulerAngleOrientation() { return orientation; };

    void print();
    Eigen::Quaternionf getQuaternionOrientation();
};

#endif