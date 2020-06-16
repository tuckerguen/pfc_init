#ifndef NEEDLE_POSE_H
#define NEEDLE_POSE_H

#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Dense>

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
    Eigen::Vector3f getEulerAngleOrientation() { return orientation; };

    void print();
    Eigen::Vector4f getQuaternionOrientation();
};

#endif