#ifndef NEEDLE_POSE
#define NEEDLE_POSE

#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "needle_pose.h"
#include "pfc_initializer_constants.h"

using namespace std;

//Return the orientation in quaternion representation
Eigen::Quaternionf NeedlePose::getQuaternionOrientation()
{
    // convert orientation from degrees to radians
    double roll_radians = pfc::deg2rad * orientation.x();
    double pitch_radians = pfc::deg2rad * orientation.y();
    double yaw_radians = pfc::deg2rad * orientation.z();

    // Convert euler angles to quaternion
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(roll_radians, Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(pitch_radians, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(yaw_radians, Eigen::Vector3f::UnitZ());
    
    return q;
}

// Format and print the location and orientation
void NeedlePose::print()
{
    Eigen::Quaternionf q = getQuaternionOrientation();
    cout << "Pos: (x,y,z)   = (" << location.x << ", " << location.y << ", " << location.z << ")" << endl;
    cout << "Rot: (x,y,z,w) = (" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << ")" << endl;
    cout << "Rot: (r,p,y)   = (" << orientation.x() << ", " << orientation.y() << ", " << orientation.z() << ")" << endl;
}

#endif