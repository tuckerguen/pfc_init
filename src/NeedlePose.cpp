#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Dense>
#include "NeedlePose.hpp"
#include "PfcInitConstants.hpp"
#include <iostream>

using namespace std;

Eigen::Quaternionf NeedlePose::getQuaternionOrientation()
{
    // double roll_radians = deg2rad * orientation.x();
    // double pitch_radians = deg2rad * orientation.y();
    double roll_radians = 0;
    double pitch_radians = 0;
    double yaw_radians = deg2rad * orientation.z();
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(roll_radians, Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(pitch_radians, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(yaw_radians, Eigen::Vector3f::UnitZ());
    return q;
}

void NeedlePose::print()
{
    Eigen::Quaternionf q = getQuaternionOrientation();
    cout << "Pos: (x,y,z)   = (" << location.x << ", " << location.y << ", " << location.z << ")" << endl;
    cout << "Rot: (x,y,z,w) = (" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << ")" << endl;
    cout << "Rot: (r,p,y)   = (" << orientation.x() << ", " << orientation.y() << ", " << orientation.z() << ")" << endl;
}