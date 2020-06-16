#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Dense>
#include "NeedlePose.hpp"
#include "PfcInitConstants.hpp"
#include <iostream>

using namespace std;

Eigen::Vector4f NeedlePose::getQuaternionOrientation()
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
    return q.coeffs();
}

void NeedlePose::print()
{
    Eigen::Vector4f quat = getQuaternionOrientation();
    cout << "3D Location: " << endl << "(" << location.x << ", " << location.y << ", " << location.z << ")" << endl;
    cout << "3D Orientation: " << endl;
    cout << "Quaternion: (" << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << ")" << endl;
    cout << "Euler angles: (" << orientation.x() << ", " << orientation.y() << ", " << orientation.z() << ")" << endl;
}