#ifndef MAIN
#define MAIN

#include "PfcInit.hpp"
#include <string>
#include "PfcInitConstants.hpp"

using namespace std;
Eigen::Vector4f RPYtoQuat(double roll, double pitch, double yaw);

int main()
{
    int pose_id = 1;

    string left_img_path = "../imgs/raw/" + std::to_string(pose_id) + "_l_c_marked.png";
    string right_img_path = "../imgs/raw/" + std::to_string(pose_id) + "_r_c_marked.png";

    PfcInit pfc(left_img_path, right_img_path, pose_id);
    pfc.computeNeedlePose();
    pfc.scorePoseEstimation();
    pfc.displayResults();
}

#endif
