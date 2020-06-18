#include "PfcInit.hpp"
#include <string>
#include "PfcInitConstants.hpp"

using namespace std;
Eigen::Vector4f RPYtoQuat(double roll, double pitch, double yaw);

int main()
{
    string left_img_path = "../imgs/raw/1_l_c_marked.png";
    string right_img_path = "../imgs/raw/1_r_c_marked.png";
    string templ_img_path = "../imgs/raw/0_l_c_fatty.png";

    PfcInit pfc(left_img_path, right_img_path, templ_img_path, 1);
    pfc.computeNeedlePose();
    pfc.scorePoseEstimation();
    pfc.displayResults();
}
