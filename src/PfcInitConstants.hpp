#ifndef PFCINIT_CONSTANTS_H
#define PFCINIT_CONSTANTS_H

#include <opencv2/core.hpp>

namespace pfc {
    //Template parameters
    const std::string templ_path = "../imgs/raw/0_l_c_fatty.png";
    const cv::Rect2i initial_rect(287,205,105,56);
    const int origin_offset_x = 52;
    const int origin_offset_y = 4;
    const double initial_rotation = -176.6537;

    // Template Match Rotation Parameters
    const double min_rotation = 0;
    const double max_rotation = 360;   //Max number of degrees to rotate template
    const double rotation_increment = 10; //Number of degrees to rotate template each iteration

    // Template Match Scaling Parameters
    const int min_scale = 95;         //minimum template scale to try to match (in %)
    const int max_scale = 110;        //maximum template scale to try to match (in %)
    const double scale_increment = 1; //% scale to increase by on each iteration

    //Degree 2 radians conversion constant
    const double deg2rad = M_PI / 180.0;
    const double rad2deg = 180.0 / M_PI;

    //HSV Filtering Parameters
    const int low_h = 0, high_h = 5;
    const int low_s = 0, high_s = 0;
    const int low_v = 0, high_v = 140;

    //Match drawing constants
    const int line_weight = 1;
    const int line_type = 8;
    const int shift = 0;
}

#endif
