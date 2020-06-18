#ifndef PFCINIT_CONSTANTS_H
#define PFCINIT_CONSTANTS_H

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

#endif
