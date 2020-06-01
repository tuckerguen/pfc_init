#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

int main(){
    //construct the output mat:
    Mat results(4,1,CV_64FC1);
    Mat p_l(2,1,CV_64FC1);
    Mat p_r(2,1,CV_64FC1);

    p_l.at<double>(0) = 278;
    p_l.at<double>(1) = 198;

    p_r.at<double>(0) = 259;
    p_r.at<double>(1) = 198;

    //Need Camera Intrinsic Matrices P_l and P_r
    Mat P_l = (Mat_<double>(3,4) << 5.749,     0, 2.207, 0, 
                                        0, 5.999, 0.530, 0, 
                                        0,     0,     1, 0);

    Mat P_r = (Mat_<double>(3,4) << 4.500,     0, 3.691, 0, 
                                        0, 4.680, 0.225, 0, 
                                        0,     0,     1, 0);

    triangulatePoints(P_l, P_r, p_l, p_r, results);

    Point3d output;

    output.x =  results.at<double>(0)/results.at<double>(3);
    output.y =  results.at<double>(1)/results.at<double>(3);
    output.z =  results.at<double>(2)/results.at<double>(3);

    cout << "location: (" << output.x << ", " << output.y << ", " << output.z << ")" << endl;
}