#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

const int max_value_H = 360/2;
const int max_value = 255;
const String window_name = "Object Detection";
const String img_window_name = "img";
    Mat img, img_HSV, img_threshold;


int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;

void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_name, low_H);
        inRange(img_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), img_threshold);

}
void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_name, high_H);
        inRange(img_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), img_threshold);

}
void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_name, low_S);
        inRange(img_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), img_threshold);

}
void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_name, high_S);
        inRange(img_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), img_threshold);

}
void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_name, low_V);
        inRange(img_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), img_threshold);

}
void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_name, high_V);
    inRange(img_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), img_threshold);

}

int main(int argc, char* argv[])
{
    namedWindow(window_name);
    namedWindow(img_window_name);
    
    // Trackbars to set thresholds for HSV values
    createTrackbar("Low H", window_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    createTrackbar("High H", window_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    createTrackbar("Low S", window_name, &low_S, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High S", window_name, &high_S, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low V", window_name, &low_V, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High V", window_name, &high_V, max_value, on_high_V_thresh_trackbar);
    

    img = imread("../imgs/raw_r_fatty.png", IMREAD_COLOR);
    if(!img.data){
        cerr << "Loading image failed" << endl;
        exit(0);
    }
    cvtColor(img, img_HSV, COLOR_BGR2HSV);
    inRange(img_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), img_threshold);

    // Detect the object based on HSV Range Values
    // Show the frames
    imshow(window_name, img_threshold);
    Vec3b hsv=img_HSV.at<Vec3b>(0,0);
    int H=hsv.val[0]; //hue
    int S=hsv.val[1]; //saturation
    int V=hsv.val[2]; //value
    cout << H << ", " << S << ", " << V << endl;
    // Convert from BGR to HSV colorspace
    
  
    imshow(img_window_name, img_HSV);
    waitKey(0);
    
    return 0;
}