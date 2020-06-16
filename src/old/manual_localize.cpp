#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <iostream>
#include <unistd.h>

using namespace std;
using namespace cv;

Mat img, templ;
Rect currPos(0, 0, 0, 0);
int offset_x = 15, offset_y=15, angle=0;
double scale=1;
string win_name = "img";
string win_templ = "templ";

//HSV Filtering Parameters
const int low_h = 0, high_h = 5;
const int low_s = 0, high_s = 0;
const int low_v = 0, high_v = 140;

void moveX(int, void*);

int main(){
    namedWindow(win_name, WINDOW_AUTOSIZE);
    namedWindow(win_templ, WINDOW_AUTOSIZE);

    img = imread("../imgs/raw_l_a.png", IMREAD_COLOR);
    //Load camera image to match
    Mat raw, HSV;
    raw = imread("../imgs/raw_l_b.png", IMREAD_COLOR);
    if (!raw.data)
    {
        cerr << "Loading image failed" << endl;
        exit(0);
    }
    //Crop template image to just needle
    Rect r(168, 92, 58, 35);
    raw = raw(r);
    cvtColor(raw, HSV, COLOR_BGR2HSV);
    inRange(HSV, Scalar(low_h, low_s, low_v), Scalar(high_h, high_s, high_v), templ);

    imwrite("../imgs/template.png", templ);

    // for(int x= 0; x < templ.cols; x++){
    //     for(int y = 0; y < templ.rows; y++){
    //         Vec3b val = templ.at<Vec3b>(y,x);
    //         int h = (int)val[0];
    //         int s = (int)val[1];
    //         int v = (int)val[2];
    //         cout << "h: " << h << " s: " << s << " v: " << v << endl;
    //         //around white
    //         if(h >= low_h && h <= high_h && s >= low_s && s <= high_s && v >= low_v && v <= high_v){
    //             img.at<Vec3b>(y + offset_y, x + offset_x) = 0;
    //         }
    //         // if()
    //     }
    // }

    // currPos = Rect(x, y, templ.cols, templ.rows);
    // // addWeighted(templ, 0.5, img, 0.5, 0, img);
    // templ.copyTo(img(currPos));

    // createTrackbar("x", win_name, &x, img.cols - templ.cols, moveX);
    // moveX(0, 0);
    imshow(win_templ, templ);
    imshow(win_name, img);
    waitKey(0);
}

// void moveX(int, void*){
//     currPos = Rect(offset_x, y, templ.cols, templ.rows);
//     // addWeighted(templ, 0.5, img, 0.5, 0, img);
//     templ.copyTo(img(currPos));
//     imshow(win_name, img);
// }

// void move(y)