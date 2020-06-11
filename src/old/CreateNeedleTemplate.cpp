//
// Created by tucker on 5/21/20.
//
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>     // std::string, std::to_string


using namespace cv;
using namespace std;

//HSV Filtering Parameters
const int low_h = 0, high_h = 5;
const int low_s = 0, high_s = 0;
const int low_v = 0, high_v = 140;

void RotateTemplate(double angle, const Mat &src, Mat &dst)
{
    /// get rotation matrix for rotating the image around its center in pixel coordinates
    Point2d center((src.cols - 1) / 2.0, (src.rows - 1) / 2.0);

    Mat rot = getRotationMatrix2D(center, angle, 1.0);
    /// determine bounding rectangle, center not relevant
    Rect2d bbox = RotatedRect(Point2d(), src.size(), angle).boundingRect2f();
    /// adjust transformation matrix
    rot.at<double>(0, 2) += bbox.width / 2.0 - src.cols / 2.0;
    rot.at<double>(1, 2) += bbox.height / 2.0 - src.rows / 2.0;

    warpAffine(src, dst, rot, bbox.size());
}

int main()
{
    Mat raw, filtered, img_HSV, templ;
    raw = imread("../../imgs/raw/0_l_c_fatty.png", IMREAD_COLOR);
    if (!raw.data)
    {
        cerr << "Loading image failed" << endl;
        exit(0);
    }

    //Crop template image to just needle
    Rect r(287, 205, 105, 56);
    raw = raw(r);

    ///With HSV filtering
    // Filter by HSV for needle
    cvtColor(raw, img_HSV, COLOR_BGR2HSV);
    inRange(img_HSV, Scalar(low_h, low_s, low_v), Scalar(high_h, high_s, high_v), templ);

    // for(int i = 0; i < 356; i+=5){
    //     Mat rotated;
    //     RotateTemplate(i, templ, rotated);
    //     string name = "../../imgs/templates/template_";
    //     string i_str = to_string(i);
        
    //     name = name + i_str + ".png";
    //     cout << name << endl;
    //     imwrite(name, rotated);
    // }

    imwrite("../imgs/templates/template.png", templ);
    Mat rotated;
    RotateTemplate(180, templ, rotated);
    namedWindow("hi", WINDOW_AUTOSIZE);
    imshow("hi", rotated);
    imwrite("../imgs/templates/template.png", rotated);

    // namedWindow("template", WINDOW_NORMAL | WINDOW_KEEPRATIO);

    // imshow( "template", rotated);

    waitKey(0);

    return 0;
}



