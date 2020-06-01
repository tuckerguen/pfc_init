//
// Created by tucker on 5/21/20.
//
#include <opencv2/opencv.hpp>

using namespace cv;

int main()
{
    Mat img;
    img = imread( "imgs/vessel_l_b.png", IMREAD_GRAYSCALE);

    if(!img.data){
        printf("No image data \n");
        return -1;
    }

    Mat templ, detected_edges;
    blur( img, detected_edges, Size(3,3) );
    Canny( detected_edges, detected_edges, 0, 0, 3 );
    templ = Scalar::all(0);
    img.copyTo( templ, detected_edges);
    ///crop template image to only needle
    Rect r(280, 200, 115, 65);
    templ = templ(r);

    imwrite("imgs/template.png", templ);

    namedWindow("template", WINDOW_NORMAL | WINDOW_KEEPRATIO);

    imshow( "template", templ);

    waitKey(0);

    return 0;
}



