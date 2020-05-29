///
/// Created by tucker on 5/21/20.
///
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <unistd.h>

using namespace std;
using namespace cv;

const string template_img_path = "../imgs/vessel_l_b.png";

//Canny edge detection parameters
int lowThreshold = 8;
const int ratio = 3;
const int kernel_size = 3;

const double max_rotation = 360; //Max number of degrees to rotate template
const double angle_increment = 10; //Number of degrees to rotate template each iteration

const int min_scale = 90; //minimum template scale to try to match (in %)
const int max_scale = 110; //maximum template scale to try to match (in %)
const double scale_increment = 1; //% scale to increase by on each iteration

struct match {
    double maxVal;
    double angle;
    double scale;
    Point maxLoc;
    Mat templ;
    Mat result;
};

int PFCInit(string left_image_path, string right_image_path);
Point3d DeProjectPoints(const Mat& img_l, const Mat& img_r, const match* match_l, const match* match_r);
void DrawMatch(Mat& src, match* match);
int LoadNeedleImg(string path, Mat& img);
void PrintResultsForImage(match *match, string side);
void LocateNeedle (const Mat& img, const Mat& templ, match *bestMatch);
void Match(const Mat& img, const Mat& templ, match* bestMatch, double angle, double scale);
void RotTemplate(double angle, const Mat &src, Mat &dst);
void DetectEdges(const Mat& img, Mat& dst);

int main(int argc, char* argv[]){
    string image_id = "a";
    string image_type = "vessel";

    if(argc == 3){
        image_id = argv[1];
        if(!strcmp(argv[2], "raw") || !strcmp(argv[2], "vessel")){
            image_type = argv[2];
        }
        else{
            cout << argv[2] << " is not a valid image type" << endl;
            return 0;
        }
    }
    else if(argc != 1){
        cout << "not proper use" << endl;
        return 0;
    }

    string left_image_path = "../imgs/" + image_type + "_l_" + image_id + ".png";
    string right_image_path = "../imgs/" + image_type + "_r_" + image_id + ".png";

    int status = PFCInit(left_image_path, right_image_path);

    if(status){
        cout << "init failed" << endl;
    }
}

int PFCInit(string left_image_path, string right_image_path){
    ///Define viewing windows
    namedWindow( left_image_path, WINDOW_AUTOSIZE );
    namedWindow( right_image_path, WINDOW_AUTOSIZE );
    namedWindow( "best template", WINDOW_AUTOSIZE );
    namedWindow( "result", WINDOW_AUTOSIZE );

    //Start timer
    double t = (double)getTickCount();

    ///Define source, template, and result mats
    Mat img_l, img_r, templ;

    //load images
    if( !LoadNeedleImg(left_image_path, img_l)
        || !LoadNeedleImg(right_image_path, img_r)
        || !LoadNeedleImg(template_img_path, templ))
    {
       return -1;
    } 
    
    //Crop template image to just needle
    Rect r(320, 175, 115, 70);
    templ = templ(r);

    Mat result;

    //Init best match info for left image
    match bestMatch_l = {
            -DBL_MAX,
            0.0,
            0.0,
            Point(0, 0),
            templ,
            result
    };

    //Init best match info for right image
    match bestMatch_r = {
            -DBL_MAX,
            0.0,
            0.0,
            Point(0, 0),
            templ,
            result
    };
    

    //Run localization algorithm on left and right images
    LocateNeedle(img_l, templ, &bestMatch_l);
    LocateNeedle(img_r, templ, &bestMatch_r);

    //Draw the matches on originals and result images
    DrawMatch(img_l, &bestMatch_l);
    DrawMatch(bestMatch_l.result, &bestMatch_l);
    DrawMatch(img_r, &bestMatch_r);
    DrawMatch(bestMatch_r.result, &bestMatch_r);

    //Record time
    t = ((double)getTickCount() - t)/getTickFrequency();
    cout << "Times passed in seconds: " << t << endl;
    cout << "--------------------------------------" << endl;

    //Print match information
    PrintResultsForImage(&bestMatch_l, "left");
    PrintResultsForImage(&bestMatch_r, "right");

    Point3d location = DeProjectPoints(img_l, img_r, &bestMatch_l, &bestMatch_r);
    cout << "location: (" << location.x << ", " << location.y << ", " << location.z << endl;

    //Display images
    imshow( left_image_path, img_l );
    // imshow(right_image_path, img_r );
    imshow("best template", bestMatch_l.templ);
    imshow("result", bestMatch_l.result);

    waitKey(0);
    return 0;
}

Point3d DeProjectPoints(const Mat& img_l, const Mat& img_r, const match* match_l, const match* match_r){
    //construct the output mat:
    Mat results(4,1,CV_64FC1);
    Mat p_l(2,1,CV_64FC1);
    Mat p_r(2,1,CV_64FC1);

    p_l.at<double>(0) = match_l->maxLoc.x;
    p_l.at<double>(1) = match_l->maxLoc.y;

    p_r.at<double>(0) = match_r->maxLoc.x;
    p_r.at<double>(1) = match_r->maxLoc.y;

    //Camera intrinsic matrices (placeholders for now)
    double fx_l = 5.749;
    double fy_l = 5.999;
    double rh_l = img_l.cols / 2.0;
    double rv_l = img_l.rows / 2.0;
    
    double fx_r = 4.500;
    double fy_r = 4.680;
    double rh_r = img_r.cols / 2.0;
    double rv_r = img_r.rows / 2.0;

    Mat P_l = (Mat_<double>(3,4) <<  fx_l,     0,  rh_l, 0, 
                                        0,  fy_l,  rv_l, 0, 
                                        0,     0,     0, 1);

    Mat P_r = (Mat_<double>(3,4) <<  fx_r,     0,  rh_r, 0, 
                                        0,  fy_r,  rv_r, 0, 
                                        0,     0,     0, 1);

    triangulatePoints(P_l, P_r, p_l, p_r, results);

    Point3d result;

    result.x =  results.at<double>(0)/results.at<double>(3);
    result.y =  results.at<double>(1)/results.at<double>(3);
    result.z =  results.at<double>(2)/results.at<double>(3);

    return result;
}

void DrawMatch(Mat& src, match* match){
    rectangle( src, match->maxLoc,
            Point(
            match->maxLoc.x + match->templ.cols ,
            match->maxLoc.y + match->templ.rows ),
                    Scalar::all(180), 2, 8, 0 );
}

int LoadNeedleImg(string path, Mat& img){
    //Load camera image to match
    Mat raw;
    raw = imread(path, IMREAD_COLOR);
    if(!raw.data){
        cerr << "Loading image failed" << endl;
        return 0;
    }
    // Do edge detection on raw image
    DetectEdges(raw, img);
    return 1;
}

void PrintResultsForImage(match *match, string side){
    cout << side << " image: " << endl;
    if(match->angle > 180.0){
        match->angle =  -(match->angle - 360.0);
    }
    cout << "Location: (" << match->maxLoc.x << ", " << match->maxLoc.y << ")" << endl;
    cout << "Yaw: degrees = " << match->angle << ", radians = " << match->angle * (3.1415926535 / 180.0) << endl;
    cout << "Scale: " << match->scale << endl;
    cout << "--------------------------------------" << endl;
}

void LocateNeedle (const Mat& img, const Mat& templ, match *bestMatch){
    //Loop over all scales
    double scale = min_scale / 100.0;
    for(int i = 0; i < ceil((max_scale - min_scale) / scale_increment); ++i){
        scale += ((double) scale_increment) / 100.0;
        Mat resized;
        //If scaling up, use inter-linear interpolation (as recommended by opencv documentation)
        if(scale > 1){
            resize(templ, resized, Size(), scale, scale, INTER_LINEAR);
        }
        //If scaling down, use inter-area interpolation
        else {
            resize(templ, resized, Size(), scale, scale, INTER_AREA);
        }

        //Use inter-linear in all cases (is faster than inter_area, similar results)
        // resize(templ, resized, Size(), scale, scale, INTER_LINEAR);
        
        ///Loop over all rotations
        double rot_angle = 0;   
        for(int j = 0; j < ceil(max_rotation / angle_increment); ++j){
            rot_angle += angle_increment;
            //Rotate
            Mat rot_templ;
            RotTemplate(rot_angle, resized, rot_templ);

            //Match rotated template to image
            Match(img, rot_templ, bestMatch, rot_angle, scale);
        }
    }
}

void DetectEdges(const Mat& img, Mat& dst){
    Mat detected_edges;

    //What should the kernel size be?
    GaussianBlur( img, detected_edges, Size(1,1), 0);
    Canny( detected_edges, dst, lowThreshold, lowThreshold * ratio, kernel_size );
}

void Match(const Mat& img, const Mat& templ, match* bestMatch, double angle, double scale){
    /// Create the result matrix
    Mat result;
    int result_cols =  img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;

    result.create( result_rows, result_cols, CV_32FC1 );

    matchTemplate(img, templ, result, TM_CCOEFF);

    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;
    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

    //If the new match is better than our previous best, record it
    // cout << ": " << maxVal << endl;
    if(bestMatch->maxVal < maxVal){
        bestMatch->maxVal = maxVal;
        bestMatch->angle = angle;
        bestMatch->scale = scale;
        bestMatch->maxLoc = maxLoc;
        bestMatch->templ = templ;
        bestMatch->result = result;
    }
    
}

void RotTemplate(double angle, const Mat &src, Mat &dst){
    /// get rotation matrix for rotating the image around its center in pixel coordinates
    Point2f center((src.cols-1)/2.0, (src.rows-1)/2.0);
    Mat rot = getRotationMatrix2D(center, angle, 1.0);
    /// determine bounding rectangle, center not relevant
    Rect2f bbox = RotatedRect(Point2f(), src.size(), angle).boundingRect2f();
    /// adjust transformation matrix
    rot.at<double>(0,2) += bbox.width/2.0 - src.cols/2.0;
    rot.at<double>(1,2) += bbox.height/2.0 - src.rows/2.0;

    warpAffine(src, dst, rot, bbox.size());
}