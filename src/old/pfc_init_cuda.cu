///
/// Created by tucker on 5/21/20.
///
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <iostream>
#include <unistd.h>

using namespace std;
using namespace cv;

//A template match
struct match {
    double maxVal; //max matching value assigned by opencv templatematch()
    double angle; //angle matched at
    double scale; //scale matched at
    Point maxLoc; //location of match
    Mat templ; //template used to match
    Mat result; //result image from opencv templatematch()
};

int PFCInit(string left_image_path, string right_image_path);
Point3d DeProjectPoints(const Mat& img_l, const Mat& img_r, const match* match_l, const match* match_r);
void DrawMatch(Mat& src, match* match);
void InitNeedleImage(string path, Mat& img);
void InitTemplate(string path, Mat& templ);
void PrintResultsForImage(match *match, string side);
__global__ void LocateNeedle (const Mat& img, const Mat& templ, match *bestMatch);
__device__ void MatchImageToTemplate(const Mat& img, const Mat& templ, match* bestMatch, double angle, double scale, bool use_gpu);
__device__ void RotateTemplate(double angle, const Mat &src, Mat &dst);
void DetectEdges(const Mat& img, Mat& dst);

const string template_img_path = "../imgs/raw_l_b.png";

//Canny edge detection parameters
int lowThreshold = 8;
const int ratio = 3;
const int kernel_size = 3;

// Rotation parameters
const double max_rotation = 360; //Max number of degrees to rotate template
const double angle_increment = 10; //Number of degrees to rotate template each iteration

// Scaling parameters
const int min_scale = 50; //minimum template scale to try to match (in %)
const int max_scale = 150; //maximum template scale to try to match (in %)
const double scale_increment = 1; //% scale to increase by on each iteration

//HSV Filtering Parameters
const int low_h = 0, high_h = 5;
const int low_s = 0, high_s = 0;
const int low_v = 0, high_v = 140;


int main(int argc, char* argv[]){
    string image_id = "a";
    string image_type = "raw";

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
    InitNeedleImage(left_image_path, img_l);
    InitNeedleImage(right_image_path, img_r);
    InitTemplate(template_img_path, templ);

    Mat raw_l, raw_r;
    raw_l = imread(left_image_path, IMREAD_COLOR);
    raw_r = imread(right_image_path, IMREAD_COLOR);

    Mat init_result;
    //Init best match info for left image
    match bestMatch_l = {
            -DBL_MAX,
            0.0,
            0.0,
            Point(0, 0),
            templ,
            init_result
    };

    //Init best match info for right image
    match bestMatch_r = {
            -DBL_MAX,
            0.0,
            0.0,
            Point(0, 0),
            templ,
            init_result
    };
    

    //Run localization algorithm on left and right images
    LocateNeedle(img_l, templ, &bestMatch_l);
    LocateNeedle(img_r, templ, &bestMatch_r);

    //Draw the matches on originals and result images
    DrawMatch(raw_l, &bestMatch_l);
    DrawMatch(bestMatch_l.result, &bestMatch_l);
    DrawMatch(raw_r, &bestMatch_r);
    DrawMatch(bestMatch_r.result, &bestMatch_r);

    //Record time
    t = ((double)getTickCount() - t)/getTickFrequency();
    cout << "Times passed in seconds: " << t << endl;
    cout << "--------------------------------------" << endl;

    //Print match information
    PrintResultsForImage(&bestMatch_l, "left");
    PrintResultsForImage(&bestMatch_r, "right");

    // Point3d location = DeProjectPoints(img_l, img_r, &bestMatch_l, &bestMatch_r);
    // cout << "location: (" << location.x << ", " << location.y << ", " << location.z << endl;

    //Display images
    imshow( left_image_path, raw_l );
    imshow(right_image_path, raw_r );
    imshow("best template", bestMatch_l.templ);
    imshow("result", bestMatch_l.result);

    waitKey(0);
    return 0;
}

double IntersectionOverUnion(const Rect *ground, const Rect *data){
    int x1, y1, x2, y2, width, height, overlap_area, rect_union;
    //Upper left corner
    x1 = max(ground->x, data->x);
    y1 = max(ground->y, data->y);
    //Lower right corner
    x2 = min(ground->x + ground->width, data->x + data->width);
    y2 = min(ground->y + ground->height, data->y + data->height);

    //overlap area
    width = x2 - x1;
    height = y2 - y1;
    if(width < 0 || height < 0)
        return 0.0;
    overlap_area = width * height;

    //Combined area
    rect_union = ground->area() + data->area();

    return (double) overlap_area / (double) rect_union;
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


    // triangulatePoints(P_l, P_r, p_l, p_r, results);

    Point3d result;

    result.x =  results.at<double>(0)/results.at<double>(3);
    result.y =  results.at<double>(1)/results.at<double>(3);
    result.z =  results.at<double>(2)/results.at<double>(3);

    return result;
}


void DrawMatch(Mat& src, match* match){
    int line_weight = 1;
    rectangle( src, match->maxLoc,
            Point(
                match->maxLoc.x + match->templ.cols ,
                match->maxLoc.y + match->templ.rows ),
                    Scalar::all(255), line_weight, 8, 0 );
}

void InitNeedleImage(string path, Mat& img){
    //Load camera image to match
    Mat raw, filtered, img_HSV;
    raw = imread(path, IMREAD_COLOR);
    if(!raw.data){
        cerr << "Loading image failed" << endl;
        exit(0);
    }

    // Filter by color for needle
    cvtColor(raw, img_HSV, COLOR_BGR2HSV);
    inRange(img_HSV, Scalar(low_h, low_s, low_v), Scalar(high_h, high_s, high_v), img);

    // Do edge detection on filtered image
    // DetectEdges(raw, img);
}

void InitTemplate(string path, Mat& templ){
    //Load camera image to match
    Mat raw, filtered, img_HSV;
    raw = imread(path, IMREAD_COLOR);
    if(!raw.data){
        cerr << "Loading image failed" << endl;
        exit(0);
    }

    //Crop template image to just needle
    Rect r(168, 92, 58, 35);
    raw = raw(r);

    ///With HSV filtering
    // Filter by HSV for needle
    cvtColor(raw, img_HSV, COLOR_BGR2HSV);
    inRange(img_HSV, Scalar(low_h, low_s, low_v), Scalar(high_h, high_s, high_v), templ);

    ///With edge detection
    // Do edge detection on filtered image
    // DetectEdges(raw, templ);
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

__global__ void LocateNeedle (const Mat& img, const Mat& templ, match *bestMatch){
    //Loop over all scales
    double scale = min_scale / 100.0;
    for(int i = 0; i < ceil((max_scale - min_scale) / scale_increment); ++i){
        scale += ((double) scale_increment) / 100.0;
        Mat resized;

        //If scaling up, use inter-linear interpolation (as recommended by opencv documentation)
        // if(scale > 1){
        //     resize(templ, resized, Size(), scale, scale, INTER_LINEAR);
        // }
        // //If scaling down, use inter-area interpolation
        // else {
        //     resize(templ, resized, Size(), scale, scale, INTER_AREA);
        // }

        //Use inter-linear in all cases (is faster than inter_area, similar results)
        resize(templ, resized, Size(), scale, scale, INTER_LINEAR);
        
        ///Loop over all rotations
        double rot_angle = 0;   
        for(int j = 0; j < ceil(max_rotation / angle_increment); ++j){
            rot_angle += angle_increment;
            //Rotate
            Mat rot_templ;
            RotateTemplate(rot_angle, resized, rot_templ);

            //Match rotated template to image
            MatchImageToTemplate(img, rot_templ, bestMatch, rot_angle, scale, false);
        }
    }
}

//With HSV filtering, no longer seems necessary
void DetectEdges(const Mat& img, Mat& dst){
    Mat detected_edges;
    //What should the kernel size be?
    GaussianBlur( img, detected_edges, Size(3,3), 0);
    Canny( detected_edges, dst, lowThreshold, lowThreshold * ratio, kernel_size );
}

__device__ void MatchImageToTemplate(const Mat& img, const Mat& templ, match* bestMatch, double angle, double scale, bool use_gpu){

    /// Create the result matrix
    Mat result;
    int result_cols =  img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;
    result.create( result_rows, result_cols, CV_32FC1 );

    if(use_gpu){
        //Using cuda GPU accelerated matching
        //Currently Slower than using non-accelerated (likely due to overhead of copying the imgs and templates from cpu to gpu and back every time)
        cuda::setDevice(0);
        cuda::GpuMat img_gpu(img), templ_gpu(templ), result_gpu;
        Ptr<cv::cuda::TemplateMatching> matcher = cuda::createTemplateMatching(img.type(), CV_TM_CCOEFF);
        matcher->match(img_gpu, templ_gpu, result_gpu);
        result_gpu.download(result);
    }
    else{
        //Match using TM_CCOEFF
        matchTemplate(img, templ, result, TM_CCOEFF);
    }

    
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

__device__ void RotateTemplate(double angle, const Mat &src, Mat &dst){
    /// get rotation matrix for rotating the image around its center in pixel coordinates
    Point2d center((src.cols-1)/2.0, (src.rows-1)/2.0);

    Mat rot = getRotationMatrix2D(center, angle, 1.0);
    /// determine bounding rectangle, center not relevant
    Rect2d bbox = RotatedRect(Point2d(), src.size(), angle).boundingRect2f();
    /// adjust transformation matrix
    rot.at<double>(0,2) += bbox.width/2.0 - src.cols/2.0;
    rot.at<double>(1,2) += bbox.height/2.0 - src.rows/2.0;

    warpAffine(src, dst, rot, bbox.size());
}