///
/// Created by tucker on 5/21/20.
///
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <iostream>
#include <unistd.h>
#include "pfc_init.hpp"

using namespace std;
using namespace cv;

//const string template_img_path = "../imgs/raw_l_b.png";
const string template_img_path = "../imgs/raw_l_b.png";

//Canny edge detection parameters
int lowThreshold = 8;
const int ratio = 3;
const int kernel_size = 3;

// Rotation parameters
const double max_rotation = 360;   //Max number of degrees to rotate template
const double angle_increment = 10; //Number of degrees to rotate template each iteration

// Scaling parameters
const int min_scale = 50;         //minimum template scale to try to match (in %)
const int max_scale = 150;        //maximum template scale to try to match (in %)
const double scale_increment = 1; //% scale to increase by on each iteration

//HSV Filtering Parameters
const int low_h = 0, high_h = 5;
const int low_s = 0, high_s = 0;
const int low_v = 0, high_v = 140;

bool use_gpu;

int main(int argc, char *argv[])
{
    string image_id = "a", image_type = "raw";

    if (!HandleArguments(argc, argv, &image_id, &image_type))
        return 1;

    string left_image_path = "../imgs/" + image_type + "_l_" + image_id + ".png";
    string right_image_path = "../imgs/" + image_type + "_r_" + image_id + ".png";

    cout << "left img: " << left_image_path << endl;
    cout << "rght img: " << right_image_path << endl;
    cout << "type: " << image_type << ", use_gpu: " << (use_gpu == true ? "yes" : "no") << endl;

    PFCInit(left_image_path, right_image_path, true);
    return 0;
}

int HandleArguments(int argc, char **argv, string *image_id, string *image_type)
{
    if (argc == 4)
    {
        *image_id = argv[1];
        if (!strcmp(argv[2], "raw") || !strcmp(argv[2], "vessel"))
        {
            *image_type = argv[2];
        }
        else
        {
            cout << argv[2] << " is not a valid image type" << endl;
            return 0;
        }
        if (!strcmp(argv[3], "1") || !strcmp(argv[3], "0"))
        {
            int val = atoi(argv[3]);
            use_gpu = val == 1;
        }
        else
        {
            cout << argv[3] << " use_gpu only assumes value 1 or 0" << endl;
        }
    }
    else if (argc == 3)
    {
        *image_id = argv[1];
        if (!strcmp(argv[2], "raw") || !strcmp(argv[2], "vessel"))
        {
            *image_type = argv[2];
        }
        else
        {
            cout << argv[2] << " is not a valid image type" << endl;
            return 0;
        }
    }
    else if (argc == 2)
    {
        *image_id = argv[1];
    }
    else if (argc != 1)
    {
        cout << "Use: ./pfc_init <image_id ('a','marked', etc)> <image_type (raw, vessel)> <use_gpu (1,0)>" << endl;
        return 0;
    }
    return 1;
}

double PFCInit(string left_image_path, string right_image_path, bool display_results)
{
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
        Rect(0,0,0,0),
        templ,
        init_result};

    //Init best match info for right image
    match bestMatch_r = {
        -DBL_MAX,
        0.0,
        0.0,
        Rect(0,0,0,0),
        templ,
        init_result};

    //Run localization algorithm on left and right images
    LocateNeedle(img_l, templ, &bestMatch_l);
    LocateNeedle(img_r, templ, &bestMatch_r);
    
    // Point3d location = DeProjectPoints(img_l, img_r, &bestMatch_l, &bestMatch_r);
    // cout << "location: (" << location.x << ", " << location.y << ", " << location.z << endl;

    t = ((double)getTickCount() - t) / getTickFrequency();

    if(display_results)
        DisplayResults(left_image_path, right_image_path, raw_l, raw_r, bestMatch_l, bestMatch_r, t);

    //Record time
    return t;
}

void DisplayResults(string left_image_path, string right_image_path, Mat& raw_l,  Mat& raw_r, match bestMatch_l, match bestMatch_r, double t){
    cout << endl <<  "Times passed in seconds: " << t << endl;
    cout << "--------------------------------------" << endl;
    
    ///Define viewing windows
    namedWindow(left_image_path, WINDOW_AUTOSIZE);
    namedWindow(right_image_path, WINDOW_AUTOSIZE);
    namedWindow("best template", WINDOW_AUTOSIZE);
    namedWindow("result", WINDOW_AUTOSIZE);

    //Draw algorithm matches on originals and result images
    Scalar white = Scalar::all(255);
    Scalar cyan = Scalar(255, 255, 0);
    DrawMatch(raw_l, bestMatch_l.maxRect, white);
    DrawMatch(bestMatch_l.result, bestMatch_l.maxRect, white);
    DrawMatch(raw_r, bestMatch_r.maxRect, white);
    DrawMatch(bestMatch_r.result, bestMatch_r.maxRect, white);

    //Draw the ground truth matches on images
    Rect left_truth = GetTrueMatchFromMeta(left_image_path);
    Rect right_truth = GetTrueMatchFromMeta(right_image_path);
    DrawMatch(raw_l, left_truth, cyan);
    DrawMatch(raw_r, right_truth, cyan);


    //Print match information
    PrintResultsForImage(&bestMatch_l, left_image_path);
    PrintResultsForImage(&bestMatch_r, right_image_path);

    //Display images
    imshow(left_image_path, raw_l);
    imshow(right_image_path, raw_r);
    imshow("best template", bestMatch_l.templ);
    imshow("result", bestMatch_l.result);

    waitKey(0);
}

void InitNeedleImage(string path, Mat &img)
{
    //Load camera image to match
    Mat raw, filtered, img_HSV;
    raw = imread(path, IMREAD_COLOR);
    if (!raw.data)
    {
        cerr << "Loading image failed" << endl;
        exit(0);
    }

    // Filter by color for needle
    cvtColor(raw, img_HSV, COLOR_BGR2HSV);
    inRange(img_HSV, Scalar(low_h, low_s, low_v), Scalar(high_h, high_s, high_v), img);

    // Do edge detection on filtered image
    // DetectEdges(raw, img);
}

void InitTemplate(string path, Mat &templ)
{
    //Load camera image to match
    Mat raw, filtered, img_HSV;
    raw = imread(path, IMREAD_COLOR);
    if (!raw.data)
    {
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

//With HSV filtering, no longer seems necessary
void DetectEdges(const Mat &img, Mat &dst)
{
    Mat detected_edges;
    //What should the kernel size be?
    GaussianBlur(img, detected_edges, Size(3, 3), 0);
    Canny(detected_edges, dst, lowThreshold, lowThreshold * ratio, kernel_size);
}

void LocateNeedle(const Mat &img, const Mat &templ, match *bestMatch)
{
    //Loop over all scales
    double scale = min_scale / 100.0;
    for (int i = 0; i < ceil((max_scale - min_scale) / scale_increment); ++i)
    {
        scale += ((double)scale_increment) / 100.0;
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
        for (int j = 0; j < ceil(max_rotation / angle_increment); ++j)
        {
            rot_angle += angle_increment;
            //Rotate
            Mat rot_templ;
            RotateTemplate(rot_angle, resized, rot_templ);

            //Match rotated template to image
            MatchImageToTemplate(img, rot_templ, bestMatch, rot_angle, scale, false);
        }
    }
}

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

void MatchImageToTemplate(const Mat &img, const Mat &templ, match *bestMatch, double angle, double scale, bool use_gpu)
{

    /// Create the result matrix
    Mat result;
    int result_cols = img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;
    result.create(result_rows, result_cols, CV_32FC1);

    if (use_gpu)
    {
        //Using cuda GPU accelerated matching
        //Currently Slower than using non-accelerated (likely due to overhead of copying the imgs and templates from cpu to gpu and back every time)
        cuda::setDevice(0);
        cuda::GpuMat img_gpu(img), templ_gpu(templ), result_gpu;
        Ptr<cv::cuda::TemplateMatching> matcher = cuda::createTemplateMatching(img.type(), CV_TM_CCOEFF);
        matcher->match(img_gpu, templ_gpu, result_gpu);
        result_gpu.download(result);
    }
    else
    {
        //Match using TM_CCOEFF
        matchTemplate(img, templ, result, TM_CCOEFF);
    }

    /// Localizing the best match with minMaxLoc
    double minVal;
    double maxVal;
    Point minLoc;
    Point maxLoc;
    Point matchLoc;
    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

    //If the new match is better than our previous best, record it
    // cout << ": " << maxVal << endl;
    if (bestMatch->maxVal < maxVal)
    {
        bestMatch->maxVal = maxVal;
        bestMatch->angle = angle;
        bestMatch->scale = scale;
        bestMatch->maxRect.x = maxLoc.x;
        bestMatch->maxRect.y = maxLoc.y;
        bestMatch->maxRect.width = templ.cols;
        bestMatch->maxRect.height = templ.rows;
        bestMatch->templ = templ;
        bestMatch->result = result;
    }
}

void DrawMatch(Mat &src, Rect match, Scalar color)
{
    int line_weight = 1;
    rectangle(src, match, color, line_weight, 8, 0);
}

void PrintResultsForImage(match *match, string img_path)
{
    cout << endl << "--------------------------------------" << endl;
    cout << "Results for: " << img_path << endl;
    if (match->angle > 180.0)
    {
        match->angle = -(match->angle - 360.0);
    }
    cout << "Location: (" << match->maxRect.x << ", " << match->maxRect.y << ")" << endl;
    cout << "Size: " << "width: " << match->maxRect.width << ", height: " << match->maxRect.height << endl;
    cout << "Yaw: degrees = " << match->angle << ", radians = " << match->angle * (3.1415926535 / 180.0) << endl;
    cout << "Scale: " << match->scale << endl;
        
    Rect truth = GetTrueMatchFromMeta(img_path);
    double score  = IntersectionOverUnion(&truth, &match->maxRect);;    
    cout << "IoU Score : " << score << endl;    
}

Rect GetTrueMatchFromMeta(string img_path){
    //read from metafile
    string meta_path = img_path;
    meta_path.erase(0, 8);
    meta_path.erase(meta_path.length() - 3, 3);
    meta_path = "../imgs/meta/" + meta_path + "meta";

    string data;
    ifstream meta_file;
    meta_file.open(meta_path.c_str());
    if(meta_file.good()){
        getline(meta_file, data);
        meta_file.close();

        istringstream ss(data);
        int tx, ty, tw, th;
        ss >> ty >> tx >> tw >> th;
        return Rect (tx, ty, tw, th);
    }
    else {
        cout << "unable to open meta_file at: " << meta_path << endl;
        return Rect(-1,-1,-1,-1);
    }
}

Point3d DeProjectPoints(const Mat &img_l, const Mat &img_r, const match *match_l, const match *match_r)
{
    //construct the output mat:
    Mat results(4, 1, CV_64FC1);
    Mat p_l(2, 1, CV_64FC1);
    Mat p_r(2, 1, CV_64FC1);

    p_l.at<double>(0) = match_l->maxRect.x;
    p_l.at<double>(1) = match_l->maxRect.y;

    p_r.at<double>(0) = match_r->maxRect.x;
    p_r.at<double>(1) = match_r->maxRect.y;

    //Camera intrinsic matrices (placeholders for now)
    double fx_l = 5.749;
    double fy_l = 5.999;
    double rh_l = img_l.cols / 2.0;
    double rv_l = img_l.rows / 2.0;

    double fx_r = 4.500;
    double fy_r = 4.680;
    double rh_r = img_r.cols / 2.0;
    double rv_r = img_r.rows / 2.0;

    Mat P_l = (Mat_<double>(3, 4) << fx_l, 0, rh_l, 0,
               0, fy_l, rv_l, 0,
               0, 0, 0, 1);

    Mat P_r = (Mat_<double>(3, 4) << fx_r, 0, rh_r, 0,
               0, fy_r, rv_r, 0,
               0, 0, 0, 1);

    // triangulatePoints(P_l, P_r, p_l, p_r, results);

    Point3d result;

    result.x = results.at<double>(0) / results.at<double>(3);
    result.y = results.at<double>(1) / results.at<double>(3);
    result.z = results.at<double>(2) / results.at<double>(3);

    return result;
}

double IntersectionOverUnion(const Rect *ground, const Rect *data)
{
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

    if (width < 0 || height < 0){
        cout << "width or height < 0" << endl;
        return 0.0;
    }

    overlap_area = width * height;

    //Combined area
    rect_union = ground->area() + data->area() - overlap_area;    
    
    return (double)overlap_area / (double)rect_union;
}
