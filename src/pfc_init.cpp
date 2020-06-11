///
/// Created by tucker on 5/21/20.
///
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <opencv2/sfm/triangulation.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <unistd.h>
#include "pfc_init.hpp"

using namespace std;
using namespace cv;
using namespace Eigen;

//const string template_img_path = "../imgs/raw_l_b.png";
const string template_img_path = "../imgs/raw/0_l_c_fatty.png";

//Canny edge detection parameters
int lowThreshold = 8;
const int ratio = 3;
const int kernel_size = 3;

// Rotation parameters
const double max_rotation = 360;   //Max number of degrees to rotate template
const double angle_increment = 10; //Number of degrees to rotate template each iteration

// Scaling parameters
const int min_scale = 95;         //minimum template scale to try to match (in %)
const int max_scale = 110;        //maximum template scale to try to match (in %)
const double scale_increment = 1; //% scale to increase by on each iteration

//HSV Filtering Parameters
const int low_h = 0, high_h = 5;
const int low_s = 0, high_s = 0;
const int low_v = 0, high_v = 140;

//Needle position offset (move coordinate frame from top left 
//to point bisecting line between ends)
const int needle_origin_offset_x = 52;
const int needle_origin_offset_y = 4;
const int template_width_0 = 105;
const int template_height_0 = 56;

//Degree 2 radians conversion constant
const double deg2rad = M_PI / 180.0;

bool use_gpu;

int main(int argc, char *argv[])
{
    string image_id = "marked";
    string image_num = "8";

    if (!HandleArguments(argc, argv, &image_id))
        return 1;

    string left_image_path = "../imgs/raw/" + image_num + "_l_c_" + image_id + ".png";
    string right_image_path = "../imgs/raw/" + image_num + "_r_c_" + image_id + ".png";

    cout << "left img: " << left_image_path << endl;
    cout << "rght img: " << right_image_path << endl;
    cout << "type: " << ", use_gpu: " << (use_gpu == true ? "yes" : "no") << endl;

    PFCInit(left_image_path, right_image_path, true);
    return 0;
}

int HandleArguments(int argc, char **argv, string *image_id)
{
    if (argc == 4)
    {
        *image_id = argv[1];
        if (!strcmp(argv[2], "raw") || !strcmp(argv[2], "vessel"))
        {
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
    
    Point3d location = DeProjectPoints(&bestMatch_l, &bestMatch_r);

    t = ((double)getTickCount() - t) / getTickFrequency();

    if(display_results)
        DisplayResults(left_image_path, right_image_path, raw_l, raw_r, bestMatch_l, bestMatch_r, location, t);

    //Record time
    return t;
}

void DisplayResults(string left_image_path, string right_image_path, Mat& raw_l,  Mat& raw_r, match bestMatch_l, match bestMatch_r, Point3d location, double t){
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
    DrawMatch(raw_l, &bestMatch_l, white);
    DrawMatch(bestMatch_l.result, &bestMatch_l, white);
    DrawMatch(raw_r, &bestMatch_r, white);
    DrawMatch(bestMatch_r.result, &bestMatch_r, white);

    cout << "match angle: " << bestMatch_l.angle << endl;
    //Print match information
    PrintResultsForImage(&bestMatch_l, left_image_path);
    PrintResultsForImage(&bestMatch_r, right_image_path);

    //Final 3D coordinate location
    cout << "3D Location: (" << location.x << ", " << location.y << ", " << location.z << ")" << endl;

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
    Mat raw, filtered, img_HSV, bw;
    raw = imread(path, IMREAD_COLOR);
    if (!raw.data)
    {
        cerr << "Loading image: " << path << " failed" << endl;
        exit(0);
    }

    //Crop template image to just needle
    Rect r(287, 205, template_width_0, template_height_0);
    raw = raw(r);

    ///With HSV filtering
    // Filter by HSV for needle
    cvtColor(raw, img_HSV, COLOR_BGR2HSV);
    inRange(img_HSV, Scalar(low_h, low_s, low_v), Scalar(high_h, high_s, high_v), bw);
    RotateTemplate(180, bw, templ);

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

void DrawMatch(Mat &src, match* match, Scalar color)
{
    int line_weight = 1;
    rectangle(src, match->maxRect, color, line_weight, 8, 0);

    //Apply the same rotation matrix used to rotate the template to the base template origin offset
    Mat needle_origin_offset_mat = (Mat_<double>(3,1) << needle_origin_offset_x, needle_origin_offset_y, 1);
    Point2d center((template_width_0 - 1) / 2.0, (template_height_0 - 1) / 2.0);
    Mat rot = getRotationMatrix2D(center, match->angle, match->scale);
    Rect2d bbox = RotatedRect(Point2d(), Size2d(template_width_0, template_height_0), match->angle).boundingRect2f();
    /// adjust transformation matrix
    rot.at<double>(0, 2) += bbox.width / 2.0 - template_width_0 / 2.0;
    rot.at<double>(1, 2) += bbox.height / 2.0 - template_height_0 / 2.0;
    Mat needle_origin_offset_rot = rot * needle_origin_offset_mat;

    circle(src, 
            Point(match->maxRect.x + needle_origin_offset_rot.at<double>(0), 
                  match->maxRect.y + needle_origin_offset_rot.at<double>(1)),
            0.1, color, 1, 8, 0);
}

void PrintResultsForImage(match *match, string img_path)
{
    cout << endl << "--------------------------------------" << endl;
    cout << "Results for: " << img_path << endl;
    // if (match->angle > 180.0)
    // {
    //     match->angle = -(match->angle - 360.0);
    // }
    cout << "Pixel Location: (" << match->maxRect.x << ", " << match->maxRect.y << ")" << endl;
    cout << "Size: " << "width: " << match->maxRect.width << ", height: " << match->maxRect.height << endl;
    cout << "Yaw: degrees = " << match->angle << ", radians = " << match->angle * deg2rad << endl;
    cout << "Scale: " << match->scale << endl;

    Vector4f orientation = RPYtoQuat(0,0, -match->angle);
    cout << "3D Orientation: " << endl;
    cout << "Quaternion: (" << orientation.x() << ", " << orientation.y() << ", " << orientation.z() << ", " << orientation.w() << ")" << endl;
    cout << "Euler angles: yaw= " << match->angle << endl;
}

Vector4f RPYtoQuat(double roll, double pitch, double yaw){
    double roll_radians = deg2rad * roll;
    double pitch_radians = deg2rad * pitch;
    double yaw_radians = deg2rad * yaw;
    Quaternionf q;
    q = AngleAxisf(roll_radians, Vector3f::UnitX())
        * AngleAxisf(pitch_radians, Vector3f::UnitY())
        * AngleAxisf(yaw_radians, Vector3f::UnitZ());
    return q.coeffs();
}

// vector<double> GetNeedleOriginOffset(double rotation

Point3d DeProjectPoints(const match *match_l, const match *match_r)
{
    Mat p_l(2, 1, CV_64FC1);
    Mat p_r(2, 1, CV_64FC1);

    cout << "cols: " << match_l->templ.cols << ", rows: " << match_l->templ.rows << "size: " << match_l->templ.size() 
    << "angle: " << match_l->angle << "scale: " << match_l->scale << endl;

    //Apply the same rotation matrix used to rotate the template to the base template origin offset
    Mat needle_origin_offset_mat = (Mat_<double>(3,1) << needle_origin_offset_x, needle_origin_offset_y, 1);
    Point2d center((template_width_0 - 1) / 2.0, (template_height_0 - 1) / 2.0);
    Mat rot = getRotationMatrix2D(center, match_l->angle, match_l->scale);
    Rect2d bbox = RotatedRect(Point2d(), Size2d(template_width_0, template_height_0), match_l->angle).boundingRect2f();
    /// adjust transformation matrix
    rot.at<double>(0, 2) += bbox.width / 2.0 - template_width_0 / 2.0;
    rot.at<double>(1, 2) += bbox.height / 2.0 - template_height_0 / 2.0;
    Mat needle_origin_offset_rot = rot * needle_origin_offset_mat;
    cout << "N_O_O: " << needle_origin_offset_rot << endl;

    p_l.at<double>(0) = match_l->maxRect.x + needle_origin_offset_rot.at<double>(0);
    p_l.at<double>(1) = match_l->maxRect.y + needle_origin_offset_rot.at<double>(1);

    p_r.at<double>(0) = match_r->maxRect.x + needle_origin_offset_rot.at<double>(0);
    p_r.at<double>(1) = match_r->maxRect.y + needle_origin_offset_rot.at<double>(1);

    Mat P_r = (Mat_<double>(3, 4) << 662.450355616388, 0.0, 320.5, -3.31225177808194, 
                                     0.0, 662.450355616388, 240.5, 0.0,
                                     0.0, 0.0, 1.0, 0.0);

    Mat P_l = (Mat_<double>(3, 4) << 662.450355616388, 0.0, 320.5, 0.0, 
                                     0.0, 662.450355616388, 240.5, 0.0, 
                                     0.0, 0.0, 1.0, 0.0);


    Mat results;

    vector<Mat> points;
    points.push_back(p_l);
    points.push_back(p_r);

    vector<Mat> projections;
    projections.push_back(P_l);
    projections.push_back(P_r);

    sfm::triangulatePoints(points, projections, results);

    Point3d result;

    result.x = results.at<double>(0);
    result.y = results.at<double>(1);
    result.z = results.at<double>(2);

    Point3d truth(0.0130668,-0.00752352,0.159038);
    double dist = norm(result - truth);
    cout << "Euclidean Distance from Truth: " << dist << endl;
    cout << "Diff X: " << truth.x - result.x << " meters" << endl;
    cout << "Diff Y: " << truth.y - result.y << " meters" << endl;
    cout << "Diff Z: " << truth.z - result.z << " meters" << endl;

    return result;
}
