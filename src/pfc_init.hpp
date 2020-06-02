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

int HandleArguments(int argc, char** argv, string *image_id, string* image_type, string* left_image_path, string* right_image_path);
int PFCInit(string left_image_path, string right_image_path);
void InitNeedleImage(string path, Mat& img);
void InitTemplate(string path, Mat& templ);
void DetectEdges(const Mat& img, Mat& dst);
void LocateNeedle (const Mat& img, const Mat& templ, match *bestMatch);
void RotateTemplate(double angle, const Mat &src, Mat &dst);
void MatchImageToTemplate(const Mat& img, const Mat& templ, match* bestMatch, double angle, double scale, bool use_gpu);
void DrawMatch(Mat& src, match* match);
void PrintResultsForImage(match *match, string side);
Point3d DeProjectPoints(const Mat& img_l, const Mat& img_r, const match* match_l, const match* match_r);
double IntersectionOverUnion(const Rect *ground, const Rect *data);