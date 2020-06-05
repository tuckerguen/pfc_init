#include <opencv2/imgproc.hpp>

//A template match
struct match {
    double maxVal; //max matching value assigned by opencv templatematch()
    double angle; //angle matched at
    double scale; //scale matched at
    cv::Rect maxRect; //location of match
    cv::Mat templ; //template used to match
    cv::Mat result; //result image from opencv templatematch()
};

int HandleArguments(int argc, char** argv, std::string *image_id, std::string* image_type);
double PFCInit(std::string left_image_path, std::string right_image_path, bool display_results);
void DisplayResults(std::string left_image_path, std::string right_image_path, cv::Mat& raw_l,  cv::Mat& raw_r, match bestMatch_l, match bestMatch_r, double t);
void InitNeedleImage(std::string path, cv::Mat& img);
void InitTemplate(std::string path, cv::Mat& templ);
void DetectEdges(const cv::Mat& img, cv::Mat& dst);
void LocateNeedle (const cv::Mat& img, const cv::Mat& templ, match *bestMatch);
void RotateTemplate(double angle, const cv::Mat &src, cv::Mat &dst);
void MatchImageToTemplate(const cv::Mat& img, const cv::Mat& templ, match* bestMatch, double angle, double scale, bool use_gpu);
void DrawMatch(cv::Mat &src, cv::Rect match, cv::Scalar color);
void PrintResultsForImage(match *match, std::string side);
cv::Rect GetTrueMatchFromMeta(std::string img_path);
cv::Point3d DeProjectPoints(const cv::Mat& img_l, const cv::Mat& img_r, const match* match_l, const match* match_r);
double IntersectionOverUnion(const cv::Rect *ground, const cv::Rect *data);