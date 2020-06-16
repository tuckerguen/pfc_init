#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Dense>

//A template match
struct Match {
    double maxVal; //max matching value assigned by opencv templatematch()
    double angle; //angle matched at
    double scale; //scale matched at
    cv::Rect maxRect; //location of match
    cv::Mat templ; //template used to match
    cv::Mat result; //result image from opencv templatematch()
};

struct Pose {
    cv::Point3d location;
    Eigen::Vector4f orientation;
};

double PFCInit(std::string left_image_path, std::string right_image_path, bool display_results);
void DisplayResults(std::string left_image_path, std::string right_image_path, cv::Mat& raw_l,  cv::Mat& raw_r, Match bestMatch_l, Match bestMatch_r, cv::Point3d location, double t);
void InitNeedleImage(std::string path, cv::Mat& img);
void InitTemplate(std::string path, cv::Mat& templ);
void DetectEdges(const cv::Mat& img, cv::Mat& dst);
void LocateNeedle (const cv::Mat& img, const cv::Mat& templ, Match *bestMatch);
void RotateTemplate(double angle, const cv::Mat &src, cv::Mat &dst);
void MatchImageToTemplate(const cv::Mat& img, const cv::Mat& templ, Match* bestMatch, double angle, double scale, bool use_gpu);
void DrawMatch(cv::Mat &src, Match* match, cv::Scalar color);
void PrintResultsForImage(Match *match, std::string side);
Eigen::Vector4f RPYtoQuat(double roll, double pitch, double yaw);
cv::Point3d DeProjectPoints(const Match* match_l, const Match* match_r);
Pose ReadTruePoseFromCSV(int pose_id);
