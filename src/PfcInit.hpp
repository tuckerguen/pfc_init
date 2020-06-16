#include "NeedleImage.hpp"
#include "NeedleTemplate.hpp"
#include "TemplateMatch.hpp"
#include "NeedlePose.hpp"

class PfcInit
{
public:
    NeedleImage left_image;
    NeedleImage right_image;
    NeedleTemplate templ;
    TemplateMatch match_l;
    TemplateMatch match_r;
    NeedlePose pose;

    PfcInit(string left_image_path, string right_image_path, string template_path)
        : left_image(left_image_path), right_image(right_image_path), templ(template_path, cv::Rect2i(287, 205, 105, 56), 52, 4, 180)
    {}

    NeedlePose computeNeedlePose();
    cv::Point3d DeProjectPoints(const TemplateMatch* match_l, const TemplateMatch* match_r);
    void scorePoseEstimation(NeedlePose pose, int pose_id);
    void drawNeedleOrigin(cv::Mat&img, TemplateMatch* match, cv::Scalar color);
    void displayResults();
};
