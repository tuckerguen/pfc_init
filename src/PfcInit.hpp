

#include "NeedleImage.hpp"
#include "NeedleTemplate.hpp"
#include "TemplateMatch.hpp"
#include "NeedlePose.hpp"
#include "PfcInitConstants.hpp"

class PfcInit
{
public:
    int pose_id;
    NeedleImage left_image;
    NeedleImage right_image;
    NeedleTemplate templ;
    TemplateMatch match_l;
    TemplateMatch match_r;
    NeedlePose pose;

    PfcInit(string left_image_path, string right_image_path, int pose_id, pfc::match_params params)
        : left_image(left_image_path), right_image(right_image_path), 
            templ(pfc::templ_path, pfc::initial_rect, pfc::origin_offset_x, pfc::origin_offset_y, pfc::initial_rotation, params), 
            pose_id(pose_id)
    {}

    NeedlePose computeNeedlePose();
    cv::Point3d DeProjectPoints(TemplateMatch* match_l, TemplateMatch* match_r);
    vector<double> scorePoseEstimation();
    void drawNeedleOrigin(cv::Mat&img, TemplateMatch* match, cv::Scalar color);
    void displayResults();
    NeedlePose readTruePoseFromCSV();
    std::vector<string> getResultsVector();
};
    cv::Mat getRotatedOrigin(double angle, double scale, NeedleTemplate* templ);
