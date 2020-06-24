#include "needle_image.h"
#include "needle_template.h"
#include "template_match.h"
#include "needle_pose.h"
#include "pfc_initializer_constants.h"

class PfcInitializer
{
public:
    int pose_id;
    NeedleImage left_image;
    NeedleImage right_image;
    NeedleTemplate templ;
    TemplateMatch match_l;
    TemplateMatch match_r;
    NeedlePose pose;

    PfcInitializer(string left_image_path, string right_image_path, int pose_id, pfc::match_params params)
        : left_image(left_image_path), right_image(right_image_path), 
            templ(pfc::templ_path, pfc::initial_rect, pfc::origin, pfc::initial_rotation, params), 
            pose_id(pose_id)
    {}


    // PfcInitializer(string left_image_path, string right_image_path, int pose_id, pfc::match_params params)
    //     : left_image(left_image_path), right_image(right_image_path), 
    //         templ(pfc::templ_path, pfc::initial_rect, pfc::origin.x, pfc::origin.y, pfc::initial_rotation, params), 
    //         pose_id(pose_id)
    // {}

    NeedlePose computeNeedlePose();
    vector<double> scorePoseEstimation();
    void drawNeedleOrigin(cv::Mat&img, TemplateMatch* match, cv::Scalar color);
    void displayResults();
    NeedlePose readTruePoseFromCSV();
    std::vector<string> getResultsVector();
};