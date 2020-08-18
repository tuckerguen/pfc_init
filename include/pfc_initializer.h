#include "needle_image.h"
#include "needle_template.h"
#include "template_match.h"
#include "needle_pose.h"
#include "pfc_initializer_constants.h"
#include <queue>
#include <vector>

/**
 * @brief Class to localize needle from stereo images
 */
class PfcInitializer
{
private:
    /**
     * @brief Computes pose of needle from the left and right stereo images
     */
    void computeNeedlePose(bool multi_thread);

    /**
     * @brief Prints results of initilization to console
     */
    void displayResults(int pose_id);

public:
    /**
     * @brief Left stereo image
     */
    NeedleImage left_image;

    /**
     * @brief Right stereo image
     */
    NeedleImage right_image;

    /**
     * @brief Template used for template matching during localization
     */
    NeedleTemplate templ;

    /**
     * @brief Template match data for left image
     */
    vector<TemplateMatch> l_matches;

    /**
     * @brief Template match data for right image
     */
    vector<TemplateMatch> r_matches;

    /**
     * @brief Computed needle pose
     */
    std::vector<NeedlePose> poses;

    /**
     * @brief Constructor
     * TODO: Doc this once we decide on fields
     */
    PfcInitializer(string left_image_path, string right_image_path, pfc::match_params params)
        : left_image(left_image_path), right_image(right_image_path), 
            templ(pfc::templ_path, pfc::initial_rect, pfc::origin, pfc::initial_rotation, params) 
    {}

    /**
     * @brief Runs the initializer, stores the needle pose in initializer
     * 
     * @param print_results If function should print results to console
     * @param multi_thread Use threaded version of match
     */
    void run(bool print_results, bool multi_thread, int pose_id);

    /**
     * @brief Returns the results of initialization as a vector
     */
    std::vector<std::vector<string>> getResultsAsVector(int pose_id);
};