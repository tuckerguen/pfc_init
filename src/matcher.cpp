#include "needle_image.h"
#include "needle_template.h"
#include "template_match.h"
#include "matcher.h"

// Template match template on base image over range of scales and rotations
TemplateMatch matchOverScaleAndRotation(const cv::Mat& img, const NeedleTemplate* templ)
{
    // Note: this "params" setup isn't necessary for final implementation.
    // This is only used to "cheat" the system by allowing for known poses
    // to have their scale and rotation bounds preloaded from the 
    // PfcInitConstants.hpp file. In a final implementation, the 
    // rotation and scaling ranges and increments would just be
    // referenced directly from the PfcInitConstants.hpp file, or 
    // some other configuration file
    double min_rot = templ->params.min_rotation;
    double max_rot = templ->params.max_rotation;
    double rot_inc = templ->params.rotation_increment;
    double min_scl = templ->params.min_scale;
    double max_scl = templ->params.max_scale;
    double scl_inc = templ->params.scale_increment;

    // initialize best match with minimum rot, scale, score
    TemplateMatch best_match(min_rot, -DBL_MAX, min_scl);
    
    // Reduce scale from % to decimal
    double scale = min_scl / 100.0;
    // Loop over all scales
    for (int i = 0; i < ceil((max_scl - min_scl) / scl_inc); ++i)
    {
        scale += ((double)scl_inc) / 100.0;

        cv::Mat resized, rot_templ;

        //Use inter-linear in all cases (is faster than inter_area, similar results)
        cv::resize(templ->image, resized, cv::Size(), scale, scale, cv::INTER_LINEAR);

        // Loop over all rotations
        for (double rot_angle = min_rot; rot_angle < max_rot; rot_angle += rot_inc)
        {
            //Rotate template
            rotate(resized, rot_templ, rot_angle);

            //Match rotated template to image
            TemplateMatch new_match = match(img, rot_templ, rot_angle, scale);
     
            //If the new match is better than our previous best, record it
            if (best_match.score < new_match.score)
            {
                best_match = new_match;
            }
        }
    }

    return best_match;
}

// Run template match and store results if this match is better than bestMatch
TemplateMatch match(const cv::Mat &img, const cv:: Mat& templ, double angle, double scale)
{

    // Create the result matrix
    cv::Mat result;
    
    //Match using TM_CCOEFF_NORMED
    cv::matchTemplate(img, templ, result, cv::TM_CCOEFF_NORMED);

    /// Localizing the best match with minMaxLoc
    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;
    cv::Point matchLoc;
    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

    // Create template match 
    cv::Rect2i rect(maxLoc.x, maxLoc.y, templ.cols, templ.rows);

    TemplateMatch match = 
    {
        angle > 180 ? (360 - angle) : -angle, 
        maxVal,
        scale,
        rect,
        result,
        templ
    };

    return match;
}
