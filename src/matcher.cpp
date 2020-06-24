#include "needle_image.h"
#include "needle_template.h"
#include "template_match.h"
#include "matcher.h"

// Template match template on base image over range of scales and rotations
TemplateMatch matchOverScaleAndRotation(const cv::Mat& img, const NeedleTemplate* templ)
{
    double min_rot = templ->params.min_rotation;
    double max_rot = templ->params.max_rotation;
    double rot_inc = templ->params.rotation_increment;
    double min_scl = templ->params.min_scale;
    double max_scl = templ->params.max_scale;
    double scl_inc = templ->params.scale_increment;

    // initialize best match with minimum rot, scale, score
    TemplateMatch bestMatch(min_rot, -DBL_MAX, min_scl);
    
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
            matchAndCompare(img, rot_templ, &bestMatch, rot_angle, scale);
        }
    }

    return bestMatch;
}

// Run template match and store results if this match is better than bestMatch
void matchAndCompare(const cv::Mat &img, const cv:: Mat& templ, TemplateMatch *bestMatch, double angle, double scale)
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

    //If the new match is better than our previous best, record it
    if (bestMatch->score < maxVal)
    {
        bestMatch->score = maxVal;
        if(angle > 180.0)
            bestMatch->angle = 360 - angle;
        else
            bestMatch->angle = -angle;
        bestMatch->scale = scale;
        bestMatch->rect.x = maxLoc.x;
        bestMatch->rect.y = maxLoc.y;
        bestMatch->rect.width = templ.cols;
        bestMatch->rect.height = templ.rows;
        bestMatch->result = result;
        bestMatch->templ = templ;
    }
}
