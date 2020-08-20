#include <future>
#include <sys/sysinfo.h>
#include <queue>
#include <algorithm>
#include "needle_image.h"
#include "needle_template.h"
#include "template_match.h"
#include "matcher.h"
#include <sstream>
#include <iterator>

template <typename T, typename C>
vector<T> pq_to_vector(priority_queue<T, vector<T>, C> pq)
{
    vector<T> vec;
    while(!pq.empty())
    {
        T elem = pq.top();
        vec.push_back(elem);
        pq.pop();
    }
    return vec;
}

// Template match template on base image over range of scales and rotations
vector<TemplateMatch> match(const cv::Mat& img, NeedleTemplate templ)
{
    // Note: this "params" setup isn't necessary for final implementation.
    // This is only used to "cheat" the system by allowing for known poses
    // to have their scale and rotation bounds preloaded from the 
    // PfcInitConstants.hpp file. In a final implementation, the 
    // rotation and scaling ranges and increments would just be
    // referenced directly from the PfcInitConstants.hpp file, or 
    // some other configuration file
    double min_a = templ.params.yaw_range.start, max_a = templ.params.yaw_range.end;
    double a_inc = templ.params.yaw_inc;
    double min_b = templ.params.pitch_range.start, max_b = templ.params.pitch_range.end;
    double b_inc = templ.params.pitch_inc;
    double min_y = templ.params.roll_range.start, max_y = templ.params.roll_range.end;
    double y_inc = templ.params.roll_inc;
    double min_z = templ.params.min_z, max_z = templ.params.max_z;
    double z_inc = templ.params.z_inc;

    // Priority queue to keep track of top n matches (as min heap to give
    // efficient check of minimum score in group)
    priority_queue<TemplateMatch, vector<TemplateMatch>, TemplateMatchComparator> best_matches;
    
    // For all z
    for (int sz = 0; sz < (max_z-min_z)/z_inc; ++sz)
    {
        double tz = (double)cv::getTickCount();

        // Calc z for this step
        float z = min_z + (z_inc * sz);
        cout << z << endl;
        // cout << z << endl;

        // Fpr all yaw (a values)
        for (double sa = 0; sa < (max_a-min_a)/a_inc; ++sa)
        {
            double ta = (double)cv::getTickCount();
            float a = min_a + (a_inc * sa); 

            //For all pitch (b values)
            for (double sb = 0; sb < (max_b-min_b)/b_inc; ++sb)
            {   
                double tb= (double)cv::getTickCount();
                float b = min_b + (b_inc * sb);
                
                // For all roll (y values)
                for (double sy = 0; sy < (max_y-min_y)/y_inc; ++sy)
                {
                    float y = min_y + (y_inc * sy);
                    
                    // Generate Template
                    templ.GenerateTemplate(z, a, b, y);
                    // cout << a << ", " << b << ", " << y << endl;

                    //Match rotated template to image
                    TemplateMatch new_match = getMatch(img, templ.image);
                    // Store other match details
                    new_match.z = z;
                    // TODO: Check neg/pos direction for pitch/roll 
                    new_match.yaw = a > 180 ? (a - 360) : -a;
                    new_match.pitch = b;
                    new_match.roll = y;
                    // Offset the origin to sit in the correct location of the final image
                    new_match.origin = templ.origin + cv::Point2d(new_match.rect.x, new_match.rect.y);

                    // If queue not full
                    if (best_matches.size() < templ.params.num_matches)
                    {
                        // Insert match
                        best_matches.push(new_match);
                    }
                    // If queue full and score is better than min score
                    else if (new_match.score > best_matches.top().score)
                    {
                        // Delete the smallest score match
                        best_matches.pop();
                        // Add the new better match
                        best_matches.push(new_match);
                    }
                    // Otherwise reject the match since it isn't in the top n scores
                }
                // tb = ((double)cv::getTickCount() - tb) / cv::getTickFrequency();
                // cout << "b step time: " << tb << " s" << endl;
            }
            // ta = ((double)cv::getTickCount() - ta) / cv::getTickFrequency();
            // cout << "a step time: " << ta << " s" << endl;
        }
        tz = ((double)cv::getTickCount() - tz) / cv::getTickFrequency();
        cout << "Z step time: " << tz << " s" << endl;
    }

    return pq_to_vector(best_matches);
}

// Equivalent to match but splitting scale range amongst # of parallel threads
vector<TemplateMatch> matchThreaded(const cv::Mat& img, NeedleTemplate templ)
{
    // Init thread matches and thread return values (futures)
    priority_queue<TemplateMatch, vector<TemplateMatch>, TemplateMatchComparator> best_matches;
    std::vector<std::future<vector<TemplateMatch>>> futures;

    // Get scale range
    // double min_scl = templ.params.min_scale;
    // double max_scl = templ.params.max_scale;
    // double scl_inc = templ.params.scale_increment;

    // // Calc num threads and scale increment between sequential threads
    // int num_threads = get_nprocs();
    
    // double thread_inc = ((max_scl - min_scl) / (double)num_threads);
    
    // // Account# intervals in range < # number threads
    // if (thread_inc < 1){
    //     num_threads = max_scl - min_scl;
    //     thread_inc = 1;
    // }

    // // Loop to launch threads
    // for(int i = 0; i < num_threads; i++)
    // {
    //     // Compute thread range
    //     templ.params.min_scale = (min_scl + i * thread_inc);
    //     templ.params.max_scale = min_scl + (i+1) * thread_inc;
    //     // Except first, shift range to avoid overlap
    //     if(i != 0){
    //         templ.params.min_scale += scl_inc;
    //     }

    //     // Collect future
    //     futures.push_back(std::async(launch::async, match, img, templ));
    // }

    // // Extract matches from futures
    // for (auto &fut : futures) {
    //     vector<TemplateMatch> thread_matches = fut.get();
        
    //     for(int i = 0; i < thread_matches.size(); i++)
    //     {
    //         TemplateMatch m = thread_matches.at(i);
    //         // If queue not full
    //         if (best_matches.size() < templ.params.num_matches)
    //         {
    //             // Insert match
    //             best_matches.push(m);
    //         }
    //         // If queue full and score is better than min score
    //         else if (m.score > best_matches.top().score)
    //         {
    //             // Delete the smallest score match
    //             best_matches.pop();
    //             // Add the new better match
    //             best_matches.push(m);
    //         }
    //         // Otherwise reject the match since it isn't in the top n scores
    //     }
    // }

    return pq_to_vector(best_matches);
}

// Run template match and store results if this match is better than bestMatch
TemplateMatch getMatch(const cv::Mat &img, const cv:: Mat& templ)
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
        maxVal,
        rect,
        result,
        templ
    };

    return match;
}
