#include <string>
#include <iostream>
#include <queue>
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include "pose_helper.h"
#include "matcher.h"
#include "pfc_initializer.h"
#include "needle_template.h"
#include "needle_pose.h"

using namespace std;

// Calculates needle pose from images (primary function for use in particle filter)
// Stores pose in this object
void PfcInitializer::run(bool print_results, bool multi_thread, int pose_id)
{
    // Start timer
    double t = (double)cv::getTickCount();

    computeNeedlePose(multi_thread);
    
    //Stop timer
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

    if(print_results)
    {
        cout << "Time: " << t << " s" << endl;
        displayResults(pose_id);
    }
}

// Computes pose of needle from the left and right stereo images
void PfcInitializer::computeNeedlePose(bool multi_thread)
{
    // Perform template match on left and right images
    if(multi_thread)
    {
        l_matches = matchThreaded(left_image.image, templ);
        r_matches = matchThreaded(right_image.image, templ);
    }
    else
    {
        l_matches = match(left_image.image, templ);
        r_matches = match(right_image.image, templ);
    }

    if(l_matches.size() != r_matches.size())
    {
        std::cerr << "Left and right matches aren't the same length" << endl;
        exit(0);
    }

    // Compute needle positions for all matches
    for(int i = 0; i < l_matches.size(); i++)
    {
        TemplateMatch *match_l = &l_matches.at(i);
        TemplateMatch *match_r = &r_matches.at(i);

        //Initialize left and right needle pixel location vectors
        cv::Mat p_l(2, 1, CV_64FC1);
        cv::Mat p_r(2, 1, CV_64FC1);
        
        // Compute pixel space origin in left image
        cv::Mat rotated_origin_l = getRotatedOrigin(match_l->angle,  match_l->scale, &templ);
        p_l.at<double>(0) = match_l->rect.x + rotated_origin_l.at<double>(0);
        p_l.at<double>(1) = match_l->rect.y + rotated_origin_l.at<double>(1);
        match_l->needle_origin = p_l;

        // Compute pixel space origin in right image
        cv::Mat rotated_origin_r = getRotatedOrigin(match_r->angle, match_r->scale, &templ);
        p_r.at<double>(0) = match_r->rect.x + rotated_origin_r.at<double>(0);
        p_r.at<double>(1) = match_r->rect.y + rotated_origin_r.at<double>(1);
        match_r->needle_origin = p_r;
        
        // Get 3D location of needle
        cv::Point3d location = deProjectPoints(p_l, p_r);
        // Get Euler angle orientation
        double average_yaw = (match_l->getAngleDegrees() + match_r->getAngleDegrees()) / 2.0;
        Eigen::Vector3f orientation(0.0, 0.0, average_yaw);

        // Store location/orientation
        poses.push_back(NeedlePose(location, orientation));
    }
}

void PfcInitializer::displayResults(int pose_id)
{
    // Draw matches onto images
    for(int i = 0; i < l_matches.size(); i++)
    {
        TemplateMatch match_l = l_matches.at(i);
        TemplateMatch match_r = r_matches.at(i);
        match_l.drawOnImage(left_image.raw, cv::Scalar::all(255));
        match_r.drawOnImage(right_image.raw, cv::Scalar::all(255));
        drawNeedleOrigin(left_image.raw, &match_l, cv::Scalar(0,255,255)); 
        drawNeedleOrigin(right_image.raw, &match_r, cv::Scalar(0,255,255)); 
    }

    for(int i = 0; i < poses.size(); i++)
    {
        cout << "Candidate Point " + to_string(i) << endl;
        poses.at(i).print();
        scorePoseEstimation(poses.at(i), pose_id, true);      
        cout << "----------------------------------------------------------------------" << endl;
    }

    cv::namedWindow("left", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("right", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("left template", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("right template", cv::WINDOW_AUTOSIZE);
    cv::imshow("left", left_image.raw);
    cv::imshow("right", right_image.raw);
    // cv::imshow("left template", match_l.templ);
    // cv::imshow("right template", match_r.templ);
    cv::waitKey(0);

    cv::destroyAllWindows();
}

vector<vector<string>> PfcInitializer::getResultsAsVector(int pose_id)
{
    vector<vector<string>> results;

    // Add pixel location guesses
    for(int i = 0; i < l_matches.size(); i++)
    {
        TemplateMatch match_l = l_matches.at(i);
        TemplateMatch match_r = r_matches.at(i);
        NeedlePose pose = poses.at(i);

        //Add location guess
        results.at(i).push_back(to_string(pose.location.x));
        results.at(i).push_back(to_string(pose.location.y));
        results.at(i).push_back(to_string(pose.location.z));

        //Add orientation guess
        //Quaternion
        Eigen::Quaternionf q = pose.getQuaternionOrientation();
        results.at(i).push_back(to_string(q.x()));
        results.at(i).push_back(to_string(q.y()));
        results.at(i).push_back(to_string(q.z()));
        results.at(i).push_back(to_string(q.w()));

        //Euler Angles orientation guess
        // Eigen::Vector3f e = pose.getEulerAngleOrientation();
        // results.at(i).push_back(to_string(e.x()));
        // results.at(i).push_back(to_string(e.y()));
        // results.at(i).push_back(to_string(e.z()));
        
        vector<double> score = scorePoseEstimation(pose, pose_id, false);
        results.at(i).push_back(to_string(score.at(0)));
        results.at(i).push_back(to_string(score.at(1))); 
    }

    return results;
}