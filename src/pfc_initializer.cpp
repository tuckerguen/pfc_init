#ifndef PFC_INIT
#define PFC_INIT

#include "pfc_initializer.h"
#include "needle_image.h"
#include "needle_template.h"
#include "template_match.h"
#include "needle_pose.h"
#include "csv_reader.h"
#include <string>
#include <opencv2/core.hpp>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <opencv2/sfm/triangulation.hpp>

using namespace std;

NeedlePose PfcInitializer::computeNeedlePose()
{
    match_l = templ.matchOverScaleAndRotation(left_image.image);
    match_r = templ.matchOverScaleAndRotation(right_image.image);
    
    cv::Point3d location = DeProjectPoints(&match_l, &match_r);
    Eigen::Vector3f orientation(0.0, 0.0, match_l.getAngleDegrees());
    pose = NeedlePose(location, orientation);
    //TODO: This seems like bad design to store it in the object and then also return it?
    return pose;
}

vector<double> PfcInitializer::scorePoseEstimation()
{
    NeedlePose true_pose = readTruePoseFromCSV();

    cv::Point3d true_loc = true_pose.location;
    cv::Point3d result_loc = pose.location;
    double loc_err = cv::norm(result_loc - true_loc);

    Eigen::Quaternionf true_orientation = true_pose.getQuaternionOrientation();
    Eigen::Quaternionf result_orientation = pose.getQuaternionOrientation();  
    Eigen::Quaternionf qdiff = true_orientation.inverse() * result_orientation;
    double angle_err = 2*atan2(qdiff.vec().norm(), qdiff.w()) * pfc::rad2deg;

    cout << "----------------------------------------------------------------------" << endl;
    cout << "Scoring Results" << endl;
    cout << "----------------------------------------------------------------------" << endl;
    cout << "True Pos: (x,y,z)   = (" << true_loc.x << ", " << true_loc.y << ", " << true_loc.z << ")" << endl;
    cout << "True Rot: (x,y,z,w) = (" << true_orientation.x() << ", " << true_orientation.y() << ", " << true_orientation.z() << ", " << true_orientation.w() << ")" << endl;


    cout << "Pos error (meters)  = " << loc_err << endl;
    cout << "Rot error (degrees) = " << angle_err << endl;

    vector<double> results;
    results.push_back(loc_err);
    results.push_back(angle_err);
    return results;
}

cv::Mat getRotatedOrigin(double angle, double scale, NeedleTemplate* templ)
{
    // "Correct angle" is clockwise, we rotate counter clockwise
    angle = -angle;

    // Original template size
    double cols = scale * templ->raw.cols;
    double rows = scale * templ->raw.rows;

    //Get center of original image
    cv::Point2d center((cols-1)/2.0, (rows-1)/2.0);
    //Get rotation matrix given center, angle, and scale
    cv::Mat rot = getRotationMatrix2D(center, angle, 1.0);
    //Compute what the size of the rotated image will be
    cv::Rect2d bbox = cv::RotatedRect(cv::Point2d(), cv::Size(cols, rows), angle).boundingRect2f();
    
    // Add translation to rotation matrix to shift the center of the image to the correct location
    rot.at<double>(0, 2) += bbox.width / 2.0 - cols / 2.0;
    rot.at<double>(1, 2) += bbox.height / 2.0 - rows / 2.0;

    //Scale the original origin to account for scale of template
    cv::Mat original_origin = (cv::Mat_<double>(3,1) << scale * templ->origin.x, scale * templ->origin.y, 1);
    cv::Mat final_origin = rot * original_origin;
    return final_origin;
}

cv::Point3d PfcInitializer::DeProjectPoints(TemplateMatch *match_l, TemplateMatch *match_r)
{
    cv::Mat p_l(2, 1, CV_64FC1);
    cv::Mat p_r(2, 1, CV_64FC1);

    cv::Mat needle_origin_offset_rot_l = getRotatedOrigin(match_l->angle,  match_l->scale, &templ);

    p_l.at<double>(0) = match_l->rect.x + needle_origin_offset_rot_l.at<double>(0);
    p_l.at<double>(1) = match_l->rect.y + needle_origin_offset_rot_l.at<double>(1);

    match_l->needle_origin = p_l;

    cv::Mat needle_origin_offset_rot_r = getRotatedOrigin(match_r->angle,  match_r->scale, &templ);

    p_r.at<double>(0) = match_r->rect.x + needle_origin_offset_rot_r.at<double>(0);
    p_r.at<double>(1) = match_r->rect.y + needle_origin_offset_rot_r.at<double>(1);

    match_r->needle_origin = p_r;

    cout << "(" << p_l.at<double>(0) << ", " << p_l.at<double>(1) << ") | (" << p_r.at<double>(0) << ", " << p_r.at<double>(1) << ")" << endl;

    cv::Mat P_r = (cv::Mat_<double>(3, 4) << 662.450355616388, 0.0, 320.5, -3.31225177808194, 
                                     0.0, 662.450355616388, 240.5, 0.0,
                                     0.0, 0.0, 1.0, 0.0);

    cv::Mat P_l = (cv::Mat_<double>(3, 4) << 662.450355616388, 0.0, 320.5, 0.0, 
                                     0.0, 662.450355616388, 240.5, 0.0, 
                                     0.0, 0.0, 1.0, 0.0);


    cv::Mat results;

    vector<cv::Mat> points;
    points.push_back(p_l);
    points.push_back(p_r);

    vector<cv::Mat> projections;
    projections.push_back(P_l);
    projections.push_back(P_r);

    cv::sfm::triangulatePoints(points, projections, results);

    cv::Point3d result;

    result.x = results.at<double>(0);
    result.y = results.at<double>(1);
    result.z = results.at<double>(2);

    return result;
}

void PfcInitializer::drawNeedleOrigin(cv::Mat& img, TemplateMatch* match, cv::Scalar color){
    cv::Mat needle_origin = getRotatedOrigin(match->angle, match->scale, &templ);

    cv::circle(img, 
            cv::Point(match->needle_origin.at<double>(0), 
                  match->needle_origin.at<double>(1)),
            0.1, color, 1, 8, 0);
}


void PfcInitializer::displayResults()
{
    match_l.drawOnImage(left_image.raw, cv::Scalar::all(255));
    match_r.drawOnImage(right_image.raw, cv::Scalar::all(255));
    drawNeedleOrigin(left_image.raw, &match_l, cv::Scalar::all(255)); 
    drawNeedleOrigin(right_image.raw, &match_r, cv::Scalar::all(255)); 

    match_l.printMatchSummary("Left");
    match_r.printMatchSummary("Right");
    cout << "----------------------------------------------------------------------" << endl;
    cout << "Experimental Results" << endl;
    cout << "----------------------------------------------------------------------" << endl;
    pose.print();

    cv::namedWindow("left", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("right", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("left template", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("right template", cv::WINDOW_AUTOSIZE);
    cv::imshow("left", left_image.raw);
    cv::imshow("right", right_image.raw);
    cv::imshow("left template", match_l.templ);
    cv::imshow("right template", match_r.templ);
    cv::imwrite("raw_OUT.png", left_image.raw);
    cv::imwrite("template_OUT.png", match_l.templ);
    cv::waitKey(0);
}

NeedlePose PfcInitializer::readTruePoseFromCSV()
{
	CSVReader reader("../positions/needle_positions.csv");
    vector<vector<string> > all_pose_data = reader.getData();
    vector<string> pose_data = all_pose_data.at(pose_id);

    NeedlePose pose;
    pose.location.x = stod(pose_data.at(1));
    pose.location.y = stod(pose_data.at(2));
    pose.location.z = stod(pose_data.at(3));

    Eigen::Quaternionf q;
    q.x() = stod(pose_data.at(4));
    q.y() = stod(pose_data.at(5));
    q.z() = stod(pose_data.at(6));
    q.w() = stod(pose_data.at(7));

    pose.setOrientation(q);

    return pose;
}

vector<string> PfcInitializer::getResultsVector()
{
    vector<string> results;
    // Add pixel location guesses
    results.push_back(to_string(match_l.rect.x));
    results.push_back(to_string(match_l.rect.y));
    results.push_back(to_string(match_r.rect.x));
    results.push_back(to_string(match_r.rect.y));
    //Add scalee guess
    results.push_back(to_string(match_l.scale));
    results.push_back(to_string(match_r.scale));
    //Add location guess
    results.push_back(to_string(pose.location.x));
    results.push_back(to_string(pose.location.y));
    results.push_back(to_string(pose.location.z));
    //Add orientation guess
    //Quaternion
    Eigen::Quaternionf q = pose.getQuaternionOrientation();
    results.push_back(to_string(q.x()));
    results.push_back(to_string(q.y()));
    results.push_back(to_string(q.z()));
    results.push_back(to_string(q.w()));
    //Euler Angles (mostly for human intuition)
    Eigen::Vector3f e = pose.getEulerAngleOrientation();
    results.push_back(to_string(e.x()));
    results.push_back(to_string(e.y()));
    results.push_back(to_string(e.z()));
    //Add scores
    vector<double> scores = scorePoseEstimation();
    results.push_back(to_string(scores.at(0)));
    results.push_back(to_string(scores.at(1)));

    return results;
}

#endif