#include "PfcInit.hpp"
#include "NeedleImage.hpp"
#include "NeedleTemplate.hpp"
#include "TemplateMatch.hpp"
#include "NeedlePose.hpp"
#include "CSVReader.hpp"
#include <string>
#include <opencv2/core.hpp>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <opencv2/sfm/triangulation.hpp>

using namespace std;

// Rotation parameters
const double min_rotation = 0;
const double max_rotation = 360;   //Max number of degrees to rotate template
const double rotation_increment = 1; //Number of degrees to rotate template each iteration

// Scaling parameters
const int min_scale = 95;         //minimum template scale to try to match (in %)
const int max_scale = 110;        //maximum template scale to try to match (in %)
const double scale_increment = 1; //% scale to increase by on each iteration

NeedlePose PfcInit::computeNeedlePose()
{
    match_l = templ.matchOverScaleAndRotation(left_image.image, min_scale, max_scale, scale_increment, min_rotation, max_rotation, rotation_increment);
    match_r = templ.matchOverScaleAndRotation(right_image.image, min_scale, max_scale, scale_increment, min_rotation, max_rotation, rotation_increment);
    
    cv::Point3d location = DeProjectPoints(&match_l, &match_r);
    Eigen::Vector3f orientation(0.0, 0.0, match_l.getAngleDegrees());
    pose = NeedlePose(location, orientation);
    //TODO: This seems like bad design to store it in the object and then also return it?
    return pose;
}

void PfcInit::scorePoseEstimation()
{
    NeedlePose true_pose = ReadTruePoseFromCSV();

    cv::Point3d true_loc = true_pose.location;
    cv::Point3d result_loc = pose.location;
    double dist = cv::norm(result_loc - true_loc);

    Eigen::Quaternionf true_orientation = true_pose.getQuaternionOrientation();
    Eigen::Quaternionf result_orientation = pose.getQuaternionOrientation();  
    Eigen::Quaternionf qdiff = true_orientation.inverse() * result_orientation;
    double angle_diff = 2*atan2(qdiff.vec().norm(), qdiff.w());


    cout << "----------------------------------------------------------------------" << endl;
    cout << "Scoring Results" << endl;
    cout << "----------------------------------------------------------------------" << endl;
    cout << "True Pos: (x,y,z)   = (" << true_loc.x << ", " << true_loc.y << ", " << true_loc.z << ")" << endl;
    cout << "True Rot: (x,y,z,w) = (" << true_orientation.x() << ", " << true_orientation.y() << ", " << true_orientation.z() << ", " << true_orientation.w() << ")" << endl;


    cout << "Pos error (meters)  = " << dist << endl;
    cout << "Rot error (degrees) = " << angle_diff*rad2deg << endl;

}

cv::Point3d PfcInit::DeProjectPoints(const TemplateMatch *match_l, const TemplateMatch *match_r)
{
    cv::Mat p_l(2, 1, CV_64FC1);
    cv::Mat p_r(2, 1, CV_64FC1);

    //Apply the same rotation matrix used to rotate the template to the base template origin offset
    cv::Mat needle_origin_offset_mat = (cv::Mat_<double>(3,1) << templ.origin_offset_x, templ.origin_offset_y, 1);
    cv::Point2d center((templ.initialRect.width - 1) / 2.0, (templ.initialRect.height - 1) / 2.0);
    cv::Mat rot = getRotationMatrix2D(center, match_l->angle, match_l->scale);
    cv::Rect2d bbox = cv::RotatedRect(cv::Point2d(), cv::Size2d(templ.initialRect.width, templ.initialRect.height), match_l->angle).boundingRect2f();
    
    /// adjust transformation matrix
    rot.at<double>(0, 2) += bbox.width / 2.0 - templ.initialRect.width / 2.0;
    rot.at<double>(1, 2) += bbox.height / 2.0 - templ.initialRect.height / 2.0;
    cv::Mat needle_origin_offset_rot = rot * needle_origin_offset_mat;

    p_l.at<double>(0) = match_l->rect.x + needle_origin_offset_rot.at<double>(0);
    p_l.at<double>(1) = match_l->rect.y + needle_origin_offset_rot.at<double>(1);

    p_r.at<double>(0) = match_r->rect.x + needle_origin_offset_rot.at<double>(0);
    p_r.at<double>(1) = match_r->rect.y + needle_origin_offset_rot.at<double>(1);

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

void PfcInit::drawNeedleOrigin(cv::Mat& img, TemplateMatch* match, cv::Scalar color){
    //Apply the same rotation matrix used to rotate the template to the base template origin offset
    cv::Mat needle_origin_offset_mat = (cv::Mat_<double>(3,1) << templ.origin_offset_x, templ.origin_offset_y, 1);
    cv::Point2d center((templ.initialRect.width - 1) / 2.0, (templ.initialRect.height - 1) / 2.0);
    
    double rot_angle = -match->angle;
    if(match->angle < 180)
        rot_angle = 360 - match->angle;
    
    cv::Mat rot = getRotationMatrix2D(center, rot_angle, match->scale);
    cv::Rect2d bbox = cv::RotatedRect(cv::Point2d(), cv::Size2d(templ.initialRect.width, templ.initialRect.height), match->angle).boundingRect2f();
    /// adjust transformation matrix
    rot.at<double>(0, 2) += bbox.width / 2.0 - templ.initialRect.width / 2.0;
    rot.at<double>(1, 2) += bbox.height / 2.0 - templ.initialRect.height / 2.0;
    cv::Mat needle_origin_offset_rot = rot * needle_origin_offset_mat;

    cv::circle(img, 
            cv::Point(match->rect.x + needle_origin_offset_rot.at<double>(0), 
                  match->rect.y + needle_origin_offset_rot.at<double>(1)),
            0.1, color, 1, 8, 0);
}


void PfcInit::displayResults()
{
    match_l.drawOnImage(left_image.raw, cv::Scalar::all(255));
    match_r.drawOnImage(right_image.raw, cv::Scalar::all(255));
    drawNeedleOrigin(left_image.raw, &match_l, cv::Scalar::all(255)); 
    drawNeedleOrigin(right_image.raw, &match_r, cv::Scalar::all(255)); 
    cout << "----------------------------------------------------------------------" << endl;
    cout << "Experimental Results" << endl;
    cout << "----------------------------------------------------------------------" << endl;
    pose.print();

    cv::namedWindow("left", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("right", cv::WINDOW_AUTOSIZE);
    cv::imshow("left", left_image.raw);
    cv::imshow("right", right_image.raw);
    cv::waitKey(0);
}

NeedlePose PfcInit::ReadTruePoseFromCSV()
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