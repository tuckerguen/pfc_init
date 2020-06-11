#include <opencv2/core.hpp>
#include <iostream>
#include <opencv2/sfm.hpp>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include "CSVReader.hpp"

using namespace cv;
using namespace std;
using namespace cv::sfm;
using namespace Eigen;

struct Pose {
    cv::Point3d location;
    Eigen::Vector4f orientation;
};
Pose ReadTruePoseFromCSV(int pose_id);

int main(){
 //construct the output mat:
    Mat p_l(2, 1, CV_64FC1);
    Mat p_r(2, 1, CV_64FC1);

    //template dimensions: width=105, height=56
    //356, 211, -220 degrees
    //-220, 140
    // 220, -140
    //64, 86
    //85

    Mat needle_origin_offset0 = (Mat_<double>(3,1) << 52, 4, 1);
    cout << needle_origin_offset0 << endl;
    Point2d center((105 - 1) / 2.0, (56 - 1) / 2.0);
    Mat rot = getRotationMatrix2D(center, 220, 1.1);
    // cout << rot << endl;
    Rect2d bbox = RotatedRect(Point2d(), cv::Size2f(105,56), 220).boundingRect2f();
    /// adjust transformation matrix
    rot.at<double>(0, 2) += bbox.width / 2.0 - 105 / 2.0;
    rot.at<double>(1, 2) += bbox.height / 2.0 - 56 / 2.0;
    Mat needle_origin_offset = rot * needle_origin_offset0;
    cout << "needle origin offset: " << needle_origin_offset << endl;

    p_l.at<double>(0) = 292 + 87;
    p_l.at<double>(1) = 125 + 84;

    p_r.at<double>(0) = 271+ 87;
    p_r.at<double>(1) = 125+ 84;
    // p_l.at<double>(0) = 292 + needle_origin_offset.at<double>(0);
    // p_l.at<double>(1) = 125 + needle_origin_offset.at<double>(1);

    // p_r.at<double>(0) = 271 + needle_origin_offset.at<double>(0);
    // p_r.at<double>(1) = 125 + needle_origin_offset.at<double>(1);

    Mat P_r = (Mat_<double>(3, 4) << 662.450355616388, 0.0, 320.5, -3.31225177808194, 
                                     0.0, 662.450355616388, 240.5, 0.0,
                                     0.0, 0.0, 1.0, 0.0);

    Mat P_l = (Mat_<double>(3, 4) << 662.450355616388, 0.0, 320.5, 0.0, 
                                     0.0, 662.450355616388, 240.5, 0.0, 
                                     0.0, 0.0, 1.0, 0.0);


    Mat results;

    vector<Mat> points;
    points.push_back(p_l);
    points.push_back(p_r);

    vector<Mat> projections;
    projections.push_back(P_l);
    projections.push_back(P_r);

    sfm::triangulatePoints(points, projections, results);

    Point3d result;

    result.x = results.at<double>(0);
    result.y = results.at<double>(1);
    result.z = results.at<double>(2);

    cout << "Final Position: " << result.x << ", " << result.y << ", " << result.z << endl;

    Pose p = ReadTruePoseFromCSV(3);
    Point3d truth = p.location;
    cout << "True Position: " << truth.x << ", " << truth.y << ", " << truth.z << endl;
    double dist = norm(result - truth);
    cout << "Euclidean Distance from Truth: " << dist << endl;
    cout << "Diff X: " << truth.x - result.x << " meters" << endl;
    cout << "Diff Y: " << truth.y - result.y << " meters" << endl;
    cout << "Diff Z: " << truth.z - result.z << " meters" << endl;

    Quaternionf q;
    q = AngleAxisf(0, Vector3f::UnitX())
        * AngleAxisf(0, Vector3f::UnitY())
        * AngleAxisf(-3.839724354, Vector3f::UnitZ());
    cout << "(" << q.coeffs().x() << ", " << q.coeffs().y() << ", " << q.coeffs().z() << ", " << q.coeffs().w() << endl;;
}

// Pose ReadTruePoseFromCSV(string pose_id){
//     Pose pose;

//     fstream fin;
//     fin.open("../positions/needle_positions.csv", ios::in);
//     vector<string> pose_data;
//     string line, val, temp;
//     while (fin >> temp)
//     {
//         getline(fin, line);
//         stringstream s(line);
//         getline(s, val, ',');
//         cout << "val: " << val << endl;
//         if(val == pose_id)
//         {
//             pose_data.clear();
//             while(getline(s, val, ','))
//             {
//                 pose_data.push_back(val);
//             }
//             pose.location.x = stod(pose_data.at(0));
//             pose.location.y = stod(pose_data.at(1));
//             pose.location.z = stod(pose_data.at(2));
//             pose.orientation(0) = stod(pose_data.at(3));
//             pose.orientation(1) = stod(pose_data.at(4));
//             pose.orientation(2) = stod(pose_data.at(5));
//             pose.orientation(3) = stod(pose_data.at(6));
//         }
//     }
//     return pose;
// }

Pose ReadTruePoseFromCSV(int pose_id)
{
	CSVReader reader("../positions/needle_positions.csv");
    vector<vector<string> > all_pose_data = reader.getData();
    vector<string> pose_data = all_pose_data.at(pose_id);
 
    Pose pose;
    pose.location.x = stod(pose_data.at(1));
    pose.location.y = stod(pose_data.at(2));
    pose.location.z = stod(pose_data.at(3));
    pose.orientation(0) = stod(pose_data.at(4));
    pose.orientation(1) = stod(pose_data.at(5));
    pose.orientation(2) = stod(pose_data.at(6));
    pose.orientation(3) = stod(pose_data.at(7));
    return pose;
}