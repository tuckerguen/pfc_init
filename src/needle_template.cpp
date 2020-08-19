#include <opencv2/core.hpp>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/cudaimgproc.hpp>
#include "needle_template.h"
#include "pfc_initializer_constants.h"

using namespace std;

// constructor
NeedleTemplate::NeedleTemplate(const string& path, const cv::Rect2i& rect, const cv::Point2d& origin, double rotation, pfc::match_params iparams)
: NeedleImage(path), origin(origin), params(iparams)
{
    //Crop both raw and preprocessed template images
    raw = raw(rect);
    image = image(rect);

    // rotate template to align with ground truth 0 degree rotation
    rotate(image, image, rotation);
    rotate(raw, raw, rotation);

    // Center initial rect at top left of template image
    initialRect = cv::Rect2i(0, 0, image.cols, image.rows);
}

// void NeedleTemplate::GenerateTemplate(double rotx, double roty, double rotz, double scale)
// {
//     int resolution = 10;
//     Eigen::Vector2f needle_arc[resolution + 1];
//     float radius = 0.1;

//     Eigen::Matrix<double, 3, 4> projection;
//     Eigen::Matrix4d needle_transformation;

//     for(int i = 0; i <= resolution; i++){
//         double turn_amt = i * M_PI / resolution;

//         Eigen::Vector4d needle_arc_pt = Eigen::Vector4d(
//             radius * cos(turn_amt),
//             radius * sin(turn_amt),
//             0.0,
//             1.0
//         );

//         //Transform based on needle origin location
//         Eigen::Vector4d needle_arc_pt_tf = needle_transformation * needle_arc_pt;

//         //Transform into u, v
//         Eigen::Vector3d needle_arc_pt_uv = projection * needle_arc_pt_tf;

//         double u = needle_arc_pt_uv.x() / needle_arc_pt_uv.z();
//         double v = needle_arc_pt_uv.y() / needle_arc_pt_uv.z();

//         needle_arc[i].x() = u;
//         needle_arc[i].y() = v; 
//     }
// }