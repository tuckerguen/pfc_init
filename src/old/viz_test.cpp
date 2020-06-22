#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace std;

using namespace cv;

void rotateTempl(double degrees, const cv::Mat &src, cv::Mat &dst);

int main(){
    cv::Mat p_l(2, 1, CV_64FC1);
    cv::Mat p_r(2, 1, CV_64FC1);

    //Apply the same rotation matrix used to rotate the template to the base template origin offset
    cv::Mat needle_origin_offset_mat = (cv::Mat_<double>(3,1) << 52, 11,1);

    cv::Mat img = cv::imread("../../imgs/raw/0_l_c_fatty.png", cv::IMREAD_COLOR);

    cv::Rect2i initial_rect(287,205,105,56);
    img = img(initial_rect);
    cv::Mat rotated;
    rotateTempl(-176.6537, img, rotated);
    cv::Mat resized, rot_templ;
    //Use inter-linear in all cases (is faster than inter_area, similar results)
    double scale = 1.4;
    cv::resize(rotated, resized, cv::Size(), scale, scale, cv::INTER_LINEAR);

    cv::Mat templ;
    double angle = 162;

    rotateTempl(angle, resized, templ);
    namedWindow("h");
    imshow("h", templ);

    //rotated is the original template that will be passed into the function
    // It's the image that will be rotated, within which we are finding the origin again
    // we use these rows and cols as the center of the original imgae
    double cols = scale*rotated.cols;
    double rows = scale*rotated.rows;
    cout << cols << ", " << rows << endl;
    //Get center of original image
    cv::Point2d center((cols-1)/2.0, (rows-1)/2.0);
    //Get rotation matrix given center, angle, and scale
    cv::Mat rot = getRotationMatrix2D(center, angle, 1.0);
    //Compute what the size of the rotated image will be
    cv::Rect2d bbox = cv::RotatedRect(cv::Point2d(), cv::Size(cols, rows), angle).boundingRect2f();
    
    // Add translation to rotation matrix to shift the center of the image to the correct location
    rot.at<double>(0, 2) += bbox.width / 2.0 - cols / 2.0;
    rot.at<double>(1, 2) += bbox.height / 2.0 - rows / 2.0;
    // rot.at<double>(0,0) *= scale;
    // rot.at<double>(1,1) *= scale;

    cout << rot << endl;

    Mat P = (Mat_<double>(3,1) << scale*54,scale*9,1);
    cv::Mat final = rot * P;
    cout << final << endl;
    waitKey(0);

    // cv::Mat loc = (Mat_<double> )


    // for(int i = 0; i < rotated.size().width; i++){
    //     for(int j = 0; j < rotated.size().height; j++){
    //         Vec3b white;
    //         white[0] = 0; white[1] = 0; white[2] = 0;
    //         Vec3b black;
    //         white[0] = 255; white[1] = 255; white[2] = 255;
    //         if(j == 11 && i == 54)
    //             rotated.at<Vec3b>(j,i) = white;
    //         else
    //         {
    //             rotated.at<Vec3b>(j,i) = black;
    //         }
    //     }
    // }



    // std::vector<cv::Point2f> yourPoints;
    // yourPoints.push_back(cv::Point2f(52,4));
    // std::vector<cv::Point2f> transformedPoints;

    // cv::Point2d center((105 - 1) / 2.0, (56 - 1) / 2.0);
    // cv::Mat rot = getRotationMatrix2D(center, 162, 1.4);
    // cout << rot << endl;
    // cv::Rect2d bbox = cv::RotatedRect(cv::Point2d(), cv::Size2d(105, 56), 162).boundingRect2f();
    
    // /// adjust transformation matrix
    // rot.at<double>(0, 2) += bbox.width / 2.0 - 105 / 2.0;
    // rot.at<double>(1, 2) += bbox.height / 2.0 - 56 / 2.0;
    // cv::Mat needle_origin_offset_rot;
    // cv::warpAffine(rotated, needle_origin_offset_mat, rot, bbox.size());

    // cv::Mat compare;
    // rotateTempl(162, templ, compare);

    cv::namedWindow("h1", CV_WINDOW_AUTOSIZE);
    cv::imshow("h1", rotated);
    // cv::namedWindow("h", CV_WINDOW_AUTOSIZE);
    // cv::imshow("h", compare);
    // cv::waitKey(0);
    

    // needle_origin_offset_rot = rot *  needle_origin_offset_mat;
    // cv::perspectiveTransform(yourPoints, transformedPoints, rot);
    // cout << transformedPoints.at(0).x << ", " << transformedPoints.at(0).y << endl;

    // cout << needle_origin_offset_rot.at<double>(0) << ", " << needle_origin_offset_rot.at<double>(1) << endl;

    // p_l.at<double>(0) = 107 + needle_origin_offset_rot.at<double>(0);
    // p_l.at<double>(1) = 138 + needle_origin_offset_rot.at<double>(1);

    // p_r.at<double>(0) = 81 + needle_origin_offset_rot.at<double>(0);
    // p_r.at<double>(1) = 138 + needle_origin_offset_rot.at<double>(1);


    return 0;
}

void rotateTempl(double degrees, const cv::Mat &src, cv::Mat &dst)
{
    /// get rotation matrix for rotating the image around its center in pixel coordinates
    cv::Point2d center((src.cols - 1) / 2.0, (src.rows - 1) / 2.0);

    cv::Mat rot = getRotationMatrix2D(center, degrees, 1.0);
    /// determine bounding rectangle, center not relevant
    cv::Rect2d bbox = cv::RotatedRect(cv::Point2d(), src.size(), degrees).boundingRect2f();
    /// adjust transformation matrix
    rot.at<double>(0, 2) += bbox.width / 2.0 - src.cols / 2.0;
    rot.at<double>(1, 2) += bbox.height / 2.0 - src.rows / 2.0;

    // cv::Mat needle_origin_offset_mat = (cv::Mat_<double>(3,1) << 52, 4, 1);

    warpAffine(src, dst, rot, bbox.size());
}

// One attempt at using WarpAffine
    // cv::Mat output(2,1, CV_32F, Scalar::all(0));
    // cv::Mat input(3,1, CV_32F, Scalar::all(0));
    // double M11 = rot.at<double>(0,0);
    // double M12 = rot.at<double>(0,1);
    // double M13 = rot.at<double>(0,2);
    // double M21 = rot.at<double>(1,0);
    // double M22 = rot.at<double>(1,1);
    // double M23 = rot.at<double>(1,2);
    // cout << M11 << ", " << M12 << ", " << M13 << ", " << M21 << ", " << M22 << ", " << M23 << endl;
    // cout << rot << endl;
    // double y1 = (M21 * 52 + M21*M13+M11*M23-M11 * 4) / (M21*M12-M11*M22);
    // cout << y1 << endl;

    // double x1 = (52-M12*y1-M13)/M11;
    // cout << x1 << endl;

    // input.at<double>(0) = (x1);
    // input.at<double>(1) = (y1);
    // input.at<double>(2) = (1.0);

    // cout << output.size() << endl;
    // warpAffine(input, output, rot, output.size());
    // cout << output.at<double>(0) << ", " << output.at<double>(1) << endl;



//// Some viz stuff
    // cout << "(" << p_l.at<double>(0) << ", " << p_l.at<double>(1) << ") | (" << p_r.at<double>(0) << ", " << p_r.at<double>(1) << ")" << endl;
    // viz::Viz3d myWindow("Viz test");
    // myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());
    
    // viz::WLine axis(Point3f(-1.0f, -1.0f, -1.0f), Point3f(1.0f, 1.0f, 1.0f));
    // axis.setRenderingProperty(viz::LINE_WIDTH, 4.0);
    // myWindow.showWidget("line widget", axis);

    // viz::WCube cube_widget(Point3f(0.5,0.5,0.0), Point3f(0.0,0.0,-0.5), true, viz::Color::blue());
    // cube_widget.setRenderingProperty(viz::LINE_WIDTH, 4.0);
    // myWindow.showWidget("Cube Widget", cube_widget);

    // Mat rot_vec = Mat::zeros(1,3,CV_32F);
    // float translation_phase = 0.0, translation = 0.0;

    // while(!myWindow.wasStopped())
    // {
    //     /* Rotation using rodrigues */
    //     rot_vec.at<float>(0,0) += (float)CV_PI * 0.01f;
    //     rot_vec.at<float>(0,1) += (float)CV_PI * 0.01f;
    //     rot_vec.at<float>(0,2) += (float)CV_PI * 0.01f;
    //     translation_phase += (float)CV_PI * 0.01f;
    //     translation = sin(translation_phase);
    //     Mat rot_mat;
    //     Rodrigues(rot_vec, rot_mat);
    //     Affine3f pose(rot_mat, Vec3f(translation, translation, translation));
    //     myWindow.setWidgetPose("Cube Widget", pose);
    //     myWindow.spinOnce(1, true);
    // }
 
