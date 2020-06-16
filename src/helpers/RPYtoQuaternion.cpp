#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace Eigen;

Vector4f RPYtoQuat(double roll, double pitch, double yaw);

int main(){
    Vector4f q = RPYtoQuat(0, 0, 3.2);
    std::cout << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() <<  std::endl;

}

Vector4f RPYtoQuat(double roll, double pitch, double yaw){
    Quaternionf q;
    q = AngleAxisf(roll, Vector3f::UnitX())
        * AngleAxisf(pitch, Vector3f::UnitY())
        * AngleAxisf(yaw, Vector3f::UnitZ());
    return q.coeffs();
}