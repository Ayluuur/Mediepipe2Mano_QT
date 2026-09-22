#include "core.h"
#include <stdexcept>

namespace m2m {
Matrix SkeletonPose::positions() const {
    Matrix out(16,3);
    for(int j=0;j<16;++j) out.row(j)=transforms[j].topRightCorner<3,1>().transpose();
    return out;
}
Matrix SkeletonPose::displacements() const { return positions()-restPositions; }
std::array<Mat3,16> SkeletonPose::rotations() const {
    std::array<Mat3,16> out;
    for(int j=0;j<16;++j) out[j]=transforms[j].topLeftCorner<3,3>();
    return out;
}
Matrix SkeletonPose::quaternions() const {
    Matrix out(16,4);
    for(int j=0;j<16;++j) out.row(j)=Eigen::Quaterniond(Mat3(transforms[j].topLeftCorner<3,3>())).normalized().coeffs().transpose();
    return out;
}
std::array<Eigen::Matrix4d,16> SkeletonPose::localTransforms() const {
    auto out=transforms;
    for(int j=1;j<16;++j) out[j]=transforms[parents[j]].inverse()*transforms[j];
    return out;
}
Matrix SkeletonPose::localPositions() const {
    auto local=localTransforms(); Matrix out(16,3);
    for(int j=0;j<16;++j) out.row(j)=local[j].topRightCorner<3,1>().transpose();
    return out;
}
std::array<Mat3,16> SkeletonPose::localRotations() const {
    auto local=localTransforms(); std::array<Mat3,16> out;
    for(int j=0;j<16;++j) out[j]=local[j].topLeftCorner<3,3>();
    return out;
}
Matrix SkeletonPose::localQuaternions() const {
    auto local=localRotations(); Matrix out(16,4);
    for(int j=0;j<16;++j) out.row(j)=Eigen::Quaterniond(local[j]).normalized().coeffs().transpose();
    return out;
}
Matrix SkeletonPose::localAxisAngles() const {
    auto local=localRotations(); Matrix out(16,3);
    for(int j=0;j<16;++j) {
        Eigen::AngleAxisd aa(local[j]); out.row(j)=(aa.axis()*aa.angle()).transpose();
    }
    return out;
}
SkeletonPose SkeletonPose::toScene(const Vec3& translation) const {
    if(space!="camera" || !translation.allFinite())
        throw std::invalid_argument("Scene conversion requires camera space and finite translation");
    SkeletonPose out=*this; out.space="scene";
    Mat3 basis=Vec3(1,-1,-1).asDiagonal();
    Eigen::Matrix4d transform=Eigen::Matrix4d::Identity();
    transform.topLeftCorner<3,3>()=basis; transform.topRightCorner<3,1>()=translation;
    for(auto& t:out.transforms) t=(transform*t).eval();
    out.restPositions=restPositions*basis.transpose();
    return out;
}
}
