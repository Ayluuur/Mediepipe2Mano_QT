#pragma once
#include "core.h"
#include <QJsonArray>
#include <QJsonObject>

namespace m2m {
// JSON field names match Python SkeletonPose.to_dict().
inline QJsonObject skeletonToJson(const SkeletonPose& pose) {
    auto matrix=[](const Matrix& m) {
        QJsonArray rows;
        for(int r=0;r<m.rows();++r) {
            QJsonArray row;
            for(int c=0;c<m.cols();++c) row.append(m(r,c));
            rows.append(row);
        }
        return rows;
    };
    auto matrices=[&](const auto& values) {
        QJsonArray out;
        for(const auto& value:values) out.append(matrix(value));
        return out;
    };
    QJsonArray parents,names;
    for(int j=0;j<16;++j) {parents.append(pose.parents[j]); names.append(QString::fromStdString(pose.names[j]));}
    return {{"side",QString::fromStdString(pose.side)},{"space",QString::fromStdString(pose.space)},
        {"units","mm"},{"quaternion_order","xyzw"},{"rotation_convention","column_vectors"},
        {"mirrored_local_axes",pose.mirroredLocalAxes},{"fit_error_mm",pose.fitErrorMm},
        {"names",names},{"parents",parents},{"positions",matrix(pose.positions())},
        {"rest_positions",matrix(pose.restPositions)},{"rotations",matrices(pose.rotations())},
        {"quaternions",matrix(pose.quaternions())},{"displacements",matrix(pose.displacements())},
        {"local_positions",matrix(pose.localPositions())},{"local_rotations",matrices(pose.localRotations())},
        {"local_quaternions",matrix(pose.localQuaternions())},{"local_axis_angles",matrix(pose.localAxisAngles())},
        {"transforms",matrices(pose.transforms)},{"local_transforms",matrices(pose.localTransforms())}};
}
}
