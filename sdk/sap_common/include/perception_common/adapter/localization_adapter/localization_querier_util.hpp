/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng<wangxiaofeng@sensetime.com>
 */

#pragma once

#ifndef BUILD_ADAS

#include <vector>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "perception_common/log/error.hpp"
#include "perception_common/base/data_type/base.hpp"
#include "perception_common/base/data_type/localization_info.hpp"

namespace senseAD {
namespace perception {

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

class LocalizationQuerierUtil {
 public:
    static apStatus_t GetCarPoseFromWindow(const LocalizationInfo &info,
                                           uint64_t timestamp,
                                           CarPose *car_pose);

    static apStatus_t GetVelocityFromWindow(const LocalizationInfo &info,
                                            uint64_t timestamp,
                                            Point3D_t *velocity);

    static apStatus_t GetAngularSpeedFromWindow(
        const LocalizationInfo &info,
        uint64_t timestamp,
        Point3D_t *angular_velocity);
    static apStatus_t GetAccelerationFromWindow(
        const LocalizationInfo &info,
        uint64_t timestamp,
        Point3D_t *acceleration);

    static apStatus_t GetPredictedCarPoseFromWindow(
        const LocalizationInfo &info,
        uint64_t timestamp,
        OdometryInfo *odom_info,
        CarPose *car_pose);

    static apStatus_t GetCarPoseFromWindow(const DequeLocalizationInfo &info,
                                           uint64_t timestamp,
                                           CarPose *car_pose);
    static apStatus_t GetCarPoseFromNearest(const DequeLocalizationInfo &info,
                                            uint64_t timestamp,
                                            CarPose *car_pose);

    static apStatus_t GetVelocityFromWindow(const DequeLocalizationInfo &info,
                                            uint64_t timestamp,
                                            Point3D_t *velocity);

    static apStatus_t GetAngularSpeedFromWindow(
        const DequeLocalizationInfo &info,
        uint64_t timestamp,
        Point3D_t *angular_velocity);
    static apStatus_t GetAccelerationFromWindow(
        const DequeLocalizationInfo &info,
        uint64_t timestamp,
        Point3D_t *acceleration);

    static apStatus_t GetPredictedCarPoseFromWindow(
        const DequeLocalizationInfo &info,
        uint64_t timestamp,
        OdometryInfo *odom_info,
        CarPose *car_pose);

    static apStatus_t CarPose2Tgl(const CarPose &car_pose, Matrix4d *Tgl);

    static apStatus_t SE3Pose2Tgl(const SE3Pose &se3_pose, Matrix4d *Tgl);

    static apStatus_t Tgl2CarPose(const Matrix4d &Tgl, CarPose *car_pose);

    static apStatus_t Tgl2SE3Pose(const Matrix4d &Tgl, SE3Pose *se3_pose);

    static apStatus_t SE3Pose2CarPose(const SE3Pose &se3_pose,
                                      CarPose *car_pose);

    static apStatus_t CarPose2SE2(const CarPose &car_pose,
                                  Matrix3d *se2_car_pose);

    static Matrix3d GetTransform2D(const CarPose &car_pose1,
                                   const CarPose &car_pose2);

    static apStatus_t GetTransform2D(const LocalizationInfo &info,
                                     uint64_t from_timestamp,
                                     uint64_t to_timestamp,
                                     Matrix3d *T);

    static apStatus_t GetTransform2D(const DequeLocalizationInfo &info,
                                     uint64_t from_timestamp,
                                     uint64_t to_timestamp,
                                     Matrix3d *T);

    static Matrix4d GetTransform(const CarPose &car_pose1,
                                 const CarPose &car_pose2);

    static apStatus_t GetTransform(const LocalizationInfo &info,
                                   uint64_t from_timestamp,
                                   uint64_t to_timestamp,
                                   Matrix4d *T);

    static apStatus_t GetTransform(const DequeLocalizationInfo &info,
                                   uint64_t from_timestamp,
                                   uint64_t to_timestamp,
                                   Matrix4d *T);

    static apStatus_t ExtrapolateCarpose(const SE3Pose &carpose0,
                                         const SE3Pose &carpose1,
                                         uint64_t timestamp,
                                         CarPose *outer_pose);

    static apStatus_t InterpolateCarpose(const SE3Pose &carpose0,
                                         const SE3Pose &carpose1,
                                         uint64_t timestamp,
                                         CarPose *inter_pose);

    static Vector3d R2ypr(const Matrix3d &R);

    static Matrix3d ypr2R(const Vector3d &ypr);
};

}  // namespace perception
}  // namespace senseAD

#endif  // BUILD_ADAS
