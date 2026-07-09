/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Ma Quanmeng <maquanmeng@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>
#include <string>

#include "eigen3/Eigen/Dense"
#include "perception_common/base/data_type/object_label.hpp"
#include "perception_common/log/error.hpp"

#include "perception_common/state_estimation/state/state_info.hpp"
#include "perception_common/state_estimation/state/processmodel_info.hpp"
#include "perception_common/math_helper/angle_util.hpp"

namespace senseAD {
namespace perception {
// base object
typedef struct StructObjectState {
    // type
    ObjectLabel label = ObjectLabel::UNKNOWN;
    ProcessModelType model_type = ProcessModelType::UNKNOWN_PROCESSMODEL;
    // sensor name
    std::string sensor_name;
    // confidence
    // TODO(zhangshqiuan): specify what kind of confidence
    float confidence = -1.0f;

    // track
    uint64_t object_id = 0;
    uint64_t timestamp_ns = 0;

    // 3D shape, width [m], height [m], length [m]
    Eigen::Vector3f shape;
    std::vector<Eigen::Vector2f> poly_box;

    // 3D center position info
    Eigen::Vector3f position = Eigen::Vector3f(0, 0, 0);
    Eigen::Matrix3f position_uncertainty = Eigen::Matrix3f::Identity();

    // 3D direction info
    Eigen::Vector3f direction = Eigen::Vector3f(1, 0, 0);
    float yaw = 0;
    // bird view direction
    // vehicle coordinate system, x-axis forward is 0
    // increase anticlockwise
    float theta = 0.0f;  // [rad], range:[-pi,pi]
    float theta_variance = 0.0f;

    // motion info
    float static_score = 0.0f;
    // object motion param: velocity, acceleration. yaw rate
    Eigen::Vector3f velocity = Eigen::Vector3f(0, 0, 0);
    // covariance matrix of the velocity uncertainty, required
    Eigen::Matrix3f velocity_uncertainty = Eigen::Matrix3f::Identity();
    // brief acceleration of the object, required
    Eigen::Vector3f acceleration = Eigen::Vector3f(0, 0, 0);
    // brief covariance matrix of the acceleration uncertainty, required
    Eigen::Matrix3f acceleration_uncertainty = Eigen::Matrix3f::Identity();

    Eigen::Matrix3f position_velocity_uncertainty = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f velocity_acceleration_uncertainty = Eigen::Matrix3f::Zero();

    Eigen::Vector3f jerk = Eigen::Vector3f(0, 0, 0);
    Eigen::Matrix3f jerk_uncertainty = Eigen::Matrix3f::Identity();
    // yaw rate
    float yaw_rate = 0;      // [rad/s]
    float var_yaw_rate = 0;  // [rad/s] standard deviation
    // meas bias between sensors
    float x_bias = 0;
    float y_bias = 0;
    // main sensor information
    bool is_main_sensor = false;
    bool is_optimze_camera_radar_bias = false;
    bool is_optimze_lidar_radar_bias = false;
    bool is_optimze_yaw = false;
    bool is_transformed = false;
    uint64_t target_timestamp = 0;

    // Add temp variables to reduces CPU/Memory consumption
    Eigen::Matrix2f rot_temp = Eigen::Matrix2f::Zero();
    Eigen::Vector2f t_temp = Eigen::Vector2f(0, 0);
    StateVectorf full_state_temp = StateVectorf::Zero(STATE_DIMS);
    Eigen::Vector2f postion_temp = Eigen::Vector2f(0, 0);
    Eigen::Vector2f velocity_temp = Eigen::Vector2f(0, 0);
    Eigen::Vector2f acceleration_temp = Eigen::Vector2f(0, 0);
    Eigen::Vector2f jerk_temp = Eigen::Vector2f(0, 0);
    StateMatrixf state_cov_temp = StateMatrixf::Identity();
    StateMatrixf src_cov_temp = StateMatrixf::Identity();
    StateMatrixf dst_cov_temp = StateMatrixf::Identity();

    StateVectorf full_state2D = StateVectorf::Zero(STATE_DIMS);
    StateMatrixf full_cov2D = StateMatrixf::Identity();
    bool is_valid = false;

    StateVectorf CovertToEigenVec2D();

    void InitFullCov2D();

    apStatus_t FromEigenVec2D(const StateVectorf &full_state);

    void TransformSelfByMatrix(const Eigen::Matrix4f &trans);

    static struct StructObjectState TransformByMatrix(
        const Eigen::Matrix4f &trans, const struct StructObjectState &data);
} ObjectState;

}  // namespace perception
}  // namespace senseAD
