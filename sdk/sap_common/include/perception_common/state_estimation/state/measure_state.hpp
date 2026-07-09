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
#include "perception_common/base/data_type/sensor_info.hpp"
#include "perception_common/log/error.hpp"
#include "perception_common/base/data_type/bbox_2d.hpp"
#include "perception_common/state_estimation/state/state_info.hpp"
#include "perception_common/math_helper/angle_util.hpp"

namespace senseAD {
namespace perception {
// base object
typedef struct StructMeasureState {
    // type
    ObjectLabel label = ObjectLabel::UNKNOWN;
    // sensor name
    std::string sensor_name;
    std::string merge_sensor_name;
    std::string lateral_merge_sensor_name;
    std::string velocity_sensor_name;
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
    Eigen::Vector3i position_validity = Eigen::Vector3i(1, 1, 0);

    // 3D direction info
    Eigen::Vector3f direction = Eigen::Vector3f(1, 0, 0);
    Eigen::Vector3i direction_validity = Eigen::Vector3i(0, 0, 0);
    float yaw = 0;
    float yaw_variance = 0;
    bool yaw_validity = false;
    // bird view direction
    // vehicle coordinate system, x-axis forward is 0
    // increase anticlockwise
    float theta = 0.0f;  // [rad], range:[-pi,pi]
    float theta_variance = 0.0f;
    bool theta_validity = false;

    // motion info
    float static_score = 0.0f;
    // object motion param: velocity, acceleration. yaw rate
    Eigen::Vector3f velocity = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f velocity_bias = Eigen::Vector3f(0, 0, 0);
    // covariance matrix of the velocity uncertainty, required
    Eigen::Matrix3f velocity_uncertainty = Eigen::Matrix3f::Identity();
    Eigen::Vector3i velocity_validity = Eigen::Vector3i(0, 0, 0);
    Eigen::Vector3i velocity_bias_validity = Eigen::Vector3i(0, 0, 0);
    // brief acceleration of the object, required
    Eigen::Vector3f acceleration = Eigen::Vector3f(0, 0, 0);
    // brief covariance matrix of the acceleration uncertainty, required
    Eigen::Matrix3f acceleration_uncertainty = Eigen::Matrix3f::Identity();
    Eigen::Vector3i acceleration_validity = Eigen::Vector3i(0, 0, 0);

    Eigen::Matrix3f position_velocity_uncertainty = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f velocity_acceleration_uncertainty = Eigen::Matrix3f::Zero();

    Eigen::Vector3f jerk = Eigen::Vector3f(0, 0, 0);
    Eigen::Matrix3f jerk_uncertainty = Eigen::Matrix3f::Identity();
    Eigen::Vector3i jerk_validity = Eigen::Vector3i(0, 0, 0);

    MotionDirection motion_direction = MotionDirection::UNKNOWN;
    // yaw rate
    float yaw_rate = 0;  // [rad/s]
    bool yaw_rate_validity = false;
    float var_yaw_rate = 0;  // [rad/s] standard deviation
    // meas bias between sensors
    float x_bias = 0;
    float y_bias = 0;
    bool velocity_uncertainty_validity = false;
    Eigen::Vector3i bias_validity = Eigen::Vector3i(0, 0, 0);
    // main sensor information
    bool is_main_sensor = false;
    bool is_optimze_camera_radar_bias = false;
    bool is_optimze_lidar_radar_bias = false;
    bool is_optimze_yaw = false;
    bool has_main_sensor = false;
    std::string main_sensor_name;
    bool is_transformed = false;
    uint64_t target_timestamp = 0;
    std::vector<Eigen::Vector3f> position_gaussian_means;
    std::vector<Eigen::Matrix3f> position_gaussian_covariances;
    std::vector<float> gaussian_weights;
    Bbox2D bbox2d;
    float overlap_ratio = 0;
    bool is_stability = false;
    bool fix_velocity = false;
    bool fix_state = false;
    bool is_valid = true;
    bool lateral_strong_stability = false;

    std::vector<std::string> measure_sensor_names;
    bool potential_maneuver = false;
    float eps_max_threshold = 25.0f;
    float eps_limit_threshold = 100.0f;
    Eigen::Vector4f Q_scale_factor = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);

    static struct StructMeasureState TransformByMatrix(
        const Eigen::Matrix4f& trans, const struct StructMeasureState& data);
} MeasureState;

typedef std::shared_ptr<MeasureState> MeasureStatePtr;

}  // namespace perception
}  // namespace senseAD
