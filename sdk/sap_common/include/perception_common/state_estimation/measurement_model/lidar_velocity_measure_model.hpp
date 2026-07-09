/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Ma Quanmeng <maquanmeng@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>

#include "eigen3/Eigen/Dense"

#include "perception_common/log/error.hpp"
#include "proto/state_estimation.pb.h"  // NOLINT
#include "perception_common/math_helper/angle_util.hpp"
#include "perception_common/state_estimation/measurement_model/measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/measure_cost_function.hpp"
#include "perception_common/state_estimation/state/state_info.hpp"

namespace senseAD {
namespace perception {

class LidarVelocityMeasureModel : public MeasureModelBase {
 public:
    LidarVelocityMeasureModel(const MeasureState& measure_state,
                              const DepthTrackerParameter& depth_smooth_param);

    explicit LidarVelocityMeasureModel(
        const LidarVelocityMeasureModel& lidar_measure_model);

    virtual ~LidarVelocityMeasureModel() = default;

    apStatus_t GenerateMeasureMatrix(const KFStatePtr& state_in,
                                     Eigen::MatrixXf* measure_mat);
    apStatus_t GenerateMeasureNoiseMatrix(const KFStatePtr& state_ptr,
                                          Eigen::MatrixXf* measure_noise_mat);
    apStatus_t GenerateObservedState(const KFStatePtr& state_in,
                                     Eigen::VectorXf* observed_state);
    apStatus_t AddMeasureConstraints(const KFStatePtr& cur_state_ptr,
                                     ceres::Problem* problem);
    Eigen::VectorXf Diff(const Eigen::VectorXf& observed_vec);
    int GetMeasurementDim() { return state2meas_.size(); }
    Eigen::VectorXf GetMeasurement();
    bool IsLidar() { return true; }

    virtual apStatus_t GetPosition(Eigen::Vector2f* position) {
        if (nullptr == position) {
            return AP_NULL_PTR;
        }
        *position = measure_state_.position.head(2);
        return AP_SUCCESS;
    }
    virtual apStatus_t SetPosition(const Eigen::Vector2f& p) {
        measure_state_.position.head(2) = p;
        return AP_SUCCESS;
    }
    virtual apStatus_t GetVelocity(Eigen::Vector2f* vel) {
        if (nullptr == vel) {
            return AP_NULL_PTR;
        }
        *vel = measure_state_.velocity.head(2);
        return AP_SUCCESS;
    }
    virtual apStatus_t SetVelocity(const Eigen::Vector2f& vel) {
        measure_state_.velocity.head(2).block(2, 0, 2, 1) = vel;
        return AP_SUCCESS;
    }
    virtual apStatus_t SetYawMeasurement(const float yaw) {
        measure_state_.yaw = yaw;
        return AP_SUCCESS;
    }
    virtual apStatus_t GetYawMeasurement(float* yaw) {
        if (nullptr == yaw) {
            return AP_NULL_PTR;
        }
        *yaw = measure_state_.yaw;
        return AP_SUCCESS;
    }
    virtual apStatus_t GetPolygon(
        std::vector<Eigen::Vector2f>* poly_box) const {
        if (poly_box == nullptr) {
            return AP_NULL_PTR;
        }
        *poly_box = measure_state_.poly_box;
        return AP_SUCCESS;
    }
    virtual apStatus_t SetPolygon(
        const std::vector<Eigen::Vector2f>& poly_box) {
        AP_LERROR(LidarVelocityMeasureModel)
            << "set with poly box of " << poly_box.size() << " points";
        measure_state_.poly_box = poly_box;
        return AP_SUCCESS;
    }

    virtual apStatus_t GetStaticScore(float* static_score) const {
        if (nullptr == static_score) {
            return AP_NULL_PTR;
        }
        *static_score = measure_state_.static_score;
        return AP_SUCCESS;
    }
    virtual apStatus_t SetStaticScore(const float static_score) {
        measure_state_.static_score = static_score;
        return AP_SUCCESS;
    }
    virtual apStatus_t TransMeasurementCoordinate(
        const Eigen::Matrix4f& from2to);
};

}  // namespace perception
}  // namespace senseAD
