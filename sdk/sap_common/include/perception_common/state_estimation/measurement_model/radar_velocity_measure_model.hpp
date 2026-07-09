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
#include "perception_common/state_estimation/measurement_model/measure_model.hpp"

namespace senseAD {
namespace perception {

class RadarVelocityMeasureModel : public MeasureModelBase {
 public:
    RadarVelocityMeasureModel(const MeasureState& measure_state,
                              const DepthTrackerParameter& depth_smooth_param);

    explicit RadarVelocityMeasureModel(
        const RadarVelocityMeasureModel& radar_measure_model);

    virtual ~RadarVelocityMeasureModel() = default;
    apStatus_t virtual GenerateMeasureMatrix(const KFStatePtr& state_in,
                                             Eigen::MatrixXf* measure_mat);
    apStatus_t virtual GenerateMeasureNoiseMatrix(
        const KFStatePtr& state_ptr, Eigen::MatrixXf* measure_noise_mat);
    apStatus_t virtual GenerateObservedState(const KFStatePtr& state_in,
                                             Eigen::VectorXf* observed_state);
    apStatus_t virtual AddMeasureConstraints(const KFStatePtr& cur_state_ptr,
                                             ceres::Problem* problem);
    apStatus_t virtual GenerateObservedStateWithNoise(
        const Eigen::VectorXf& noise_vec,
        const KFStatePtr& state_in,
        Eigen::VectorXf* observed_state);
    Eigen::VectorXf virtual Diff(const Eigen::VectorXf& observed_vec);
    int virtual GetMeasurementDim() { return state2meas_.size(); }
    Eigen::VectorXf GetMeasurement();
    bool IsRadar() { return true; }

 private:
    apStatus_t Range(const Eigen::VectorXf& state_input, float* rho);
    apStatus_t Bearing(const Eigen::VectorXf& state_input, float* theta);
    apStatus_t RangeRate(const Eigen::VectorXf& state_input, float* ran_vel);
};

}  // namespace perception
}  // namespace senseAD
