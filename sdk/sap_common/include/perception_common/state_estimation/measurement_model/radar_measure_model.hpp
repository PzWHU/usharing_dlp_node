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
#include "perception_common/state_estimation/state/state_info.hpp"

namespace senseAD {
namespace perception {

class RadarMeasureModel : public MeasureModelBase {
 public:
    RadarMeasureModel(const MeasureState& measure_state,
                      const DepthTrackerParameter& depth_smooth_param);

    explicit RadarMeasureModel(const RadarMeasureModel& radar_measure_model);

    virtual ~RadarMeasureModel() = default;

    apStatus_t GenerateMeasureMatrix(const KFStatePtr& state_in,
                                     Eigen::MatrixXf* measure_mat);
    apStatus_t GenerateMeasureNoiseMatrix(const KFStatePtr& state_ptr,
                                          Eigen::MatrixXf* measure_noise_mat);
    apStatus_t GenerateObservedState(const KFStatePtr& state_in,
                                     Eigen::VectorXf* observed_state);
    apStatus_t SetLidarRadarBias(const float x_bias, const float y_bias);
    Eigen::VectorXf Diff(const Eigen::VectorXf& observed_vec);
    apStatus_t GenerateObservedStateWithNoise(const Eigen::VectorXf& noise_vec,
                                              const KFStatePtr& state_in,
                                              Eigen::VectorXf* observed_state);
    apStatus_t AddMeasureConstraints(const KFStatePtr& cur_state_ptr,
                                     ceres::Problem* problem);
    int GetMeasurementDim();
    Eigen::VectorXf GetMeasurement();

    apStatus_t virtual TransMeasurementCoordinate(
        const Eigen::Matrix4f& from2to);
    apStatus_t virtual GetTheta(float* theta);
    apStatus_t virtual SetTheta(float theta);
    apStatus_t virtual GetVelocity(Eigen::Vector2f* vel);
    bool IsRadar();

 private:
    apStatus_t Range(const Eigen::VectorXf& state_input, float* rho);
    apStatus_t Bearing(const Eigen::VectorXf& state_input, float* theta);
    apStatus_t RangeRate(const Eigen::VectorXf& state_input, float* ran_vel);

 private:
    float range_rate_;
    float range_;
    float theta_;
    float range_acc_;
};

}  // namespace perception
}  // namespace senseAD
