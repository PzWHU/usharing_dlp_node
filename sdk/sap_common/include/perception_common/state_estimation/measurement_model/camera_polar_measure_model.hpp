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

class CameraPolarMeasureModel : public MeasureModelBase {
 public:
    CameraPolarMeasureModel(const MeasureState& measure_state,
                            const DepthTrackerParameter& depth_smooth_param);

    explicit CameraPolarMeasureModel(
        const CameraPolarMeasureModel& camera_measure_model);

    virtual ~CameraPolarMeasureModel() = default;

    apStatus_t GenerateMeasureMatrix(const KFStatePtr& state_in,
                                     Eigen::MatrixXf* measure_mat);
    apStatus_t GenerateMeasureNoiseMatrix(const KFStatePtr& state_ptr,
                                          Eigen::MatrixXf* measure_noise_mat);
    apStatus_t GenerateObservedState(const KFStatePtr& state_in,
                                     Eigen::VectorXf* observed_state);
    Eigen::VectorXf Diff(const Eigen::VectorXf& observed_vec);
    apStatus_t GenerateObservedStateWithNoise(const Eigen::VectorXf& noise_vec,
                                              const KFStatePtr& state_in,
                                              Eigen::VectorXf* observed_state);
    apStatus_t AddMeasureConstraints(const KFStatePtr& cur_state_ptr,
                                     ceres::Problem* problem);
    int GetMeasurementDim() { return state2meas_.size(); }
    Eigen::VectorXf GetMeasurement();

    bool IsCamera() { return true; }

 private:
    const Eigen::Vector2f ToPolar(const Eigen::VectorXf& state);
    const float MThrehHuber(const float x);
    const float Huber(const float x);

 private:
    float range_low_threshold_;
    float range_high_threshold_;
};

}  // namespace perception
}  // namespace senseAD
