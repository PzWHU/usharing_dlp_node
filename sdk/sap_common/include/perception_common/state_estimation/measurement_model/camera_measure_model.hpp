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

namespace senseAD {
namespace perception {

class CameraMeasureModel : public MeasureModelBase {
 public:
    CameraMeasureModel(const MeasureState& measure_state,
                       const DepthTrackerParameter& depth_smooth_param);

    explicit CameraMeasureModel(const CameraMeasureModel& camera_measure_model);

    virtual ~CameraMeasureModel() = default;

    apStatus_t GenerateMeasureMatrix(const KFStatePtr& state_in,
                                     Eigen::MatrixXf* measure_mat);
    apStatus_t GenerateSlimMeasureMatrix(const KFStatePtr& state_in,
                                         Eigen::MatrixXf* measure_mat);
    apStatus_t GenerateMeasureNoiseMatrix(const KFStatePtr& state_ptr,
                                          Eigen::MatrixXf* measure_noise_mat);
    apStatus_t GenerateObservedState(const KFStatePtr& state_in,
                                     Eigen::VectorXf* observed_state);
    apStatus_t AddMeasureConstraints(const KFStatePtr& cur_state_ptr,
                                     ceres::Problem* problem);
    Eigen::VectorXf Diff(const Eigen::VectorXf& observed_vec);
    int GetMeasurementDim() { return state2meas_.size(); }
    apStatus_t GetMaxLikehoodGaussianModel(const KFStatePtr state_ptr,
                                           const Eigen::VectorXf& diff,
                                           Eigen::MatrixXf* covariance);
    apStatus_t GetMeasureStateDims(std::vector<int>* meas_dims,
                                   std::vector<int>* state_dims);
    Eigen::VectorXf GetMeasurement();
    bool IsCamera() { return true; }
};
}  // namespace perception
}  // namespace senseAD
