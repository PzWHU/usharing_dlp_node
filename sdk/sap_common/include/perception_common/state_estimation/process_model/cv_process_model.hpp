/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Ma Quanmeng <maquanmeng@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>
#include <string>

#include "eigen3/Eigen/Dense"

#include "perception_common/log/error.hpp"
#include "proto/state_estimation.pb.h"  // NOLINT
#include "perception_common/state_estimation/process_model/process_model.hpp"
#include "perception_common/state_estimation/state/state_info.hpp"
#include "perception_common/state_estimation/process_model/motion_cost_function.hpp"
#include "perception_common/math_helper/timestamp_util.hpp"
#include "perception_common/math_helper/transform_util.hpp"

namespace senseAD {
namespace perception {

// this is a class of constant velority tracker
class CVProcessModel : public BaseProcessModel {
 public:
    CVProcessModel() = default;
    explicit CVProcessModel(CVProcessModel& model) : BaseProcessModel(model) {}

    apStatus_t Init(const DepthTrackerParameter& depth_smooth_param,
                    const bool is_optimze_yaw,
                    const bool is_optimze_bias);
    apStatus_t GenerateTransMatrix(const float& dt, Eigen::MatrixXf* trans_mat);
    apStatus_t GenerateProcessNoiseMatrix(const float dt,
                                          const KFStatePtr& state_ptr,
                                          Eigen::MatrixXf* process_noise_mat);
    apStatus_t GenerateProcessSlimNoiseMatrix(
        const float dt,
        const KFStatePtr& state_ptr,
        Eigen::MatrixXf* process_noise_mat);
    apStatus_t GenerateTransMatrix(const float dt,
                                   const KFStatePtr& state_ptr,
                                   Eigen::MatrixXf* trans_mat);
    apStatus_t GenerateSlimTransMatrix(const float dt,
                                       const KFStatePtr& state_ptr,
                                       Eigen::MatrixXf* trans_mat);
    apStatus_t StateTransFunWithoutNoise(const float dt,
                                         const KFStatePtr& state_ptr,
                                         Eigen::VectorXf* state_out);
    apStatus_t SlimStateTransFunWithoutNoise(const float dt,
                                             const KFStatePtr& state_ptr,
                                             Eigen::VectorXf* state_out);
    apStatus_t AddMotionConstraints(const KFStatePtr& prev_state_ptr,
                                    const KFStatePtr& cur_state_ptr,
                                    ceres::Problem* problem);
    apStatus_t AddPriorConstraints(const KFStatePtr& cur_state_ptr,
                                   ceres::Problem* problem);
    apStatus_t Predict(const KFStatePtr& state_in,
                       const KFStatePtr& state_out,
                       const double delta_t);

    virtual ~CVProcessModel() = default;
    virtual std::string Name() const { return "CV_ProcessModel"; }
    virtual const ProcessModelType Type() { return ProcessModelType::CV; }
};
REGISTER_PROCESSMODEL(CVProcessModel);

}  // namespace perception
}  // namespace senseAD
