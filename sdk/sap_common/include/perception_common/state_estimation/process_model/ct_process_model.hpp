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
#include "perception_common/state_estimation/process_model/motion_cost_function.hpp"
#include "perception_common/math_helper/timestamp_util.hpp"

namespace senseAD {
namespace perception {

// this is a class of constant velority tracker
class CTProcessModel : public BaseProcessModel {
 public:
    CTProcessModel() : epsilon_(1e-7) {}
    explicit CTProcessModel(CTProcessModel& model)
        : BaseProcessModel(model), epsilon_(1e-7) {}

    apStatus_t virtual Init(const DepthTrackerParameter& depth_smooth_param,
                            const bool is_optimze_yaw,
                            const bool is_optimze_bias);

    apStatus_t GenerateTransMatrix(const float dt,
                                   const KFStatePtr& state_ptr,
                                   Eigen::MatrixXf* trans_mat);
    apStatus_t GenerateSlimTransMatrix(const float dt,
                                       const KFStatePtr& state_ptr,
                                       Eigen::MatrixXf* trans_mat);
    apStatus_t GenerateProcessNoiseMatrix(const float dt,
                                          const KFStatePtr& state_ptr,
                                          Eigen::MatrixXf* process_noise_mat);
    apStatus_t GenerateProcessSlimNoiseMatrix(
        const float dt,
        const KFStatePtr& state_ptr,
        Eigen::MatrixXf* process_noise_mat);
    apStatus_t StateTransFunWithoutNoise(const float dt,
                                         const KFStatePtr& state_ptr,
                                         Eigen::VectorXf* state_out);
    apStatus_t SlimStateTransFunWithoutNoise(const float dt,
                                             const KFStatePtr& state_ptr,
                                             Eigen::VectorXf* state_out);
    apStatus_t StateTransFunWithNoise(const float dt,
                                      const Eigen::VectorXf& noise_vec,
                                      const KFStatePtr& state_ptr,
                                      Eigen::VectorXf* state_out);
    apStatus_t AddMotionConstraints(const KFStatePtr& prev_state_ptr,
                                    const KFStatePtr& cur_state_ptr,
                                    ceres::Problem* problem);
    apStatus_t AddPriorConstraints(const KFStatePtr& cur_state_ptr,
                                   ceres::Problem* problem);
    apStatus_t GenerateNoiseMatrix(Eigen::MatrixXf* noise_mat);
    apStatus_t Predict(const KFStatePtr& state_in,
                       const KFStatePtr& state_out,
                       const double delta_t);
    virtual ~CTProcessModel() = default;

    virtual std::string Name() const { return "CT_ProcessModel"; }
    virtual const ProcessModelType Type() { return ProcessModelType::CT; }

 private:
    const float epsilon_;
};
REGISTER_PROCESSMODEL(CTProcessModel);

}  // namespace perception
}  // namespace senseAD
