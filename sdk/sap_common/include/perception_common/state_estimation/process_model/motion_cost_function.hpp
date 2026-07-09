/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Ma Quanmeng <maquanmeng@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>
#include <string>

#include "ceres/ceres.h"
#include "eigen3/Eigen/Dense"

#include "perception_common/log/error.hpp"
#include "proto/state_estimation.pb.h"  // NOLINT
#include "perception_common/state_estimation/process_model/process_model.hpp"
#include "perception_common/state_estimation/state/state_info.hpp"
#include "perception_common/state_estimation/state/KFState.hpp"

namespace senseAD {
namespace perception {
class MotionConstraintCostFunction : public ceres::CostFunction {
 public:
    MotionConstraintCostFunction(const double dt,
                                 const KFStatePtr& cur_state,
                                 const std::vector<StateDimIdx>& params_dims,
                                 const KFStatePtr& prev_state,
                                 const Eigen::MatrixXd& sqrt_information,
                                 BaseProcessModel* process_model);

    virtual ~MotionConstraintCostFunction() = default;

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const;

 private:
    const double dt_;
    KFStatePtr cur_state_ptr_;
    const std::vector<StateDimIdx> params_dims_;
    KFStatePtr prev_state_ptr_;
    BaseProcessModel* process_model_;
    Eigen::MatrixXd sqrt_information_;
};

class PriorConstraintCostFunction : public ceres::CostFunction {
 public:
    PriorConstraintCostFunction(const KFStatePtr& cur_state,
                                const std::vector<StateDimIdx>& params_dims,
                                const Eigen::MatrixXd& sqrt_information,
                                BaseProcessModel* process_model);

    virtual ~PriorConstraintCostFunction() = default;

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const;

 private:
    std::vector<StateDimIdx> params_dims_;
    KFStatePtr prev_state_ptr_;
    BaseProcessModel* process_model_;
    Eigen::MatrixXd sqrt_information_;
    KFStatePtr cur_state_ptr_;
};
}  // namespace perception
}  // namespace senseAD
