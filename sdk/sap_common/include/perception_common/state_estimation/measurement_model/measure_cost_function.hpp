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
#include "perception_common/state_estimation/measurement_model/measure_model.hpp"

namespace senseAD {
namespace perception {
class MeasureConstraintCostFunction : public ceres::CostFunction {
 public:
    MeasureConstraintCostFunction(const int residuals_dim,
                                  const KFStatePtr& cur_state,
                                  const Eigen::MatrixXd& sqrt_information,
                                  MeasureModelBase* measure_model_ptr);

    virtual ~MeasureConstraintCostFunction() = default;

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const;

 private:
    const int residuals_dim_;
    MeasureModelBase* measure_model_ptr_;
    const Eigen::MatrixXd sqrt_information_;
    KFStatePtr cur_state_ptr_;
};
}  // namespace perception
}  // namespace senseAD
