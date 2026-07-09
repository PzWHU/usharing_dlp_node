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
class WeightConstraintCostFunction : public ceres::CostFunction {
 public:
    WeightConstraintCostFunction(const double constant_weight_sum,
                                 const int motion_model_num);

    virtual ~WeightConstraintCostFunction() = default;

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const;

 private:
    const double constant_weight_sum_;
    const int motion_model_num_;
};

class WeightMarkovConstraintCostFunction : public ceres::CostFunction {
 public:
    explicit WeightMarkovConstraintCostFunction(
        const Eigen::MatrixXd& trans_prob);

    virtual ~WeightMarkovConstraintCostFunction() = default;

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const;

 private:
    const Eigen::MatrixXd trans_prob_;
};

}  // namespace perception
}  // namespace senseAD
