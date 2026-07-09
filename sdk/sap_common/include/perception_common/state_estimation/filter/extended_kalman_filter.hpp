/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Ma Quanmeng <maquanmeng@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>

#include "eigen3/Eigen/Dense"

#include "perception_common/log/error.hpp"
#include "perception_common/state_estimation/process_model/process_model.hpp"
#include "perception_common/state_estimation/measurement_model/measure_model.hpp"
#include "perception_common/state_estimation/filter/kalman_filter_base.hpp"

namespace senseAD {
namespace perception {

class ExtendedKalmanFilter : public KalmanFilterBase {
 public:
    ExtendedKalmanFilter() = default;
    explicit ExtendedKalmanFilter(const ExtendedKalmanFilter& filter)
        : KalmanFilterBase(filter) {}
    apStatus_t Predict(const KFStatePtr& state_in,
                       const KFStatePtr& state_out,
                       const double delta_t,
                       const BaseProcessModelPtr& process_model);
    apStatus_t Update(const KFStatePtr& state_in,
                      const std::shared_ptr<MeasureModelBase>& measure_model);
    apStatus_t UpdateCov(
        const KFStatePtr& state_in,
        const std::shared_ptr<MeasureModelBase>& measure_model);
};
REGISTER_KALMANFILTER(ExtendedKalmanFilter);
}  // namespace perception
}  // namespace senseAD
