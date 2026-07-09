/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Jiang Qinhong <jiangqinhong@sensetime.com>
 * Zhang Shiquan <zhangshiquan@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>

#include "eigen3/Eigen/Dense"

#include "perception_common/log/error.hpp"
#include "perception_common/base/data_type/state_buffer.hpp"
#include "perception_common/state_estimation/process_model/process_model.hpp"
#include "perception_common/state_estimation/measurement_model/measure_model.hpp"
#include "perception_common/state_estimation/state/KFState.hpp"

namespace senseAD {
namespace perception {

class KalmanFilterBase {
 public:
    KalmanFilterBase() = default;
    explicit KalmanFilterBase(const KalmanFilterBase& kalman_filter_base) {}
    virtual ~KalmanFilterBase() = default;
    apStatus_t virtual Predict(const KFStatePtr& state_in,
                               const KFStatePtr& state_out,
                               const double delta_t,
                               const BaseProcessModelPtr& process_model);
    apStatus_t virtual Update(
        const KFStatePtr& state_in,
        const std::shared_ptr<MeasureModelBase>& measure_model);
    apStatus_t virtual UpdateCov(
        const KFStatePtr& state_in,
        const std::shared_ptr<MeasureModelBase>& measure_model);
    apStatus_t UpdateOneToOneMeasureModel(
        const KFStatePtr& state_in, const MeasureModelBasePtr& measure_model);
    apStatus_t BatchUpdate(
        const int window_batch_size,
        const int model_index,
        const KFStatePtr& state_in,
        const BaseProcessModelPtr& process_model,
        const std::shared_ptr<StateBuffer<KFStateVecPtr> >& state_buffer,
        const std::shared_ptr<StateBuffer<MeasureModelBasePtr> >&
            measure_model_buffer,
        ceres::Problem* problem);
};
typedef std::shared_ptr<KalmanFilterBase> KalmanFilterBasePtr;
REGISTER_FACTORY(Common, KalmanFilterBase);

#define REGISTER_KALMANFILTER(name) \
    REGISTER_CLASS(Common, KalmanFilterBase, name)

}  // namespace perception
}  // namespace senseAD
