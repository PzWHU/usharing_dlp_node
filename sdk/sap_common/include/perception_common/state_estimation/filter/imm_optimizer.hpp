/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Ma Quanmeng <maquanmeng@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>

#include "ceres/ceres.h"

#include "perception_common/log/error.hpp"
#include "perception_common/state_estimation/process_model/process_model.hpp"
#include "perception_common/state_estimation/measurement_model/measure_model.hpp"
#include "perception_common/state_estimation/state/KFState.hpp"
#include "perception_common/state_estimation/state/KFStateVec.hpp"
#include "perception_common/state_estimation/filter/kalman_filter_base.hpp"
#include "perception_common/state_estimation/filter/imm_filter_base.hpp"
#include "perception_common/state_estimation/state/state_info.hpp"
#include "perception_common/math_helper/angle_util.hpp"
#include "perception_common/math_helper/timestamp_util.hpp"
#include "perception_common/state_estimation/filter/imm_motion_cost_function.hpp"

namespace senseAD {
namespace perception {

class IMMOptimizer : public IMMFilterBase {
 public:
    IMMOptimizer();

    IMMOptimizer(const IMMOptimizer& imm_filter);

    virtual ~IMMOptimizer() = default;

    apStatus_t virtual Init(
        const Eigen::MatrixXf& transfer_prob,
        const Eigen::MatrixXf& adjacent_mat,
        const std::vector<KalmanFilterBasePtr>& kalman_filter,
        const std::vector<BaseProcessModelPtr>& process_model);

    apStatus_t Predict(const KFStateVecPtr& state_in,
                       const KFStateVecPtr& state_out,
                       const double delta_t,
                       const std::vector<BaseProcessModelPtr>& process_model);

    apStatus_t Fusion(const KFStateVecPtr& state_in,
                      const KFStatePtr& state_out);
    apStatus_t Update(
        const int window_batch_size,
        const KFStateVecPtr& state_in,
        const std::shared_ptr<StateBuffer<KFStateVecPtr> >& state_buffer,
        const std::shared_ptr<StateBuffer<MeasureModelBasePtr> >&
            measure_buffer);

 private:
    apStatus_t ModelReinitialize(const KFStateVecPtr& state_in);
    apStatus_t UpdateModelProb(const KFStateVecPtr& state_in);
    apStatus_t UpdateCov(
        const KFStateVecPtr& state_in,
        const std::shared_ptr<MeasureModelBase>& measure_model);

 private:
    Eigen::MatrixXf transfer_prob_;
    std::vector<KalmanFilterBasePtr> kalman_filter_;
    std::vector<BaseProcessModelPtr> process_models_;
    float epsilon_;
    float change_model_threshold_;
    float likelihood_threshold_;
    float markov_sum_threshold_;
    float max_model_prob_;
    bool initialized_;
    int windows_batch_size_;
};
typedef std::shared_ptr<IMMOptimizer> IMMOptimizerPtr;
REGISTER_IMMFILTER(IMMOptimizer)
}  // namespace perception
}  // namespace senseAD
