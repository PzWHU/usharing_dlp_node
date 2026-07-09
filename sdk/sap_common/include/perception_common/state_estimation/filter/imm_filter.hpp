/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Jiang Qinhong <jiangqinhong@sensetime.com>
 * Zhang Shiquan <zhangshiquan@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>

#include "perception_common/log/error.hpp"
#include "perception_common/state_estimation/process_model/process_model.hpp"
#include "perception_common/state_estimation/measurement_model/measure_model.hpp"
#include "perception_common/state_estimation/state/KFState.hpp"
#include "perception_common/state_estimation/state/KFStateVec.hpp"
#include "perception_common/state_estimation/filter/kalman_filter_base.hpp"
#include "perception_common/state_estimation/filter/imm_filter_base.hpp"

#include "perception_common/state_estimation/state/state_info.hpp"
#include "perception_common/math_helper/angle_util.hpp"

namespace senseAD {
namespace perception {

class IMMFilter : public IMMFilterBase {
 public:
    IMMFilter();

    explicit IMMFilter(const IMMFilter& imm_filter);

    virtual ~IMMFilter() = default;

    apStatus_t virtual Init(
        const Eigen::MatrixXf& transfer_prob,
        const Eigen::MatrixXf& adjacent_mat,
        const std::vector<KalmanFilterBasePtr>& kalman_filter,
        const std::vector<BaseProcessModelPtr>& process_model);

    apStatus_t Predict(const KFStateVecPtr& state_in,
                       const KFStateVecPtr& state_out,
                       const double delta_t,
                       const std::vector<BaseProcessModelPtr>& process_model);

    apStatus_t Update(const KFStateVecPtr& state_in,
                      const std::shared_ptr<MeasureModelBase>& measure_model);

    apStatus_t Fusion(const KFStateVecPtr& state_in,
                      const KFStatePtr& state_out);

 private:
    apStatus_t ModelReinitialize(const KFStateVecPtr& state_in);
    apStatus_t UpdateModelProb(const KFStateVecPtr& state_in);

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
};
typedef std::shared_ptr<IMMFilter> IMMFilterPtr;
REGISTER_IMMFILTER(IMMFilter)
}  // namespace perception
}  // namespace senseAD
