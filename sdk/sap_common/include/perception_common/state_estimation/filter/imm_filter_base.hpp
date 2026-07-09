/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Jiang Qinhong <jiangqinhong@sensetime.com>
 * Zhang Shiquan <zhangshiquan@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>
#include "perception_common/base/factory.hpp"

#include "perception_common/log/error.hpp"
#include "perception_common/state_estimation/process_model/cv_process_model.hpp"
#include "perception_common/state_estimation/process_model/ca_process_model.hpp"
#include "perception_common/state_estimation/process_model/ct_process_model.hpp"
#include "perception_common/state_estimation/process_model/cp_process_model.hpp"
#include "perception_common/state_estimation/process_model/ctra_process_model.hpp"
#include "perception_common/state_estimation/process_model/cj_process_model.hpp"
#include "perception_common/state_estimation/process_model/process_model.hpp"
#include "perception_common/state_estimation/measurement_model/measure_model.hpp"
#include "perception_common/state_estimation/state/KFState.hpp"
#include "perception_common/state_estimation/state/KFStateVec.hpp"
#include "perception_common/state_estimation/filter/kalman_filter_base.hpp"
#include "perception_common/state_estimation/filter/imm_filter_base.hpp"
#include "perception_common/state_estimation/filter/extended_kalman_filter.hpp"
#include "perception_common/state_estimation/filter/unscented_kalman_filter.hpp"
#include "perception_common/state_estimation/state/state_info.hpp"
#include "perception_common/math_helper/angle_util.hpp"

namespace senseAD {
namespace perception {

class IMMFilterBase {
 public:
    IMMFilterBase() = default;

    virtual ~IMMFilterBase() = default;

    explicit IMMFilterBase(const IMMFilterBase& imm_filter_base) {}

    apStatus_t virtual Init(
        const Eigen::MatrixXf& transfer_prob,
        const Eigen::MatrixXf& adjacent_mat,
        const std::vector<KalmanFilterBasePtr>& kalman_filter,
        const std::vector<BaseProcessModelPtr>& process_model) = 0;

    apStatus_t virtual Predict(
        const KFStateVecPtr& state_in,
        const KFStateVecPtr& state_out,
        const double delta_t,
        const std::vector<BaseProcessModelPtr>& process_model) = 0;

    apStatus_t virtual Update(
        const KFStateVecPtr& state_in,
        const std::shared_ptr<MeasureModelBase>& measure_model) {
        return AP_NOT_IMPLEMENTED;
    }

    apStatus_t virtual Fusion(const KFStateVecPtr& state_in,
                              const KFStatePtr& state_out) = 0;
    apStatus_t virtual Update(
        const int window_batch_size,
        const KFStateVecPtr& state_in,
        const std::shared_ptr<StateBuffer<KFStateVecPtr> >& state_buffer,
        const std::shared_ptr<StateBuffer<MeasureModelBasePtr> >&
            measure_buffer) {
        return AP_NOT_IMPLEMENTED;
    }
};
typedef std::shared_ptr<IMMFilterBase> IMMFilterBasePtr;
REGISTER_FACTORY(Common, IMMFilterBase);

#define REGISTER_IMMFILTER(name) REGISTER_CLASS(Common, IMMFilterBase, name)
}  // namespace perception
}  // namespace senseAD
