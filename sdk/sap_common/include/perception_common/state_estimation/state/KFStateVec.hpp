/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Ma Quanmeng <maquanmeng@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>
#include <string>

#include "eigen3/Eigen/Dense"
#include "perception_common/base/data_type/object_label.hpp"
#include "perception_common/log/error.hpp"

#include "perception_common/state_estimation/process_model/process_model.hpp"
#include "perception_common/state_estimation/state/KFState.hpp"

namespace senseAD {
namespace perception {

class KFStateVec {
 public:
    KFStateVec();

    explicit KFStateVec(int vec_size, uint64_t timestamp_ns);

    explicit KFStateVec(const std::shared_ptr<KFStateVec> &states_ptr);

    explicit KFStateVec(const KFStateVec &kf_state_vec);

    ~KFStateVec();

    apStatus_t SetState(const std::vector<StateVectorf> &state_in);

    apStatus_t SetPolyBox(
        const std::vector<std::vector<Eigen::Vector2f>> &poly_box);

    apStatus_t SetCov(const std::vector<StateMatrixf> &cov_in);

    apStatus_t SetTimestampNS(uint64_t timestamp_ns);

    apStatus_t SetModelProb(const std::vector<Eigen::VectorXf> &model_prob);

    apStatus_t SetPredictModelProb(
        const std::vector<Eigen::VectorXf> &model_prob);

    apStatus_t SetLikelihood(const std::vector<float> &likelihood);

    apStatus_t SetObjectLabel(const ObjectLabel &label);

    apStatus_t SetNextModelSetIdxs(const std::vector<int> &model_set);

    apStatus_t SetCurModelSetIdxs(const std::vector<int> &model_set);

    const StateVectorf &GetStateAtIndex(int idx);

    const StateMatrixf &GetCovAtIndex(int idx);

    const std::vector<Eigen::Vector2f> &GetPolyBoxAtIndex(int idx);

    uint64_t GetTimestampNS();

    const Eigen::VectorXf &GetModelProbAtIndex(int idx);

    const Eigen::VectorXf &GetPredictModelProbAtIndex(int idx);

    const float GetLikelihoodAtIndex(int idx);

    const ObjectLabel &GetObjectLabel();

    size_t GetSize();

    const KFStatePtr &GetKFStateAtIndex(int idx);

    apStatus_t CheckStateVecIsEmpty();

    apStatus_t CheckStateVecSize(int idx);

    std::vector<KFStatePtr> GetStatePtrVec();

    const std::vector<int> &GetNextModelSetIdxs();

    const std::vector<int> &GetCurModelSetIdxs();

    apStatus_t SetPrevKFStatePtr(std::shared_ptr<KFStateVec> state_ptr);

    const std::shared_ptr<KFStateVec> GetPrevKFStatePtr();

    bool GetIsTransformed();

    bool GetIsMainSensor();

    void SetIsMainSensor(bool main_sensor);

 private:
    std::vector<KFStatePtr> states_vec_;
    uint64_t timestamp_ns_ = 0;
    std::string sensor_name_;
    ObjectLabel label_ = ObjectLabel::UNKNOWN;
    std::weak_ptr<KFStateVec> prev_state_ptr_;
    std::vector<int> next_modelset_idxs_;
    std::vector<int> cur_modelset_idxs_;
};
typedef std::shared_ptr<KFStateVec> KFStateVecPtr;
}  // namespace perception
}  // namespace senseAD
