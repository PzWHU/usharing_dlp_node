/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Ma Quanmeng <maquanmeng@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>
#include <string>
#include <utility>

#include "eigen3/Eigen/Dense"
#include "perception_common/base/data_type/object_label.hpp"
#include "perception_common/log/error.hpp"
#include "perception_common/state_estimation/state/object_state.hpp"

namespace senseAD {
namespace perception {

class KFState {
 public:
    KFState();

    explicit KFState(const std::shared_ptr<KFState> &state_ptr);

    ~KFState() = default;

    apStatus_t SetState(const StateVectorf &state_in);

    apStatus_t SetSlimState(const Eigen::VectorXf &state_in);

    Eigen::VectorXf GetSlimState();

    apStatus_t SetSlimCov(const Eigen::MatrixXf &cov_in);

    Eigen::MatrixXf GetSlimCov();

    ProcessModelType GetProcessModelType();

    void SetProcessModelType(ProcessModelType model_type);

    apStatus_t SetObjectState(const ObjectState &object_state);

    apStatus_t UpdateObjectState();

    apStatus_t SetPolyBox(const std::vector<Eigen::Vector2f> &poly_box);

    apStatus_t SetCov(const StateMatrixf &cov_in);

    apStatus_t SetTimestampNS(uint64_t timestamp_ns);

    apStatus_t SetSensorName(std::string sensor_name);

    apStatus_t SetModelProb(const Eigen::VectorXf &model_prob);

    apStatus_t SetPredictModelProb(const Eigen::VectorXf &model_prob);

    apStatus_t SetLikelihood(const float likelihood);

    apStatus_t SetObjectLabel(const ObjectLabel &label);

    const StateVectorf &GetState();

    const StateMatrixf &GetCov();

    const std::vector<Eigen::Vector2f> &GetPolyBox();

    const uint64_t GetTimestampNS();

    const std::string &GetSensorName();

    const Eigen::VectorXf &GetModelProb();

    const Eigen::VectorXf &GetPredictModelProb();

    const float GetLikelihood();

    const ObjectLabel &GetObjectLabel();

    const ObjectState &GetObjectState();

    ObjectState* GetObjectStatePtr();

    Eigen::VectorXf GetSlimStateVev(
        const MapOfEnumIdx<StateDimIdx, int> &sys2loc);

    apStatus_t SetStateFromSlimState(
        const Eigen::VectorXf &slim_state,
        const MapOfEnumIdx<StateDimIdx, int> &sys2loc);

    apStatus_t GetSlimeCovMat(const MapOfEnumIdx<StateDimIdx, int> &sys2loc,
                              Eigen::MatrixXf *silm_cov);

    apStatus_t SetCovFromSlimCov(const MapOfEnumIdx<StateDimIdx, int> &sys2loc,
                                 const Eigen::MatrixXf &slim_cov,
                                 Eigen::MatrixXf *full_cov);

    const MapOfEnumIdx<StateDimIdx, int> &GetMapOfSys2Loc();

    bool GetIsTransformed();

    bool GetIsMainSensor();

    void SetIsMainSensor(const bool main_sensor);

 private:
    ObjectState object_state_;
    Eigen::VectorXf model_prob_;
    Eigen::VectorXf predict_model_prob_;
    float likelihood_;
    ObjectLabel label_;
    MapOfEnumIdx<StateDimIdx, int> sys2loc_;

 public:
    std::vector<Eigen::Vector2f> ploygon_contour_;
    double weight_;
    Eigen::VectorXd high_precision_state_;
};

typedef std::shared_ptr<KFState> KFStatePtr;

}  // namespace perception
}  // namespace senseAD
