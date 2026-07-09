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
#include "perception_common/base/factory.hpp"

#include "proto/state_estimation.pb.h"  // NOLINT
#include "perception_common/state_estimation/state/state_info.hpp"
#include "perception_common/state_estimation/state/processmodel_info.hpp"
#include "perception_common/state_estimation/state/KFState.hpp"
#include "perception_common/math_helper/transform_util.hpp"

namespace senseAD {
namespace perception {

class BaseProcessModel {
 public:
    BaseProcessModel()
        : is_linear_(false),
          is_optimze_yaw_(false),
          is_optimze_bias_(false),
          model_type_(ProcessModelType::UNKNOWN_PROCESSMODEL) {}

    explicit BaseProcessModel(const BaseProcessModel& rhs) {
        is_linear_ = rhs.is_linear_;
        is_optimze_yaw_ = rhs.is_optimze_yaw_;
        is_optimze_yaw_ = rhs.is_optimze_bias_;
        s2l_ = rhs.s2l_;
        model_type_ = rhs.GetProcessModelType();
        depth_smooth_param_ = rhs.depth_smooth_param_;
    }

    virtual ~BaseProcessModel() = default;

    apStatus_t virtual Init(const DepthTrackerParameter& depth_smooth_param,
                            const bool is_optimze_yaw,
                            const bool is_optimze_bias) = 0;
    apStatus_t virtual GenerateProcessNoiseMatrix(
        const float dt,
        const KFStatePtr& state_ptr,
        Eigen::MatrixXf* process_noise_mat) {
        return AP_NOT_IMPLEMENTED;
    }
    apStatus_t virtual GenerateProcessSlimNoiseMatrix(
        const float dt,
        const KFStatePtr& state_ptr,
        Eigen::MatrixXf* process_noise_mat) {
        return AP_NOT_IMPLEMENTED;
    }
    apStatus_t virtual GenerateTransMatrix(const float dt,
                                           const KFStatePtr& state_ptr,
                                           Eigen::MatrixXf* trans_mat) {
        return AP_NOT_IMPLEMENTED;
    }
    apStatus_t virtual GenerateSlimTransMatrix(const float dt,
                                               const KFStatePtr& state_ptr,
                                               Eigen::MatrixXf* trans_mat) {
        return AP_NOT_IMPLEMENTED;
    }
    apStatus_t virtual StateTransFunWithoutNoise(const float dt,
                                                 const KFStatePtr& state_ptr,
                                                 Eigen::VectorXf* state_out) {
        return AP_NOT_IMPLEMENTED;
    }
    apStatus_t SlimStateTransFunWithoutNoise(const float dt,
                                             const KFStatePtr& state_ptr,
                                             Eigen::VectorXf* state_out) {
        return AP_NOT_IMPLEMENTED;
    }
    apStatus_t virtual StateTransFunWithNoise(const float dt,
                                              const Eigen::VectorXf& noise_vec,
                                              const KFStatePtr& state_ptr,
                                              Eigen::VectorXf* state_out) {
        return AP_NOT_IMPLEMENTED;
    }
    apStatus_t virtual GenerateNoiseMatrix(Eigen::MatrixXf* noise_mat) {
        return AP_NOT_IMPLEMENTED;
    }
    apStatus_t virtual AddMotionConstraints(const KFStatePtr& prev_state_ptr,
                                            const KFStatePtr& cur_state_ptr,
                                            ceres::Problem* problem) {
        return AP_NOT_IMPLEMENTED;
    }
    apStatus_t virtual AddPriorConstraints(const KFStatePtr& cur_state_ptr,
                                           ceres::Problem* problem) {
        return AP_NOT_IMPLEMENTED;
    }
    virtual std::string Name() const { return "BaseProcessModel"; }
    const MapOfEnumIdx<StateDimIdx, int>& GetMapOfSys2Loc() const {
        return s2l_;
    }
    bool IsLinear() { return is_linear_; }
    ProcessModelType GetProcessModelType() const { return model_type_; }
    apStatus_t virtual Predict(const KFStatePtr& state_in,
                               const KFStatePtr& state_out,
                               const double delta_t) {
        return AP_NOT_IMPLEMENTED;
    }

 protected:
    bool is_linear_;
    bool is_optimze_yaw_;
    bool is_optimze_bias_;
    ProcessModelType model_type_;
    MapOfEnumIdx<StateDimIdx, int> s2l_;
    DepthTrackerParameter depth_smooth_param_;
};
typedef std::shared_ptr<BaseProcessModel> BaseProcessModelPtr;
REGISTER_FACTORY(Common, BaseProcessModel);

#define REGISTER_PROCESSMODEL(name) \
    REGISTER_CLASS(Common, BaseProcessModel, name)

}  // namespace perception
}  // namespace senseAD
