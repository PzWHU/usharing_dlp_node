/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Ma Quanmeng <maquanmeng@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>
#include <string>
#include <algorithm>

#include "ceres/ceres.h"

#include "eigen3/Eigen/Dense"

#include "perception_common/log/error.hpp"
#include "proto/state_estimation.pb.h"  // NOLINT
#include "perception_common/base/data_type/object_label.hpp"

#include "perception_common/state_estimation/state/KFState.hpp"
#include "perception_common/state_estimation/state/measure_state.hpp"
#include "perception_common/math_helper/angle_util.hpp"
#include "perception_common/math_helper/transform_util.hpp"
#include "perception_common/math_helper/kalman_util.hpp"

namespace senseAD {
namespace perception {

class MeasureModelBase {
 public:
    MeasureModelBase(const MeasureState& measure_state,
                     const DepthTrackerParameter& depth_smooth_param,
                     bool is_linear,
                     bool one_to_one);

    explicit MeasureModelBase(const MeasureModelBase& measure_model_base);

    virtual ~MeasureModelBase() = default;

    apStatus_t virtual GenerateMeasureMatrix(const KFStatePtr& state_in,
                                             Eigen::MatrixXf* measure_mat) = 0;
    apStatus_t virtual GenerateSlimMeasureMatrix(const KFStatePtr& state_in,
                                                 Eigen::MatrixXf* measure_mat);
    apStatus_t virtual GenerateMeasureNoiseMatrix(
        const KFStatePtr& state_ptr, Eigen::MatrixXf* measure_noise_mat) = 0;
    apStatus_t virtual GenerateObservedState(
        const KFStatePtr& state_in, Eigen::VectorXf* observed_state) = 0;
    int virtual GetMeasurementDim() = 0;
    Eigen::VectorXf virtual Diff(const Eigen::VectorXf& observed_vec) = 0;
    Eigen::VectorXf virtual GetMeasurement() = 0;
    apStatus_t virtual GenerateObservedStateWithNoise(
        const Eigen::VectorXf& noise_vec,
        const KFStatePtr& state_in,
        Eigen::VectorXf* observed_state);
    apStatus_t virtual AddMeasureConstraints(const KFStatePtr& cur_state_ptr,
                                             ceres::Problem* problem);
    apStatus_t virtual SetLidarRadarBias(const float x_bias,
                                         const float y_bias);
    apStatus_t virtual GetMaxLikehoodGaussianModel(const KFStatePtr state_ptr,
                                                   const Eigen::VectorXf& diff,
                                                   Eigen::MatrixXf* covariance);
    bool IsLinear();
    bool IsOneToOne() const;
    uint64_t GetTimestampNS();
    const ObjectLabel& GetObjectLabel();
    std::string GetSensorName();
    bool IsOptimzeYaw(bool debug);
    bool IsOptimzeBias();
    void SetMeasureState(const MeasureState& measure_state);
    float YawRateThreshold() const;
    float VelXThreshold() const;
    float VelYThreshold() const;
    float AccXThreshold() const;
    float AccYThreshold() const;
    const MeasureState& GetMeasureState();
    void SetTimestampNS(uint64_t timestamp_ns);
    // those who provide such measurement should implement correspongding
    // functions
    virtual apStatus_t GetPosition(Eigen::Vector2f* position);
    virtual apStatus_t SetPosition(const Eigen::Vector2f& p);
    virtual apStatus_t GetAcceleration(Eigen::Vector2f* acc);
    virtual apStatus_t SetAcceleration(const Eigen::Vector2f& acc);
    virtual apStatus_t GetVelocity(Eigen::Vector2f* vel);
    virtual apStatus_t SetVelocity(const Eigen::Vector2f& vel);
    virtual apStatus_t SetYawMeasurement(const float yaw);
    virtual apStatus_t GetYawMeasurement(float* yaw);
    virtual apStatus_t GetPolygon(std::vector<Eigen::Vector2f>* poly_box) const;
    virtual apStatus_t SetPolygon(const std::vector<Eigen::Vector2f>& poly_box);
    virtual apStatus_t GetStaticScore(float* static_score) const;
    virtual apStatus_t SetStaticScore(const float static_score);
    virtual apStatus_t GetTheta(float* theta);
    virtual apStatus_t SetTheta(float theta);

    bool virtual IsLidar();
    bool virtual IsRadar();
    bool virtual IsCamera();

    Eigen::Vector3f GetPositionBias() const;
    Eigen::VectorXf virtual DiffWithPosBias(const Eigen::VectorXf& observed_vec,
                                            const Eigen::VectorXf& bias);
    Eigen::VectorXf virtual DiffWithPosVelBias(
        const Eigen::VectorXf& observed_vec,
        const Eigen::VectorXf& bias,
        const Eigen::VectorXf& velocity_bias);
    void SetProcessModelType(ProcessModelType model_type);
    apStatus_t LimiteState(const KFStatePtr& state_in) const;
    apStatus_t virtual GetMeasureStateDims(std::vector<int>* meas_dims,
                                           std::vector<int>* state_dims);

 protected:
    DepthTrackerParameter depth_smooth_param_;
    bool is_linear_;
    float epsilon_;
    MeasureState measure_state_;
    bool one_state_one_measure_;
    MapOfEnumIdx<MeasDimIdx, int> state2meas_;

 protected:
    int state_dim_;
    int measurement_dim_;
    Eigen::VectorXf measure_;
    uint64_t timestamp_ns_;
    bool is_optimze_yaw_;
    bool is_optimze_bias_proto_;
    bool is_optimze_bias_;
    float meas_yaw_;
    ProcessModelType model_type_;

 private:
    ObjectLabel object_label_;
    std::string sensor_name_;
};
typedef std::shared_ptr<MeasureModelBase> MeasureModelBasePtr;
}  // namespace perception
}  // namespace senseAD
