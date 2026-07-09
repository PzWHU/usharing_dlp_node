/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Ma Quanmeng <maquanmeng@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>
#include <set>
#include <string>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Cholesky"
#include "perception_common/log/error.hpp"

#include "perception_common/base/data_type/state_buffer.hpp"
#include "perception_common/math_helper/timestamp_util.hpp"
#include "perception_common/state_estimation/filter/extended_kalman_filter.hpp"
#include "perception_common/state_estimation/measurement_model/lidar_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/radar_measure_model.hpp"
#include "perception_common/state_estimation/process_model/cv_process_model.hpp"
#include "perception_common/state_estimation/process_model/ca_process_model.hpp"
#include "perception_common/state_estimation/process_model/ct_process_model.hpp"
#include "perception_common/state_estimation/process_model/cp_process_model.hpp"
#include "perception_common/state_estimation/process_model/ctra_process_model.hpp"
#include "perception_common/state_estimation/process_model/cj_process_model.hpp"
#include "perception_common/state_estimation/process_model/process_model.hpp"
#include "perception_common/state_estimation/filter/imm_filter.hpp"
#include "perception_common/state_estimation/filter/imm_optimizer.hpp"
#include "perception_common/state_estimation/filter/imm_filter_base.hpp"
#include "perception_common/state_estimation/filter/vsimm_filter.hpp"
#include "perception_common/state_estimation/filter/unscented_kalman_filter.hpp"
#include "perception_common/state_estimation/estimator_core/base_estimator_core.hpp"
#include "perception_common/state_estimation/state/measure_state.hpp"

namespace senseAD {
namespace perception {

class IMMEstimatorCore : public EstimatorCoreBase {
 public:
    IMMEstimatorCore();
    virtual ~IMMEstimatorCore() = default;
    apStatus_t Init(const MeasureState& object_state,
                    const MeasureModelBasePtr& measure_ptr,
                    const IMMTrackerParameter& depth_tracker_param);
    apStatus_t Predict(const uint64_t timestamp_ns, ObjectState* object_state);
    /**!
     * @brief this interface will not change internal state
     * @note redefine predict function for temp state generation.
     */
    apStatus_t Predict(const uint64_t target_timestamp_ns,
                       const KFStateVecPtr prev_state_vec,
                       ObjectState* object_state);
    KFStateVecPtr GetStateBufferClosestStateVecPtr(const uint64_t timestamp_ns);
    apStatus_t Update(const MeasureModelBasePtr& measure_ptr);
    apStatus_t UpdateWithMainSensor(const MeasureModelBasePtr& measure_ptr);
    apStatus_t GetLastState(ObjectState* object_state);
    uint64_t GetLastStateTimestampNS();
    uint64_t GetLastMeasurementTimestampNS();
    apStatus_t GetLastSameSensorNameMeas(const std::string& sensor_name,
                                         MeasureState* measure_state) const;
    KFStateVecPtr GetStateBufferClosestBeforeStateVecPtr(
        const uint64_t timestamp_ns);
    apStatus_t GetLastMeasurement(MeasureState* measure_state) const;

    IMMFilterBasePtr GetIMMFilter() { return imm_filter_; }
    std::vector<BaseProcessModelPtr> GetProcessModelsVec() {
        return process_models_vec_;
    }
    void SetStateBuffer(
        std::shared_ptr<StateBuffer<KFStateVecPtr>> state_buffer);
    void UpdateStateBuffer(
        std::shared_ptr<StateBuffer<KFStateVecPtr>> state_buffer);
    void SetMeasureBuffer(
        std::shared_ptr<StateBuffer<MeasureModelBasePtr>> measure_buffer);
    void UpdateMeasureBuffer(
        std::shared_ptr<StateBuffer<MeasureModelBasePtr>> measure_buffer);

 private:
    apStatus_t AddMeasure(const MeasureModelBasePtr& measure_ptr);
    apStatus_t ProcessMainSensor(const uint64_t timestamp_ns);
    apStatus_t HandleFutureMeasureWithHighRate();
    apStatus_t GetStateAtTimeNoAddBuffer(const uint64_t timestamp_ns,
                                         ObjectState* state_out);
    /**!
     * @brief this interface do not involve ego-motion compensation and will not
     * change internal state
     */
    apStatus_t GetStateAtTimeNoAddBuffer(const uint64_t timestamp_ns,
                                         const KFStateVecPtr prev_state_vec,
                                         ObjectState* state_out);

    apStatus_t ClearOldState();
    KFStateVecPtr GenerateStateAtTime(const uint64_t timestamp_ns,
                                      const bool add_buffer = true);
    /**!
     * @brief generate a new state at time "timestamp_ns" with input state
     * "prev_state_vec_ptr", without changing internal state.
     */
    KFStateVecPtr GenerateStateAtTime(const uint64_t timestamp_ns,
                                      const KFStateVecPtr prev_state_vec_ptr);

    apStatus_t PrograteOneStep(const KFStateVecPtr& old_state,
                               const KFStateVecPtr& new_state);
    apStatus_t PrograteOneStepWithoutMotionCompsate(
        const KFStateVecPtr& old_state, const KFStateVecPtr& new_state);
    apStatus_t ConstructKalmanFilter(
        const IMMTrackerParameter& depth_tracker_param,
        std::vector<KalmanFilterBasePtr>* filter_vec);
    apStatus_t ConstructProcessModel(
        const IMMTrackerParameter& depth_tracker_param,
        const MeasureState& measure_state,
        std::vector<BaseProcessModelPtr>* process_model_vec);
    apStatus_t MeasureToObjectState(const MeasureState& measure_state,
                                    ObjectState* object_state);
    apStatus_t SetFilter(const std::string& filter_name,
                         const bool using_batch_filter);

 private:
    bool initialized_;
    double largest_time_gap_;
    double prograte_time_gap_;
    double clear_old_interval_;
    double add_newstate_time_gap_;
    double invalid_state_value_;
    // kalman filter pointer
    IMMFilterBasePtr imm_filter_;
    // process model
    std::vector<BaseProcessModelPtr> process_models_vec_;
    Eigen::MatrixXf transfer_prob_;
    Eigen::MatrixXf adjacent_mat_;
    std::vector<KalmanFilterBasePtr> filter_vec_;
    std::string current_filter_name_;
};
typedef std::shared_ptr<IMMEstimatorCore> IMMTrackerCorePtr;

REGISTER_CORE(IMMEstimatorCore);
}  // namespace perception
}  // namespace senseAD
