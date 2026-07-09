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
#include "perception_common/base/factory.hpp"

#include "perception_common/base/data_type/state_buffer.hpp"
#include "perception_common/state_estimation/measurement_model/measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/camera_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/camera_polar_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/camera_radar_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/camera_velocity_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/camera_lateral_velocity_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/lidar_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/radar_velocity_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/camera_velocity_acceleration_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/camera_acceleration_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/camera_lateral_velocity_acceleration_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/lidar_velocity_measure_model.hpp"
#include "perception_common/state_estimation/state/measure_state.hpp"
#include "perception_common/state_estimation/state/object_state.hpp"

namespace senseAD {
namespace perception {
class EstimatorCoreBase {
 public:
    EstimatorCoreBase()
        : state_buffer_(new StateBuffer<KFStateVecPtr>()),
          measure_buffer_(new StateBuffer<MeasureModelBasePtr>()),
          future_measure_buffer_(new StateBuffer<MeasureModelBasePtr>()),
          using_batch_filter_(false) {}
    virtual ~EstimatorCoreBase() = default;
    apStatus_t virtual Init(const MeasureState& object_state,
                            const MeasureModelBasePtr& measure_ptr,
                            const IMMTrackerParameter& depth_tracker_param) = 0;
    /**!
     * @brief this interface will change internal state
     */
    apStatus_t virtual Predict(const uint64_t timestamp_ns,
                               ObjectState* object_state) = 0;
    /**!
     * @brief this interface will not change internal state
     * @note redefine predict function for temp state generation.
     */
    apStatus_t virtual Predict(const uint64_t target_timestamp_ns,
                               const KFStateVecPtr prev_state_vec,
                               ObjectState* object_state) = 0;
    KFStateVecPtr virtual GetStateBufferClosestStateVecPtr(
        const uint64_t timestamp_ns) = 0;
    apStatus_t virtual Update(const MeasureModelBasePtr& measure_ptr) = 0;
    apStatus_t virtual UpdateWithMainSensor(
        const MeasureModelBasePtr& measure_ptr) = 0;
    apStatus_t virtual GetLastState(ObjectState* object_state) = 0;
    uint64_t virtual GetLastStateTimestampNS() = 0;
    uint64_t virtual GetLastMeasurementTimestampNS() = 0;
    apStatus_t virtual GetLastMeasurement(
        MeasureState* measure_state) const = 0;
    void virtual SetStateBuffer(
        std::shared_ptr<StateBuffer<KFStateVecPtr>> state_buffer) = 0;
    void virtual UpdateStateBuffer(
        std::shared_ptr<StateBuffer<KFStateVecPtr>> state_buffer) = 0;
    void virtual SetMeasureBuffer(
        std::shared_ptr<StateBuffer<MeasureModelBasePtr>> measure_buffer) = 0;
    void virtual UpdateMeasureBuffer(
        std::shared_ptr<StateBuffer<MeasureModelBasePtr>> measure_buffer) = 0;
    std::shared_ptr<StateBuffer<KFStateVecPtr>> GetStateBuffer() {
        return state_buffer_;
    }
    apStatus_t virtual GetLastSameSensorNameMeas(
        const std::string& sensor_name, MeasureState* measure_state) const = 0;
    KFStateVecPtr virtual GetStateBufferClosestBeforeStateVecPtr(
        const uint64_t timestamp_ns) = 0;
    std::shared_ptr<StateBuffer<MeasureModelBasePtr>> GetMeasureBuffer() {
        return measure_buffer_;
    }
    std::shared_ptr<StateBuffer<MeasureModelBasePtr>> GetFutureMeasureBuffer() {
        return future_measure_buffer_;
    }
    apStatus_t virtual SetFilter(const std::string& filter_name,
                                 const bool using_batch_filter) {
        using_batch_filter_ = using_batch_filter;
        return AP_SUCCESS;
    }

 protected:
    std::shared_ptr<StateBuffer<KFStateVecPtr>> state_buffer_;
    std::shared_ptr<StateBuffer<MeasureModelBasePtr>> measure_buffer_;
    std::shared_ptr<StateBuffer<MeasureModelBasePtr>> future_measure_buffer_;
    bool using_batch_filter_;
    int window_batch_size_;
};
typedef std::shared_ptr<EstimatorCoreBase> EstimatorCoreBasePtr;
REGISTER_FACTORY(Common, EstimatorCoreBase);

#define REGISTER_CORE(name) REGISTER_CLASS(Common, EstimatorCoreBase, name)
}  // namespace perception
}  // namespace senseAD
