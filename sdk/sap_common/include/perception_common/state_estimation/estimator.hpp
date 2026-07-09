/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Ma Quanmeng <maquanmeng@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>
#include <string>
#include <unordered_map>
#include <fstream>
#include <map>

#include "eigen3/Eigen/Dense"

#include "perception_common/log/error.hpp"
#include "proto/state_estimation.pb.h"  // NOLINT
#include "perception_common/base/data_type/sensor_info.hpp"
#include "perception_common/adapter/ego_car_info_adapter.hpp"

#include "perception_common/state_estimation/filter/extended_kalman_filter.hpp"

#include "perception_common/state_estimation/process_model/process_model.hpp"
#include "perception_common/state_estimation/process_model/cv_process_model.hpp"
#include "perception_common/state_estimation/process_model/ct_process_model.hpp"
#include "perception_common/state_estimation/process_model/cp_process_model.hpp"
#include "perception_common/state_estimation/process_model/ca_process_model.hpp"

#include "perception_common/state_estimation/measurement_model/measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/lidar_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/radar_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/camera_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/camera_velocity_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/camera_polar_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/camera_radar_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/radar_velocity_measure_model.hpp"
#include "perception_common/state_estimation/estimator_core/imm_estimator_core.hpp"

namespace senseAD {
namespace perception {
// this is a class of constant velority tracker
class Estimator {
 public:
    Estimator()
        : initialized_(false),
          time_since_update_(0),
          hit_streak_(0),
          id_(0),
          hits_(0),
          age_(0),
          is_updated_(false),
          base_time_(0),
          local_win_size_(5),
          is_radar_position_measurement_valid_(false),
          last_predict_timestamp_(0),
          is_stable_(true),
          time_since_camera_update_(true),
          time_to_state_windows_s_(2) {}
    ~Estimator() = default;
    apStatus_t Init(const MeasureState& measure_state,
                    const EstimatorParameter& depth_tracker_param);
    apStatus_t Init(const MeasureState& measure_state,
                    const MeasureModelBasePtr& measure_model,
                    const EstimatorParameter& depth_tracker_param);
    apStatus_t Update(const MeasureState& measure_state);
    apStatus_t Update(const MeasureModelBasePtr& measure_model);
    apStatus_t Predict(const uint64_t timestamp_ns, ObjectState* object_state);
    // Get State at Specific Time, Do NOT Add State to Buffer
    apStatus_t GetStateAtTimeWithoutAddBuffer(
        const uint64_t tgt_timestamp_ns,
        ObjectState* state_out,
        std::fstream* log_handler = nullptr);
    apStatus_t GetStateAtTimeWithoutAddBufferDetach(
        const uint64_t tgt_timestamp_ns,
        ObjectState* state_out,
        std::fstream* log_handler = nullptr);
    // Get State at Specific Time, Add to Tmp Association Buffer
    apStatus_t GetStateAtTimeWithCacheStateBuffer(const uint64_t timestamp_ns,
                                                  ObjectState* state_out);
    apStatus_t GetLastSameSensorNameMeas(const std::string& sensor_name,
                                         MeasureState* measure_state) const;
    apStatus_t ClearCacheStateBuffer();

    apStatus_t State(ObjectState* object_state) const {
        return core_ptr_->GetLastState(object_state);
    }
    apStatus_t SetID(const uint64_t& id) {
        id_ = id;
        return AP_SUCCESS;
    }
    uint64_t GetLastStateTimestampNS() const;
    uint64_t GetLastMeasurementTimestampNS() const;
    apStatus_t GetLastMeasurement(MeasureState* measure_state) const;
    const MeasureState& GetLastMeasurement();
    apStatus_t SetLastMeasurement(const MeasureState& measure_state);
    apStatus_t GetEstimatorParam(EstimatorParameter* estimator_param) const;
    apStatus_t SetEstimatorParam(const EstimatorParameter& estimator_param);
    const double TimeSecondSinceUpdate() const { return time_since_update_; }
    void SetSecondSinceUpdate(const double& time_since_update) {
        time_since_update_ = time_since_update;
    }
    double TimeSinceCameraUpdate() const { return time_since_camera_update_; }
    void SetTimeSinceCameraUpdate(double time) {
        time_since_camera_update_ = time;
    }
    size_t GetHitStreak() { return hit_streak_; }
    void SetHitStreak(const size_t& hit_streak) { hit_streak_ = hit_streak; }
    void GetTimeToState(std::map<uint64_t, ObjectState>* time_to_state) const {
        *time_to_state = time_to_state_;
    }
    void SetTimeToState(const std::map<uint64_t, ObjectState>& time_to_state) {
        time_to_state_ = time_to_state;
    }
    const uint64_t GetPredictTimestamp() const {
        return last_predict_timestamp_;
    }
    void SetPredictTimestamp(const uint64_t& last_predict_timestamp) {
        last_predict_timestamp_ = last_predict_timestamp;
    }

    const uint64_t ID() const { return id_; }
    const double Age() const { return age_; }
    void SetAge(const double& age) { age_ = age; }
    const size_t Hits() const { return hits_; }
    void SetHits(uint64_t hits) { hits_ = hits; }
    const bool IsStable() const { return is_stable_; }
    void SetIsStable(bool is_stable) { is_stable_ = is_stable; }
    const bool Initialized() const { return initialized_; }
    const bool IsUpdated() const { return is_updated_; }
    const EstimatorCoreBasePtr& GetEstimatorCore() { return core_ptr_; }
    void SetEstimatorCore(const EstimatorCoreBasePtr core_ptr) {
        core_ptr_ = core_ptr;
    }
    const DepthTrackerParameter& GetDepthTrackerParameter() const {
        return depth_tracker_param_;
    }
    apStatus_t SetDepthTrackerParameter(
        const DepthTrackerParameter& depth_tracker_param) {
        depth_tracker_param_ = depth_tracker_param;
        return AP_SUCCESS;
    }
    void HitsPlus() { hits_++; }

 private:
    apStatus_t CheckMeasureBufferIsTransfromed();
    apStatus_t CheckStateBufferIsTransfromed();
    apStatus_t MeasureCoorToLocalWord(const uint64_t start_timestamp_ns,
                                      const uint64_t to_timestamp_ns);
    apStatus_t FutureMeasureToLocalWord(uint64_t to_timestamp_ns);
    apStatus_t StateCoorToLocalWord(const uint64_t start_timestamp_ns,
                                    const uint64_t to_timestamp_ns);
    apStatus_t StateCoorToLocalWord(const KFStateVecPtr prev_state_vec_ptr,
                                    const uint64_t to_timestamp_ns);
    apStatus_t LocalWordToMeasureCoor(const uint64_t start_timestamp_ns,
                                      const uint64_t from_timestamp_ns);
    apStatus_t LocalWordToStateCoor(const uint64_t start_timestamp_ns,
                                    const uint64_t from_timestamp_ns);
    apStatus_t FindStartTime(const uint64_t base_timestamp_ns,
                             const int local_win_size,
                             uint64_t* start_timestamp_ns);
    apStatus_t ClearOldStateInCacheStateBuffer(const uint64_t time_gap);

 public:
    apStatus_t TransformMeasure(const uint64_t src_timestamp,
                                const uint64_t dst_timestamp,
                                MeasureState* state);

    apStatus_t TransformState(const uint64_t src_timestamp,
                              const uint64_t dst_timestamp,
                              ObjectState* state);
    apStatus_t GetTransformFromSecToDst(const uint64_t src_timestamp,
                                        const uint64_t dst_timestamp,
                                        Eigen::Matrix4f* trans);

 private:
    // System dimensions
    bool initialized_;
    double time_since_update_;
    size_t hit_streak_;
    uint64_t id_;
    size_t hits_;
    double age_;
    bool is_updated_;
    // System Setting
    EstimatorParameter estimator_param_;
    DepthTrackerParameter depth_tracker_param_;
    EstimatorCoreBasePtr core_ptr_;
    uint64_t base_time_;
    int local_win_size_;
    bool is_radar_position_measurement_valid_;
    std::map<uint64_t, ObjectState> time_to_state_;
    uint64_t time_to_state_windows_s_;
    uint64_t last_predict_timestamp_;
    bool is_stable_;
    double time_since_camera_update_;
    MeasureState last_measure_;
    std::fstream* log_handler_ = nullptr;
};

typedef std::shared_ptr<Estimator> EstimatorPtr;
typedef std::shared_ptr<const Estimator> EstimatorConstPtr;

}  // namespace perception
}  // namespace senseAD
