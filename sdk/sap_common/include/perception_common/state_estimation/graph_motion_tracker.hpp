/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Song Zhourui <songzhourui1@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>
#include <string>
#include <unordered_map>
#include <map>

#include "eigen3/Eigen/Dense"

#include "perception_common/log/error.hpp"
#include "proto/state_estimation.pb.h"
#include "perception_common/base/data_type/sensor_info.hpp"
#include "perception_common/base/data_type/state_buffer.hpp"
#include "perception_common/state_estimation/measurement_model/measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/lidar_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/radar_measure_model.hpp"
#include "perception_common/state_estimation/measurement_model/radar_velocity_measure_model.hpp"
#include "perception_common/state_estimation/process_model/cv_process_model.hpp"
#include "perception_common/state_estimation/process_model/ca_process_model.hpp"
#include "perception_common/state_estimation/process_model/ct_process_model.hpp"
#include "perception_common/state_estimation/process_model/cp_process_model.hpp"
#include "perception_common/state_estimation/filter/kalman_filter_base.hpp"
#include "perception_common/state_estimation/filter/extended_kalman_filter.hpp"
#include "perception_common/state_estimation/filter/unscented_kalman_filter.hpp"
#include "perception_common/state_estimation/state/KFState.hpp"
#include "perception_common/state_estimation/graph_optimizer/graph_optimizer.hpp"
#include "perception_common/state_estimation/graph_optimizer/graph_tracker_core.hpp"

namespace senseAD {
namespace perception {

class GraphTracker {
 public:
    GraphTracker()
        : initialized_(false),
          time_since_update_(0),
          latest_measure_time_(0),
          hit_streak_(0),
          id_(0),
          hits(0),
          age_(0),
          is_updated_(false),
          is_global_mode_(true),
          graph_tracker_core_(new GraphTrackerCore()) {}
    ~GraphTracker() = default;

    apStatus_t Predict(const uint64_t current_time_ns,
                       Eigen::VectorXf *predicted_state);
    apStatus_t Update(const float observation_base_cov,
                      const Eigen::VectorXf &observation,
                      const std::vector<bool> &observation_validity,
                      const std::string &sensor_name,
                      const uint64_t time_ns,
                      const std::vector<Eigen::Vector2f> &polygon,
                      const float static_score,
                      Eigen::VectorXf *updated_state = nullptr);
    apStatus_t Init(const Eigen::VectorXf &state,
                    const GraphTrackerParameter &param,
                    const uint64_t time_ns,
                    const std::string &sensor_name,
                    const std::vector<Eigen::Vector2f> &polygon,
                    const uint64_t object_id,
                    const ObjectLabel &object_label);
    apStatus_t GetStateAtTime(const uint64_t time_ns, Eigen::VectorXf *state);
    apStatus_t GetPolyBoxAtTime(const uint64_t time_ns,
                                std::vector<Eigen::Vector2f> *poly_box);
    double TimeSecondSinceUpdate() const {
        return static_cast<double>(time_since_update_) * 1e-9;
    }

 private:
    apStatus_t ChangeMeasureToGlobalCoordinate(
        const uint64_t time_ns, const MeasureModelBasePtr &measure_model);

    apStatus_t ChangeStateToLocalCoordinate(const uint64_t time_ns,
                                            Eigen::VectorXf *state);
    apStatus_t QueryEgoPose(const uint64_t time_ns, Eigen::Matrix4f *Twc);

    apStatus_t ChangeStateToGlobalCoordinate(const uint64_t time_ns,
                                             Eigen::VectorXf *state,
                                             Eigen::MatrixXf *cov = nullptr);

    apStatus_t Query3DTransformFrom2To(uint64_t from_time_ns,
                                       uint64_t to_time_ns,
                                       Eigen::Matrix4f *from2to);

    apStatus_t AddLidarMeasurement(
        const Eigen::VectorXf &observation,
        const std::vector<bool> &observation_validity,
        const uint64_t time_ns,
        float static_score,
        const std::vector<Eigen::Vector2f> &polygon);

    apStatus_t AddRadarMeasurement(
        const Eigen::VectorXf &measurement,
        const std::vector<bool> &measurement_validity,
        uint64_t time_ns);

 private:
    bool initialized_;
    std::string main_sensor_name_;
    uint64_t time_since_update_;
    uint64_t latest_measure_time_;
    uint64_t hit_streak_;
    uint64_t id_;
    uint64_t hits;
    uint64_t age_;
    int state_dim_;
    bool is_updated_;
    bool is_global_mode_;
    std::unique_ptr<GraphTrackerCore> graph_tracker_core_;
    std::vector<std::string> measure_types_;
    std::unordered_map<std::string, int> name2_measure_dim_;
    GraphTrackerParameter graph_tracker_param_;
    bool is_optimize_yaw_;
};

using GraphTrackerPtr = std::shared_ptr<GraphTracker>;
using GraphTrackerConstPtr = std::shared_ptr<const GraphTracker>;
}  // namespace perception
}  // namespace senseAD
