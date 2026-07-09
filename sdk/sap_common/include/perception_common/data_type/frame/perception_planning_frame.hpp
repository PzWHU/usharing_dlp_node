/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <vector>
#include <string>
#include <utility>

#include "Eigen/Core"

#include "perception_common/data_type/object/base_object.hpp"
#include "perception_common/data_type/sensor_info.hpp"
#include "perception_common/data_type/frame/frame_header.hpp"

namespace senseAD {
namespace perception {

enum class Gear {
    None = 0,
    Drive = 1,
    Neutral = 2,
    Drive2 = 3,
    Park = 4,
};

enum class TurnSignal {
    None = 0,
    Left = 1,
    Right = 2,
    Emergency = 3,
};
enum class TrajectoryMode {
    None = 0,
    Acc = 1,
    Lka = 2,
};
enum class TrajectoryState {
    None = 0,
    Both = 1,
    OnlyLat = 2,
    OnlyLon = 3,
    Error = 4,
};

struct PlanningInstruction {
    bool updated_by_vehicle_status = false;
    bool enter_autodrive_flag = false;
    void Clear() {
        updated_by_vehicle_status = false;
        enter_autodrive_flag = false;
    }
};

struct TrajectoryPoint {
    Eigen::Vector2f position = Eigen::Vector2f(0.0, 0.0);
    Eigen::Vector2f direction = Eigen::Vector2f(0.0, 0.0);
    double velocity = 0.0;
    float theta = 0.0f;
    float curvature = 0.0f;
    double sum_distance = 0.0;
    double time_difference = 0.0;  // relative time, unit, s
    double yaw_rate = 0.0;
    double lon_acc = 0.0;
    double lat_acc = 0.0;
    double lat_jerk = 0.0;  // m/s3
    double front_wheel_angle = 0.0;
    uint64_t timestamp = 0;    // unit: ns
    double theta_error = 0.0;  // used in ideal point

    TrajectoryPoint() = default;

    TrajectoryPoint operator+(const TrajectoryPoint &rhs) const;

    TrajectoryPoint operator-(const TrajectoryPoint &rhs) const;

    TrajectoryPoint operator*(const double num) const;
};

// @brief general object frame
#ifdef WITH_SERIALIZATION_INTERFACE
typedef struct StructPerceptionPlanningFrame : public FrameHeader {
#else
typedef struct StructPerceptionPlanningFrame {
#endif  // WITH_SERIALIZATION_INTERFACE
    // frame id
    SensorID sensor_id = SensorID::UNKNOWN_SENSOR;
    // timestamp of current frame
    uint64_t traj_base_timestamp_ns;

    Gear gear = Gear::None;
    TurnSignal turn_signal = TurnSignal::None;
    TrajectoryState traj_state = TrajectoryState::None;
    TrajectoryMode traj_mode = TrajectoryMode::None;
    PlanningInstruction instruction;
    std::vector<TrajectoryPoint> traj_point_array;

    void Clear() {
        traj_point_array.clear();
        instruction.Clear();
        gear = Gear::None;
        turn_signal = TurnSignal::None;
        traj_state = TrajectoryState::None;
        traj_mode = TrajectoryMode::None;
    }

#ifdef WITH_SERIALIZATION_INTERFACE
    FrameType GetFrameType() final {
        return FrameType::PERCEPTION_OBJECT_FRAME;
    }
#endif  // WITH_SERIALIZATION_INTERFACE
} PerceptionPlanningFrame;

typedef std::shared_ptr<PerceptionPlanningFrame> PerceptionPlanningFramePtr;

}  // namespace perception
}  // namespace senseAD
