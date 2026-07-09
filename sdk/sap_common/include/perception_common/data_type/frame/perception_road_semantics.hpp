/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@sensetime.com>
 */
#pragma once

#include <vector>
#include <memory>
#include <string>

#include "Eigen/Core"

#include "perception_common/base/data_type/base.hpp"

#include "perception_common/data_type/sensor_info.hpp"
#include "perception_common/data_type/object/camera/traffic_light.hpp"
#include "perception_common/data_type/object/camera/traffic_sign.hpp"
#include "perception_common/data_type/object/camera/road_marker_object.hpp"
#include "perception_common/data_type/object/camera/board_pole.hpp"

namespace senseAD {
namespace perception {

typedef struct StructRoadSemantics {
    std::vector<TrafficLight> traffic_light_results;
    std::vector<TrafficSign> traffic_sign_results;
    std::vector<RoadMarker> road_marker_results;
    std::vector<BoardPole> board_pole_results;
} RoadSemantics;

typedef struct StructPerceptionRoadSemanticsFrame {
    // @brief sensor id, required
    SensorID sensor_id = SensorID::UNKNOWN_SENSOR;
    // @brief sensor name, required
    std::string sensor_name = "unknown_sensor";
    // @brief timestamp of current frame
    uint64_t timestamp_ns;
    RoadSemantics road_semantics;

    inline void Clear() {
        sensor_id = SensorID::UNKNOWN_SENSOR;
        timestamp_ns = 0;
        road_semantics.traffic_light_results.clear();
        road_semantics.traffic_sign_results.clear();
        road_semantics.road_marker_results.clear();
        road_semantics.board_pole_results.clear();
    }
} PerceptionRoadSemanticsFrame;

}  // namespace perception
}  // namespace senseAD
