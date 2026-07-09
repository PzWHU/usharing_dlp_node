/*
 * Copyright (C) 2020-2021 by SenseTime Group Limited. All rights reserved.
 * He Yuan <heyuan@sensetime.com>
 * Zhang Shiquan <zhangshiquan@senseauto.com>
 */
#pragma once

#include <vector>
#include <memory>
#include <string>

#include "Eigen/Core"

#include "perception_common/data_type/sensor_info.hpp"

namespace senseAD {
namespace perception {

typedef struct StructPerceptionRoadArea {
    std::vector<uint8_t> image;
    uint16_t width;
    uint16_t height;
} PerceptionRoadArea;

typedef struct StructPerceptionRoadAreaFrame {
    // @brief sensor id, required
    SensorID sensor_id = SensorID::UNKNOWN_SENSOR;
    // @brief timestamp of current frame
    uint64_t timestamp_ns;
    PerceptionRoadArea road_area;
} PerceptionRoadAreaFrame;

}  // namespace perception
}  // namespace senseAD
