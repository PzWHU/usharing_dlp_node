/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <vector>
#include <string>

#include "Eigen/Core"

#include "perception_common/data_type/sensor_info.hpp"
#include "perception_common/data_type/object/base_object.hpp"
#include "perception_common/data_type/object/radar/input_radar_raw_object.hpp"

namespace senseAD {
namespace perception {

// @brief raw radar object input frame
typedef struct StructRadarRawInputFrame {
    // @brief sensor id of radar frame
    SensorID sensor_id = SensorID::UNKNOWN_SENSOR;
    std::string sensor_type = "unknown";
    // @brief calib param for frame_id to target frame: car_center(default)
    Eigen::Matrix4f calib_trans = Eigen::Matrix4f::Identity();
    // timestamp of current frame
    uint64_t timestamp_ns;
    // total object list of current frame
    std::vector<std::unique_ptr<InputRadarRawObject>> radar_objects_list;
    inline size_t size() const { return radar_objects_list.size(); }
    inline void clear() {
        timestamp_ns = 0;
        radar_objects_list.clear();
    }
} RadarRawInputFrame;

typedef std::unique_ptr<RadarRawInputFrame> RadarRawInputFrameUniquePtr;

}  // namespace perception
}  // namespace senseAD
