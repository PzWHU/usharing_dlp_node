/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <set>
#include <memory>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include "perception_common/data_type/sensor_info.hpp"

namespace senseAD {
namespace perception {

enum class DangerousZoneLevel {
    NONE = 0,
    DANGEROUS = 1,
    SAFE = 2,
    UNSURE = 3,
};

enum class DangerousZoneType {
    NONE = 0,
    CHILD = 1,
    ADULT = 2,
    CYCLIST = 3,
    VEHICLE = 4,
    VEHICLE_BUS = 5,
    OTHERS = 6,
};

struct LidarDangerousZone {
    std::vector<cv::Point2f> polygon;
    DangerousZoneLevel dangerous_level;
    uint64_t dzone_id;
    std::vector<uint64_t> pre_dzone_ids;
    std::vector<uint64_t> occluder_ids;
    std::set<uint64_t> shadow_ids;
    // int esm_obj_num = 0;
};

struct LidarDangerousZoneFrame {
    uint64_t timestamp_ns = 0;
    SensorID sensor_id = SensorID::UNKNOWN_SENSOR;
    float height;
    DangerousZoneType type;
    std::vector<LidarDangerousZone> dzones;
};

struct LidarFOVFrame {
    uint64_t timestamp_ns = 0;
    SensorID sensor_id = SensorID::UNKNOWN_SENSOR;
    float grid_resolution;
    cv::Point2f xy_pos_range;
    cv::Point2i map_size;
    std::vector<uint8_t> fov_data;
};

}  // namespace perception
}  // namespace senseAD
