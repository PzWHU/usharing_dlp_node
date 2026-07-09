/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <vector>
#include <string>

#include "Eigen/Core"

#include "perception_common/base/data_type/traffic_light_info.hpp"
#include "perception_common/base/data_type/bbox_2d.hpp"
#include "perception_common/data_type/object/base_object.hpp"
#include "perception_common/base/data_type/geometry.hpp"

namespace senseAD {
namespace perception {

// @brief image plane info
typedef struct StructTLImage2DInfo {
    std::string camera_name;
    senseAD::perception::Bbox2D detected_bbox;
    senseAD::perception::Point2f projected_center_point;
    bool is_projection_valid = false;
} TLImage2DInfo;

// @brief output traffic light frame
typedef struct StructMapTrafficLight {
    // general info
    uint64_t timestamp_ns = 0;
    // image 2D info
    std::vector<TLImage2DInfo> image_2d_info;

    // geometry
    // world coordinates, optional
    std::vector<senseAD::perception::Vec3f> world_geometry;
    // ego veh coordinate, optional
    senseAD::perception::Vec3f local_geometry;

    // id
    std::string id;
    std::string group_id;
    // turn type
    // TODO(zhangshiquan): remove turn type
    uint16_t turn_type = static_cast<uint16_t>(TurnType::FREE);
    // color label
    uint16_t color_label = static_cast<uint16_t>(TrafficSignal::NONE);
    float during_time = -1;
} MapTrafficLight;

typedef struct StructMapTrafficLightGroup {
    std::vector<MapTrafficLight> traffic_lights;
    std::string group_id;
    uint16_t color_label = static_cast<uint16_t>(TrafficSignal::NONE);
    uint16_t turn_type = static_cast<uint16_t>(TurnType::FREE);
    float during_time = -1;
} MapTrafficLightGroup;

}  // namespace perception
}  // namespace senseAD
