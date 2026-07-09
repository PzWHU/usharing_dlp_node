/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "perception_common/base/data_type/bbox_2d.hpp"

namespace senseAD {
namespace perception {

// @brief radar roadside output interface
typedef struct RadarRoadsidesCurves {
    uint64_t timestamp_ns;
    Eigen::Vector4f left_side = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 0.0f);
    Eigen::Vector4f right_side = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 0.0f);
    bool left_valid = false;
    bool right_valid = false;
    Bbox2D left_bbox_2d;
    Bbox2D right_bbox_2d;
    float max_left_fitting_distance = 0;
    float max_right_fitting_distance = 0;
} RadarRoadsides;
using RadarRoadsidesPtr = std::shared_ptr<RadarRoadsides>;
using RadarRoadsidesConstPtr = std::shared_ptr<const RadarRoadsides>;

}  // namespace perception
}  // namespace senseAD
