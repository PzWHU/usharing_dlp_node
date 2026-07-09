/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <vector>
#include <string>

#include "Eigen/Core"

#include "perception_common/data_type/object/camera/map_traffic_light_group.hpp"

namespace senseAD {
namespace perception {

// @brief general object frame
typedef struct StructPerceptionMapTrafficLightFrame {
    // timestamp of current frame
    uint64_t timestamp_ns;
    // total object list of current frame
    std::vector<MapTrafficLightGroup> tl_group_list;
    inline size_t Size() const { return tl_group_list.size(); }

    std::vector<std::string> selected_camera_name;

    inline void Clear() {
        timestamp_ns = 0;
        tl_group_list.clear();
        selected_camera_name.clear();
    }
} PerceptionMapTrafficLightFrame;

typedef std::shared_ptr<PerceptionMapTrafficLightFrame>
    PerceptionMapTrafficLightFramePtr;

}  // namespace perception
}  // namespace senseAD
