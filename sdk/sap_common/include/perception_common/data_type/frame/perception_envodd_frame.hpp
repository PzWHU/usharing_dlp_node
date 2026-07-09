/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Xu Zhipeng <xuzhipeng@senseauto.com>
 */
#pragma once

#include <memory>
#include <vector>
#include <string>

#include "Eigen/Core"
#include "perception_common/data_type/frame/perception_blob_frame.hpp"
#include "perception_common/data_type/frame/perception_scene_frame.hpp"

namespace senseAD {
namespace perception {

typedef struct StructPerceptionEnvODDFrame {
    // @brief sensor id, required
    SensorID sensor_id = SensorID::UNKNOWN_SENSOR;
    // @brief sensor name, required
    std::string sensor_name = "unknown_sensor";
    // @breif timestamp of current frame
    uint64_t timestamp_ns;
    PerceptionBlobFrame blob_frame;
    PerceptionSceneFrame scene_frame;
} PerceptionEnvODDFrame;

}  // namespace perception
}  // namespace senseAD
