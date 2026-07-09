/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@sensetime.com>
 */
#pragma once

#include <vector>
#include <memory>
#include <string>

#include "perception_common/base/data_type/base.hpp"
#include "perception_common/data_type/sensor_info.hpp"

namespace senseAD {
namespace perception {

typedef struct StructPerceptionOccInfoFrame {
    // @brief sensor id, required
    SensorID sensor_id = SensorID::UNKNOWN_SENSOR;
    // @brief timestamp of current frame
    uint64_t timestamp_ns;
    std::vector<uint8_t> occ_data;
    std::vector<size_t> occ_shape;
    std::vector<float32_t> range_3d;
    Eigen::Vector3f voxel_size;
    Eigen::Vector3i grid_size;
    inline void Clear() {
        sensor_id = SensorID::UNKNOWN_SENSOR;
        timestamp_ns = 0;
        occ_data.clear();
        occ_shape.clear();
        range_3d.clear();
    }
} PerceptionOccInfoFrame;

typedef std::shared_ptr<PerceptionOccInfoFrame> PerceptionOccInfoFramePtr;

}  // namespace perception
}  // namespace senseAD
