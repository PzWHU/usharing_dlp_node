/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <vector>
#include <string>
#include <utility>

#include "Eigen/Core"

#include "perception_common/data_type/object/fusion/shadow_mode_object_tag.hpp"
#include "perception_common/data_type/sensor_info.hpp"
#include "perception_common/data_type/frame/frame_header.hpp"

namespace senseAD {
namespace perception {

#ifdef WITH_SERIALIZATION_INTERFACE
typedef struct StructPerceptionObjectFrame : public FrameHeader {
#else
typedef struct StructPerceptionShadowModeFrame {
#endif  // WITH_SERIALIZATION_INTERFACE
    // frame id
    SensorID sensor_id = SensorID::UNKNOWN_SENSOR;
    // timestamp of current frame
    uint64_t timestamp_ns;

    std::vector<ShadowModeObjectTagPtr> objects_tag_list;

    int64_t num_tag = 0;

    inline size_t Size() const { return objects_tag_list.size(); }
    inline void Clear() {
        sensor_id = SensorID::UNKNOWN_SENSOR;
        timestamp_ns = 0;
        num_tag = 0;
        objects_tag_list.clear();
    }

#ifdef WITH_SERIALIZATION_INTERFACE
    FrameType GetFrameType() final {
        return FrameType::PERCEPTION_OBJECT_FRAME;
    }
#endif  // WITH_SERIALIZATION_INTERFACE
} PerceptionShadowModeFrame;

typedef std::shared_ptr<PerceptionShadowModeFrame> PerceptionShadowModeFramePtr;

}  // namespace perception
}  // namespace senseAD
