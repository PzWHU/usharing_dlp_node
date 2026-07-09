/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <vector>
#include <string>
#include <utility>

#include "Eigen/Core"

#include "perception_common/data_type/object/base_object.hpp"
#include "perception_common/data_type/sensor_info.hpp"
#include "perception_common/data_type/frame/frame_header.hpp"

namespace senseAD {
namespace perception {

// @brief general object frame
#ifdef WITH_SERIALIZATION_INTERFACE
typedef struct StructPerceptionObjectFrame : public FrameHeader {
#else
typedef struct StructPerceptionObjectFrame {
#endif  // WITH_SERIALIZATION_INTERFACE
    // frame id
    SensorID sensor_id = SensorID::UNKNOWN_SENSOR;
    // timestamp of current frame
    uint64_t timestamp_ns;
    FrameTimeCostInfo time_cost;
    // total object list of current frame
    std::vector<BaseObjectPtr> base_objects_list;
    inline size_t Size() const { return base_objects_list.size(); }
    inline void Clear() {
        sensor_id = SensorID::UNKNOWN_SENSOR;
        timestamp_ns = 0;
        base_objects_list.clear();
    }

    inline bool HasObjectsByLabel(
        const ObjectLabel& process_label_start,
        const ObjectLabel& process_label_end) const {
        for (const auto& obj_ptr : this->base_objects_list) {
            if (obj_ptr->label >= process_label_start &&
                obj_ptr->label <= process_label_end) {
                return true;
            }
        }
        return false;
    }

    inline std::vector<BaseObjectPtr> GetObjectsByLabel(
        const ObjectLabel& process_label_start,
        const ObjectLabel& process_label_end) const {
        std::vector<BaseObjectPtr> selected_object_list;
        for (const auto& obj_ptr : this->base_objects_list) {
            if (obj_ptr->label >= process_label_start &&
                obj_ptr->label <= process_label_end) {
                selected_object_list.push_back(obj_ptr);
            }
        }
        return selected_object_list;
    }

    inline std::vector<BaseObjectPtr> GetObjectsBySubtype(
        const ObjectLabel& process_label_start,
        const ObjectLabel& process_label_end) const {
        std::vector<BaseObjectPtr> selected_object_list;
        for (const auto& obj_ptr : this->base_objects_list) {
            if (obj_ptr->sub_type >= process_label_start &&
                obj_ptr->sub_type <= process_label_end) {
                selected_object_list.push_back(obj_ptr);
            }
        }
        return selected_object_list;
    }

#ifdef WITH_SERIALIZATION_INTERFACE
    FrameType GetFrameType() final {
        return FrameType::PERCEPTION_OBJECT_FRAME;
    }
#endif  // WITH_SERIALIZATION_INTERFACE
} PerceptionObjectFrame;

typedef std::shared_ptr<PerceptionObjectFrame> PerceptionObjectFramePtr;

}  // namespace perception
}  // namespace senseAD
