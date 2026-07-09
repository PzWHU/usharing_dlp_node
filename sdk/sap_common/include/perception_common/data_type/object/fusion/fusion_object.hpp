/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"

#include "perception_common/data_type/object/base_object.hpp"
#include "perception_common/data_type/object/camera/automobile.hpp"

namespace senseAD {
namespace perception {

typedef struct StructVehicleLightStatusInfo : ValidInfo {
    AutomobileLightStatus status;
} VehicleLightStatusInfo;

// @deprecated
// TODO(zhangshiquan): delete definition, instead, use associated sensor object
// lifetime history.
typedef struct StructMatchedSensorObjectInfo {
    StructMatchedSensorObjectInfo() = default;
    StructMatchedSensorObjectInfo(const SensorID& sensor_id_in,
                                  const uint64_t object_id_in,
                                  const uint64_t timestamp_ns_in)
        : sensor_id(sensor_id_in),
          frame_timestamp_ns(timestamp_ns_in),
          object_id(object_id_in) {}
    inline bool IsCompareSame(const StructMatchedSensorObjectInfo& other) {
        if (sensor_id != other.sensor_id) {
            return false;
        }
        if (frame_timestamp_ns != other.frame_timestamp_ns) {
            return false;
        }
        if (object_id != other.object_id) {
            return false;
        }
        return true;
    }
    SensorID sensor_id;
    uint64_t frame_timestamp_ns;
    uint64_t object_id;
} MatchedSensorObjectInfo;

enum class InterfaceTrackStatus {
    INVALID = 0,
    NEW = 1,
    UPDATED = 2,
    COASTED = 3
};

typedef struct StructFusionObject : public BaseObject {
    StructFusionObject() {}
    explicit StructFusionObject(BaseObject* object) : BaseObject(*object) {}
    StructFusionObject* GetThisPtr() override { return this; }
    // @brief vehicle light description, is valid, optional
    AutomobileDoorStatusINT door_status;
    VehicleLightStatusInfo vehicle_light_info;
    // @brief associated sensor object header info, optional
    std::vector<MatchedSensorObjectInfo> matched_sensor_object_info;

    std::vector<PerceptionLifeTimeHistory>
        associated_sensor_object_history_list;

    InterfaceTrackStatus track_status;
} FusionObject;

typedef std::shared_ptr<FusionObject> FusionObjectPtr;

}  // namespace perception
}  // namespace senseAD
