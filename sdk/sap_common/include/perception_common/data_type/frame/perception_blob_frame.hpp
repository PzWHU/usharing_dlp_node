/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Xu Zhipeng <xuzhipeng@senseauto.com>
 */
#pragma once

#include <memory>
#include <vector>
#include <string>
#include <map>

#include "Eigen/Core"
#include "perception_common/base/data_type/bbox_2d.hpp"

namespace senseAD {
namespace perception {

enum class PerceptionBlobType {
    UNKNOWN = 0,
    HEADLIGHT = 1,
    TAILLIGHT = 2,
    STREETLAMP = 3,
};

enum class PerceptionBlobStatus {
    UNKNOWN = 0,
    STATIC = 1,
    DYNAMIC = 2,
};

enum class PerceptionBlobBrightnessState {
    UNKNOWN = 0,
    BRIGHT,
    DARK,
};

static const std::map<PerceptionBlobType, std::string>
    blobtypetostring{
    {PerceptionBlobType::UNKNOWN, "unknown"},
    {PerceptionBlobType::HEADLIGHT, "headlight"},
    {PerceptionBlobType::TAILLIGHT, "taillight"},
    {PerceptionBlobType::STREETLAMP, "streetlamp"}
};
static const std::map<PerceptionBlobStatus, std::string>
    blobstatustostring{
    {PerceptionBlobStatus::UNKNOWN, "unknown"},
    {PerceptionBlobStatus::STATIC, "static"},
    {PerceptionBlobStatus::DYNAMIC, "dynamic"}
};

typedef struct StructBlobObjectAttributes {
    PerceptionBlobType blob_type = PerceptionBlobType::UNKNOWN;
    float type_confidence = 1.0f;
    PerceptionBlobStatus blob_status = PerceptionBlobStatus::UNKNOWN;
    uint8_t probability_oncoming = 0;
    uint8_t probability_proceeding = 0;
} BlobObjectAttributes;

typedef struct StructBlobObjectKinematicWorld {
    Bbox2D det_bbox_2d;
    float eul_distance;
    // @brief center of the boundingbox (cx, cy, cz), meter unit, required
    Eigen::Vector3f center_pos = Eigen::Vector3f(0, 0, 0);
    // @brief camera angle info about blob bounding box, required
    Eigen::Vector4f angle_info = Eigen::Vector4f(0, 0, 0, 0);
} BlobObjectKinematicWorld;

typedef struct StructBlobObjectGeneral {
    uint64_t objectID;
    uint64_t objectIDPaired;
    uint16_t object_cycles_traced = 0;
    uint16_t intensity;
    float luminousFlux;
} BlobObjectGeneral;

typedef struct StructBlobObject {
    BlobObjectAttributes blob_object_attributes;
    BlobObjectKinematicWorld blob_object_kinematic_world;
    BlobObjectGeneral blob_object_general;
} BlobObject;

typedef struct StructBlobStruct {
    std::vector<BlobObject> blob_info_results;
    PerceptionBlobBrightnessState blob_brightness_state;
} BlobStruct;

typedef struct StructPerceptionBlobFrame {
    // @brief sensor id, required
    SensorID sensor_id = SensorID::UNKNOWN_SENSOR;
    // @brief sensor name, required
    std::string sensor_name = "unknown_sensor";
    // @breif timestamp of current frame
    uint64_t timestamp_ns;
    BlobStruct blob_struct;

    inline void Clear() {
        sensor_id = SensorID::UNKNOWN_SENSOR;
        timestamp_ns = 0;
        blob_struct.blob_brightness_state =
            PerceptionBlobBrightnessState::UNKNOWN;
        blob_struct.blob_info_results.clear();
    }
} PerceptionBlobFrame;

}  // namespace perception
}  // namespace senseAD
