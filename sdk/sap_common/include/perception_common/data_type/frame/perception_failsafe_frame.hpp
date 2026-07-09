/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Li Hongwei <lihongwei1@senseauto.com>
 */
#pragma once

#include <vector>
#include <memory>
#include <string>
#include <map>

namespace senseAD {
namespace perception {

enum class CameraBlockageType {
    UNKNOWN = 0,
    CLEAR,
    GLARED,
    BLOCKED,  // sensor blindness
    PARTIAL_BLOCKED
};

enum class GlareStatus {
    UNKNOWN = 0,
    CLEAR,
    GLARED,
};

static const std::map<CameraBlockageType, std::string>
    blockagetypetostring{
    {CameraBlockageType::UNKNOWN, "unknown"},
    {CameraBlockageType::CLEAR, "clear"},
    {CameraBlockageType::GLARED, "glared"},
    {CameraBlockageType::BLOCKED, "blocked"},
    {CameraBlockageType::PARTIAL_BLOCKED, "partial_blocked"}
};
static const std::map<GlareStatus, std::string>
    glarestatustostring{
    {GlareStatus::UNKNOWN, "unknown"},
    {GlareStatus::CLEAR, "clear"},
    {GlareStatus::GLARED, "glared"}
};

typedef struct StructCameraFailsafeResult {
    CameraBlockageType blockage_type = CameraBlockageType::BLOCKED;
    GlareStatus glare_status = GlareStatus::UNKNOWN;
} CameraFailsafeResult;

typedef struct StructPerceptionFailsafeFrame {
    // @brief sensor id, required
    SensorID sensor_id = SensorID::UNKNOWN_SENSOR;
    // @brief sensor name, required
    std::string sensor_name = "unknown_sensor";
    // @brief timestamp of current frame
    uint64_t timestamp_ns;

    CameraFailsafeResult camera_failsafe_result;
} PerceptionFailsafeFrame;

}  // namespace perception
}  // namespace senseAD
