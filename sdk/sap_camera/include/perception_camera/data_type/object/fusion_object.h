/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Wang Zhaowei <wangzhaowei@senseauto.com>
 */
#pragma once

#include <stdint.h>
#include <string.h>

#include "base_object.h"

#ifdef __cplusplus
extern "C" {
#endif

struct sapAutomobileDoorStatus {
    int64_t bboxValidity = -2;
    int64_t leftDoorStatus = -2;
    int64_t rightDoorStatus = -2;
    int64_t rearDoorStatus = -2;
};

enum class sapAttrBool { DEFAULT = -2, UNKNOWN = -1, NEGATIVE, POSITIVE };
enum class sapBinaryDirection {
    UNKNOWN = -1,
    BINARY_DIRECTION_FRONT,
    BINARY_DIRECTION_BACK
};
enum class sapTrackStatus {
    INVALID = 0,
    NEW = 1,
    UPDATED = 2,
    COASTED = 3
};

typedef struct StructVehicleLightStatusInfo {
    bool is_valid = false;
    sapAttrBool left_turn = sapAttrBool::UNKNOWN;
    float left_turn_confidence = 0.0;
    sapAttrBool right_turn = sapAttrBool::UNKNOWN;
    float right_turn_confidence = 0.0;
    sapAttrBool hazard = sapAttrBool::UNKNOWN;
    float hazard_confidence = 0.0;
    sapAttrBool brake = sapAttrBool::UNKNOWN;
    float brake_confidence = 0.0;
    sapBinaryDirection binary_direction = sapBinaryDirection::UNKNOWN;
    float binary_direction_confidence = 0.0;
} sapVehicleLightStatusInfo;

struct sapMatchedSensorObjectInfo {
    uint16_t sensor_id;
    uint64_t frame_timestamp_ns;
    uint64_t object_id;
};

/**
 * @brief 目标基本信息
 */
typedef struct {
    sapBaseObject base_object;
    sapAutomobileDoorStatus door_status;
    sapVehicleLightStatusInfo vehicle_light_info;
    sapMatchedSensorObjectInfo *matched_sensor_object_info = NULL;
    uint64_t matched_sensor_object_info_num = 0;
    sapPerceptionLifeTimeHistory * associated_sensor_object_history_list = NULL;
    uint64_t associated_sensor_object_history_list_num = 0;
    sapTrackStatus track_status = sapTrackStatus::INVALID;
    /** 三方向上相对速度，融合特有 */
    float relative_velocity[3];
    /** 三方向上相对加速度，融合特有 */
    float relative_acceleration[3];

} sapFusionObject;

/**
 * @brief 初始化目标基本信息
 * @param[in,out] base_object 目标基本状态结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapFusionObject(sapFusionObject* fusion_object);
SAP_API int32_t ReleaseFusionObject(sapFusionObject* fusion_object);
#ifdef __cplusplus
}
#endif
