/*
 * Copyright (C) 2025 by SenseTime Group Limited. All rights reserved.
 * Qi Chenyu <qichenyu1@senseauto.com>
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include "perception_camera/data_type/planning/planning_info.h"
#include "perception_camera/sap_camera_define.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 规控信息
 */
typedef struct _sapPerceptionPlanningFrame {
    /** 帧号 */
    uint64_t frame_id;
    /** 传感器编号 */
    uint16_t sensor_id;  // SensorID::UNKNOWN_SENSOR
    /** 当前帧时间戳 */
    uint64_t timestamp_ns;
    uint16_t gear;
    uint16_t turn_signal;
    uint16_t traj_state;
    uint16_t traj_mode;
    sapPlanningInstruction instruction;
    sapTrajectoryPoint* traj_point_array;
    uint64_t traj_point_array_num;
    sapPredictionObjectArray traj_object_array;
} sapPerceptionPlanningFrame;

/**
 * @brief 初始化单帧planning信息结构体
 * @param[in] frame planning信息结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapPerceptionPlanningFrame(sapPerceptionPlanningFrame* frame);

/**
 * @brief 释放sapPerceptionPlanningFrame内部内存
 * @param[in] frame planning信息结构体指针
 * @return
 */
SAP_API void ReleasePlanningFrameMemory(sapPerceptionPlanningFrame* frame);

/**
 * @brief 规控调试信息
 */
typedef struct {
    char *string;
    uint64_t length;
} sapPerceptionPlanningDebugInfo;

/**
 * @brief 初始化planning调试信息结构体
 * @param[in] info planning调试信息结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapPerceptionPlanningDebugInfo(sapPerceptionPlanningDebugInfo* info);

/**
 * @brief 释放sapPerceptionPlanningDebugInfo内部内存
 * @param[in] info planning调试信息结构体指针
 * @return
 */
SAP_API void ReleasePlanningDebugInfoMemory(sapPerceptionPlanningDebugInfo* info);

#ifdef __cplusplus
}
#endif
