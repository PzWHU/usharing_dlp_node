/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 相机遮挡类型
 */
typedef enum {
    /** 未知状态 */
    BLOCK_AGE_UNKNOWN = 0,
    /** 清晰无遮挡状态 */
    BLOCK_AGE_CLEAR,
    /** 眩光状态 */
    BLOCK_AGE_GLARE,
    /** 全遮挡状态 */
    BLOCK_AGE_BLOCKED,
    /** 半遮挡状态 */
    BLOCK_AGE_PARTIAL_BLOCKED
} sapCameraBlockageType;

/**
 * @brief 炫光状态
 */
typedef enum {
    /** 未知状态 */
    GLARE_STATUS_UNKNOWN = 0,
    /** 清晰状态 */
    GLARE_STATUS_CLEAR,
    /** 炫光状态 */
    GLARE_STATUS_GLARED,
} sapGlareStatus;

/**
 * @brief 相机遮挡信息
 */
typedef struct _CameraFailsafeResult {
    /** 相机遮挡类型 */
    sapCameraBlockageType blockage_type;
    /** 相机炫光状态 */
    sapGlareStatus glare_status;
} CameraFailsafeResult;

/**
 * @brief 相机遮挡感知结果
 */
typedef struct _sapPerceptionFailsafeFrame {
    /** 图像帧号 */
    uint64_t frame_id;
    /** 传感器编号,必须 */
    uint16_t sensor_id;
    /** 传感器名称,必须，默认为unknown_sensor */
    char sensor_name[30];
    /** 当前帧时间戳 */
    uint64_t timestamp_ns;
    uint64_t sensor_send_timestamp_ns;
    /** 相机遮挡信息 */
    CameraFailsafeResult camera_failsafe_result;
} sapPerceptionFailsafeFrame;

#ifdef __cplusplus
}
#endif
