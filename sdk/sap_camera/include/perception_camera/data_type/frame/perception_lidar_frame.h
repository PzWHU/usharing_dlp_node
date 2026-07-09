/*
 * Copyright (C) 2024 by SenseTime Group Limited. All rights reserved.
 * Qi Chenyu <qichenyu1@senseauto.com>
 */
#pragma once

#include <string.h>
#include <stdlib.h>

#include "perception_camera/sap_camera_define.h"
#include "perception_camera/data_type/object/lidar_object.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _sapPerceptionLidarFrame {
    /** 帧号 */
    uint64_t frame_id;
    /** 传感器编号 */
    uint16_t sensor_id;  // SensorID::UNKNOWN_SENSOR
    /** 当前帧时间戳 */
    uint64_t timestamp_ns;
    uint64_t sensor_send_timestamp_ns;
    /** lidar目标数目 */
    uint16_t lidar_gop_num;
    /** lidar目标列表 */
    sapLidarObstacle* lidar_object;
    /** decode后的lidar目标数目 */
    uint16_t decoded_lidar_gop_num;
    /** decode后的lidar目标列表 */
    sapLidarObstacle* decoded_lidar_object;
} sapPerceptionLidarGopFrame;

/**
 * @brief 初始化lidar目标结果结构体
 * @param[in,out] frame lidar目标结果结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t InitsapPerceptionLidarGopFrame(sapPerceptionLidarGopFrame* frame);

SAP_API void ReleaseLidarGopFrameMemory(sapPerceptionLidarGopFrame* frame);

#ifdef __cplusplus
}
#endif
