/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

#include <string.h>
#include <stdlib.h>

#include "perception_camera/sap_camera_define.h"
#include "perception_camera/data_type/object/base_object.h"
#include "perception_camera/data_type/object/automobile.h"
#include "perception_camera/data_type/object/non_automobile_vehicle.h"
#include "perception_camera/data_type/object/pedestrian.h"
#include "perception_camera/data_type/object/static_obstacle.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 单帧路面目标信息，人、车、非机动车
 */
typedef struct _sapPerceptionObjectFrame {
    /** 帧号 */
    uint64_t frame_id;
    /** 传感器编号 */
    uint16_t sensor_id;  // SensorID::UNKNOWN_SENSOR
    /** 当前帧时间戳 */
    uint64_t timestamp_ns;
    uint64_t sensor_send_timestamp_ns;
    /** 机动车目标列表 */
    sapAutomobile* vehicle;
    /** 机动车数目 */
    uint16_t veh_num;
    /** 非机动车列表 */
    sapNonAutomobileVehicle* non_vehicle;
    /** 非机动车数量 */
    uint16_t non_veh_num;
    /** 行人列表 */
    sapPedestrian* pedestrian;
    /** 行人数量 */
    uint16_t ped_num;
} sapPerceptionObjectFrame;

/**
 * @brief 初始化单帧目标信息结构体
 * @param[in] frame 目标信息结构体指针
 * @return 成功返回0，否则返回错误码 
 */
SAP_API int32_t InitsapPerceptionObjectFrame(sapPerceptionObjectFrame* frame);

/**
 * @brief 释放sapPerceptionObjectFrame内部内存
 * @param[in] frame 目标信息结构体指针
 * @return 
 */
SAP_API void ReleaseObjectFrameMemory(sapPerceptionObjectFrame* frame);

#ifdef __cplusplus
}
#endif
