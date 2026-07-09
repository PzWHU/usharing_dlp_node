/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

#include <string.h>
#include <stdlib.h>

#include "perception_camera/data_type/object/base_object.h"
#include "perception_camera/data_type/object/static_obstacle.h"
#include "perception_camera/sap_camera_define.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 静态障碍物信息
 */
typedef struct {
    /** 图像帧号 */
    uint64_t frame_id;
    /** 传感器编号 */
    uint16_t sensor_id;
    /** 当前帧时间戳 */
    uint64_t timestamp_ns;
    uint64_t sensor_send_timestamp_ns;
    /** 静态障碍物集合 */
    sapStaticObstacle* static_obstacle;
    /** 静态障碍物数量 */
    uint16_t gop_num;
    /** 最大的静态障碍物数量，用于分配内存 */
    uint16_t max_gop_num;
} sapPerceptionGopFrame;

/**
 * @brief 初始化静态障碍物信息结构体
 * @param[in,out] frame 静态障碍物结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapPerceptionGopFrame(sapPerceptionGopFrame* frame);

/**
 * @brief 释放静态障碍物结构体内部指针指向的内存
 * @param[in,out] frame 静态障碍物结构体指针
 */
SAP_API void ReleaseGopFrameMemory(sapPerceptionGopFrame* frame);

#ifdef __cplusplus
}
#endif
