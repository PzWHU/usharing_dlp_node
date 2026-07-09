/*
 * Copyright (C) 2025 by SenseTime Group Limited. All rights reserved.
 * Wang Zhaowei <wangzhaowei@senseauto.com>
 */
#pragma once

#include <string.h>
#include "perception_camera/sap_macros.h"
#include "perception_camera/data_type/object/base_object.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 输入的radar帧
 *
 */
typedef struct {
    /** 图像时间戳 */
    uint64_t timestamp_ns;
    uint64_t sensor_send_timestamp_ns;
    uint64_t perception_capture_timestamp_ns;
    /** 帧号，自定义，无绝对意义，一般是顺序号 */
    int64_t frame_no;
    /** sensor编号 */
    int8_t sensor_id;
    /** sensor名称 */
    char sensor_name[MAX_NAME];
    /** radar 目标数量*/
    uint64_t radar_objects_num;
    /** radar 目标 */
    sapBaseObject* radar_objects_list;
} sapRadarFrame;

/**
 * @brief 初始化sapRadarFrame结构体数据
 * @param[in] frame sapRadarFrame类型结构体指针
 * @return 成功返回0, 否则返回错误码
 */
SAP_API int32_t initSapRadarFrame(sapRadarFrame* frame);

#ifdef __cplusplus
}
#endif
