/*
 * Copyright (C) 2024 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include "perception_camera/sap_camera_define.h"
#include "perception_camera/data_type/object/base_data_struct.h"
#include "perception_camera/data_type/road/occupancy_info.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 占用信息
 */
typedef struct _sapOccStruct {
    sapOccupancyInfoResult occupancy_info_results;
} sapOccStruct;

typedef struct _sapVDOccStruct {
    sapVDOccInfoResult vd_occ_info_results;
} sapVDOccStruct;

#define NAME_STR_MAX_LEN 30

/**
 * @brief 单帧占用信息感知结果
 */
typedef struct _sapPerceptionOccFrame {
    /** 帧号 */
    uint64_t frame_id;
    /** 传感器编号 */
    uint16_t sensor_id;  // SensorID::UNKNOWN_SENSOR
    /** 传感器名称 */
    char sensor_name[NAME_STR_MAX_LEN];
    /** 当前帧时间戳 */
    uint64_t timestamp_ns;
    uint64_t sensor_send_timestamp_ns;
    /** 占用信息 */
    sapOccStruct occ_struct;
    bool use_vd_occ;
    sapVDOccStruct vd_occ_struct;
} sapPerceptionOccFrame;

/**
 * @brief 初始化占用信息结构体
 * @param[in,out] frame 占用信息结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapPerceptionOccFrame(sapPerceptionOccFrame* frame);

/**
 * @brief 释放
 * @param[in,out] frame 占用信息结构体指针
 * @return 成功返回0，否则返回错误码
 * @note
 * 此方法只释放结构体内部指针指向的内存，并不释放sapPerceptionOccFrame结构体本身的内存
 */
SAP_API void ReleaseOccFrameMemory(sapPerceptionOccFrame* frame);

#ifdef __cplusplus
}
#endif
