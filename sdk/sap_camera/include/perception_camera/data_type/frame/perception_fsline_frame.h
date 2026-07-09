/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Xingxi <xingxi@senseauto.com>
 */

#pragma once

#include "perception_camera/sap_camera_define.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 可行驶区域线类型
 */
enum class sapFreespaceLineLabel {
    /** 车辆 */
    VEHICLE = 0,
    /** 行人 */
    PEDESTRIAN,
    /** 非机动车 */
    BIKE,
    /** 路沿 */
    CURB,
    /** 墙体 */
    WALL,
    /** 立柱 */
    COLUMN,
    /** 锥桶 */
    CONE,
    /** 车轮停止线 */
    WHEELSTOP,
    /** 其他 */
    OTHER,
};

/**
 * @brief 可行驶区域线
 */
typedef struct _sapFreespaceLine {
    /** 可行驶区域线x像素坐标 */
    uint32_t x;
    /** 可行驶区域线y像素坐标 */
    uint32_t y;
    /** 可行驶区域线类型 */
    sapFreespaceLineLabel label;
    /** 可行驶区域线置信度 */
    float confidence;
    /** 可行驶区域线x世界坐标 */
    float world_x;
    /** 可行驶区域线y世界坐标 */
    float world_y;
    /** 该点是否可用*/
    bool isvalid;
} sapFreespaceLine;

/**
 * @brief 可行驶区域，由若干可行驶区域线组成
 */
typedef struct _sapFreespaceLineResult {
    uint64_t freespace_line_num;
    sapFreespaceLine* freespace_lines;
} sapFreespaceLineResult;

/**
 * @brief 初始化可行驶区域结构体
 * @param[in] freespace 可行驶区域结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapFreespaceLineResult(sapFreespaceLineResult* freespace);

/**
 * @brief 可行驶区域信息
 */
typedef struct _sapFreespaceLineStruct {
    /** 可行驶区域数量 */
    uint16_t freespace_num;
    /** 可行驶区域集合 */
    sapFreespaceLineResult* freespace_results;
} sapFreespaceLineStruct;

#define NAME_STR_MAX_LEN 30

/**
 * @brief 单帧可行驶区域感知结果
 */
typedef struct _sapPerceptionFSLineFrame {
    /** 帧号 */
    uint64_t frame_id;
    /** 传感器编号 */
    uint16_t sensor_id;
    /** 传感器名称 */
    char sensor_name[NAME_STR_MAX_LEN];
    /** 当前帧时间戳 */
    uint64_t timestamp_ns;
    uint64_t sensor_send_timestamp_ns;
    /** 可行驶区域信息 */
    sapFreespaceLineStruct fsline_struct;
} sapPerceptionFSLineFrame;

/**
 * @brief 初始化可行驶区域结构体
 * @param[in,out] frame 可行驶区域结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapPerceptionFSLineFrame(sapPerceptionFSLineFrame* frame);

/**
 * @brief 释放
 * @param[in,out] frame 可行驶区域结构体指针
 * @return 成功返回0，否则返回错误码
 * @note
 * 此方法只释放结构体内部指针指向的内存，并不释放sapPerceptionFSLineFrame结构体本身的内存
 */
SAP_API void ReleaseFSLineFrameMemory(sapPerceptionFSLineFrame* frame);

#ifdef __cplusplus
}
#endif
