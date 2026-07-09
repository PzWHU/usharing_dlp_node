/**
 * @file perception_road_semantics.h
 * @author liuguangxiong (liuguangxiong@senseauto.com)
 * @brief 路面语义信息结构体定义
 * @version 0.1
 * @date 2023-08-16
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <stdlib.h>

#include "perception_camera/sap_camera_define.h"
#include "perception_camera/data_type/object/traffic_light.h"
#include "perception_camera/data_type/object/traffic_sign.h"
#include "perception_camera/data_type/object/road_marker_object.h"
#include "perception_camera/data_type/object/board_pole.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 路面语义信息
 */
typedef struct _sapRoadSemantics {
    /** 交通灯数量 */
    uint32_t traffic_light_num;
    /** 最大的交通灯数量，用于分配内存 */
    uint32_t max_traffic_light_num;
    /** 交通灯集合 */
    sapTrafficLight* traffic_light_results;
    /** 交通标志数量 */
    uint32_t traffic_sign_num;
    /** 最大交通标志数量，用于分配内存 */
    uint32_t max_traffic_sign_num;
    /** 交通标志集合 */
    sapTrafficSign* traffic_sign_results;
    /** 道路标记数量 */
    uint32_t road_marker_num;
    /** 最大道路标记数量，用于分配内存 */
    uint32_t max_road_marker_num;
    /** 道路标记结果集合 */
    sapRoadMarker* road_marker_results;
    /** 街道指示牌和灯杆数量 */
    uint32_t board_pole_num;
    /** 最大街道指示牌和灯杆数量，用于分配内存 */
    uint32_t max_board_pole_num;
    /** 街道指示牌和灯杆结果集合 */
    sapBoardPole* board_pole_results;
} sapRoadSemantics;

/**
 * @brief 单帧路面语义信息结果
 */
typedef struct _sapPerceptionRoadSemanticsFrame {
    /** 帧编号 */
    uint64_t frame_id;
    /** 传感器编号 */
    uint16_t sensor_id;  // SensorID::UNKNOWN_SENSOR
    /** 传感器名称 */
    char sensor_name[30];
    /** 当前帧的时间戳 */
    uint64_t timestamp_ns;
    uint64_t sensor_send_timestamp_ns;
    /** 语义结果信息 */
    sapRoadSemantics road_semantics;
} sapPerceptionRoadSemanticsFrame;

/**
 * @brief 初始化路面语义结果结构体
 * @param[in,out] frame 路面语义结果结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t
InitsapPerceptionRoadSemanticsFrame(sapPerceptionRoadSemanticsFrame* frame);

/**
 * @brief 释放sapPerceptionRoadSemanticsFrame内部内存
 * @param[in] frame 单帧路面语义结果结构体指针
 * @return 无
 * @note
 * 此方法只释放结构体内部指针指向的内存，并不释放sapPerceptionRoadSemanticsFrame结构体本身的内存
 */
SAP_API void ReleaseRoadSemanticsFrame(sapPerceptionRoadSemanticsFrame* frame);

#ifdef __cplusplus
}
#endif
