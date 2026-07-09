/*
 * Copyright (C) 2025 by SenseTime Group Limited. All rights reserved.
 * Qi Chenyu <qichenyu1@senseauto.com>
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include "perception_camera/data_type/base_type.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CrossInfo
 */
typedef struct {
    float key;
    float value;
} sapPairPoint;

typedef struct {
    sapPairPoint* pts;
    uint64_t pts_num;
    uint64_t* laneids;
    uint64_t laneids_num;
} sapVDStopLine;

typedef struct {
    uint64_t timestamp_ns;
    bool lights_flag;
    bool red_light = false;
    sapPoint2f light_center;
    sapVDStopLine* stoplines;
    uint32_t stoplines_num;
    uint32_t ego_line_id;
    sapPairPoint* leftpts;
    uint64_t leftpts_num;
    sapPairPoint* rightpts;
    uint64_t rightpts_num;
    uint32_t stop_x;
    /**
    Unknown = 0, Invalid = 1, Off = 2,
    Green = 3, Yellow = 4, Red = 5,*/
    uint32_t ego_lane_traffic_light;
    uint64_t* lines_idx;
    uint64_t lines_idx_num;
    sapPairPoint** lines_pts;
    uint32_t lines_num;
    uint32_t* lines_pts_num;
    sapPairPoint** lanecenter_pts;
    uint32_t lanecenter_num;
    uint32_t* lanecenter_pts_num;
    // kph
    float route_speed_limit = 30.0f;

    // the lane_type IS turn_type
    // Unknown = 0x0000,          // 0x0000 未知类型
    // TurnLeft = 0x0001,         // 0x0001 左转
    // TurnRight = 0x0002,        // 0x0002 右转
    // TurnLeftAround = 0x0004,   // 0x0004 左掉头
    // TurnRightAround = 0x0008,  // 0x0008 右掉头
    // Straight = 0x0010,         // 0x0010 直行
    // TurnLeftArea = 0x0020,     // 0x0020 左转待转区
    // Other = 0xffff,
    uint32_t* lane_type;
    uint32_t lane_type_num;
    uint32_t** lane_turn_type; // merge之后中心线的转向集合
    uint32_t lane_turn_num;
    uint32_t* lane_turn_type_num;
    uint32_t* next_lane_type;
    uint32_t next_lane_type_num;
} sapCrossInfo;

#ifdef __cplusplus
}
#endif