/*
 * Copyright (C) 2025 by SenseTime Group Limited. All rights reserved.
 * Qi Chenyu <qichenyu1@senseauto.com>
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include "perception_camera/data_type/planning/route_base_info.h"
#include "perception_camera/data_type/planning/route_lane_info.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief RouteGuidance
 */
typedef struct {
    uint64_t timestamp;
    // 机动点经纬度
    sapRoutePos intersectionPos;
    bool matchResult;
    // 10-15 左传，18 分流处靠左，
    // 20-25 右转， 26-28 靠右
    uint64_t nextIntersectionType;
    // 路口剩余距离,m
    uint64_t newRemainDistance;
    char pointId[30];
    // old
    uint64_t nextIntersectionRemainDistance;
    // 全路段剩余距离
    uint64_t totalRemainDistance;
    // 当前导航段距离
    uint64_t segmentLength;
    // 平行路状态
    uint64_t hintType;
    sapRoutePos enterPosA;
    sapRoutePos leavePosB;
    // 机动点路口内长度
    uint64_t actionLenth;
    // 当前导航段隧道内长度
    uint64_t tunnelLength;
    // 随后 xx 米 右转，是否接近分流转向点
    bool hasCloseTurn;
    // 分流转向点类型（同上）
    uint64_t closeIntersectionType;
    // 类似路口剩余距离
    uint64_t closeIntersectionLength;
    // 随后点的经纬度
    sapRoutePos closeIntersectionPos;
    uint64_t nextSegmentLength;
    uint64_t limitSpeedKmph;
    // 机动点处的推荐车道
    sapRouteLaneInfo intersectionLaneInfo;
} sapRouteGuidance;

#ifdef __cplusplus
}
#endif
