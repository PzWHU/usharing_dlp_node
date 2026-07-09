/*
 * Copyright (C) 2025 by SenseTime Group Limited. All rights reserved.
 * Qi Chenyu <qichenyu1@senseauto.com>
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include "perception_camera/data_type/planning/route_base_info.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief RouteLaneInfo
 */
typedef struct {
    // a: 直行，b:右转， c: 左调头
    // d: 左转，e:右调头
    char laneFlag[30];
    /*
        0:直行 1:左转 2:直行+左转
        3: 右转 4:直行+右转
        5: 左掉头 6: 左转+右转 7: 直行+左转+右转
        8: 右掉头 9: 直行+左掉头 A: 直行+右掉头
        B: 左转+左掉头 C:右转+右掉头 E: 右转+左掉头
        F: 直行+左转+左掉头 P:左转+右掉头 K: 左转+直行+右掉头
        I: 直行+右转+右掉头 G: 左掉头+右掉头 O: 左掉头+左转+右掉头
        J: 左掉头+直行+右掉头M: 左掉头+右转+右掉头
        L: 左转+右转+左掉头 N:右转+左转+右掉头 H: 直行+右转+左掉头
    */
    char laneArrow[30];
    // 车道类型String :
    // 0:普通车道
    // 1:公交车道
    // 2:无时段公交车道
    // 5:HOV车道
    // c:潮汐车道
    // f:空车道
    // g:可变车道
    // z:其他(预留)
    char laneProperty[30];
} sapRouteLaneData;

typedef struct {
    uint64_t timestamp;// 车道线位置信息
    sapRoutePos position;
    // tips 到路线终点的距离
    uint64_t distanceToRouteEnd;
    sapRouteLaneData* lanesData;
    uint64_t lanesData_num;
    char recommendLanes[30];
    // 在车道级定位方案中，该接口表示根据定位提供前方车道信息
    uint64_t laneNumLeft;
    // 在车道级定位方案中，该接口表示根据定位提供前方车道信息
    uint64_t laneNumRight;
} sapRouteLaneInfo;

#ifdef __cplusplus
}
#endif
