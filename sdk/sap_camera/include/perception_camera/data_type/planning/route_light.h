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
 * @brief RouteLight
 */
typedef struct {
    //red=3 green=5 yellow=7
    uint16_t lightType;
    //None=-1 Straight=0 Left=1 Right=2 LeftTurn=3 RightTurn=4
    uint16_t turnArrowType;
    uint64_t remainTime;
    uint64_t totalTime;
} sapOneLight;

typedef struct {
    sapRoutePos point;
    sapOneLight* onelightinfo;
    uint64_t onelightinfo_num;
} sapLightInfo;

typedef struct {
    uint64_t timestamp;
    sapLightInfo* lightsinfo;
    uint64_t lightsinfo_num;
} sapRouteLight;

#ifdef __cplusplus
}
#endif
