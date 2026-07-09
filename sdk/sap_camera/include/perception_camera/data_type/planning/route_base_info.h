/*
 * Copyright (C) 2025 by SenseTime Group Limited. All rights reserved.
 * Qi Chenyu <qichenyu1@senseauto.com>
 */

#pragma once

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief RoutePos
 */
typedef struct {
    uint64_t coorStart;
    double lon;
    double lat;
    double height;
    double x;
    double y;
} sapRoutePos;

#ifdef __cplusplus
}
#endif
