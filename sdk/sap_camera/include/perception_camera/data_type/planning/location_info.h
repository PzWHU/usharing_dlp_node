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
 * @brief LocationInfo
 */
typedef struct {
    uint64_t timestamp;
    double longitude;
    double latitude;
    double altitude;
    double position_x;
    double position_y;
    double position_z;
    double roll;
    double pitch;
    double yaw;
    double lon_acc;
    double lat_acc;
} sapLocationInfo;

#ifdef __cplusplus
}
#endif
