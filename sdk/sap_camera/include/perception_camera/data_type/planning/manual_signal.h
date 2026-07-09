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
 * @brief ManualSignal
 */
typedef struct {
    uint64_t timestamp;
    // 1->left, 2->right
    uint16_t TurnSignal;
    float DLPEgolight;
    // need a multiplier
    float ExpectedSpeed;
    float front_wheel_angle;
} sapManualSignal;

#ifdef __cplusplus
}
#endif
