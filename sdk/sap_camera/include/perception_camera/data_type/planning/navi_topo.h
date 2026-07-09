/*
 * Copyright (C) 2025 by SenseTime Group Limited. All rights reserved.
 * Xuetao Cheng <chengxuetao@senseauto.com>
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include "perception_camera/data_type/base_type.h"
#include "perception_camera/data_type/planning/local_map.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief NaviTopo
 */
 
typedef struct {
    uint64_t timestamp_ns;
    uint64_t cur_status;
    sapLaneData* lanes;
    uint64_t lanes_num;
} sapNaviTopo;

#ifdef __cplusplus
}
#endif