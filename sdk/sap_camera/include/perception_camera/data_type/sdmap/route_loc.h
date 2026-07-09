/*
 * Copyright (C) 2024 by SenseTime Group Limited. All rights reserved.
 * Wang Zhaowei <wangzhaowei@senseauto.com>
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include "perception_camera/data_type/base_type.h"

/** 导航路径 */
struct sapNavRouteInfo{
    /** 导航路径所对应的所有link raw id */
    uint64_t* link_raw_ids;
    uint64_t link_raw_ids_num;
    /** 导航路径所对应的坐标列表 */
    sapPointLLH* points_llh;
    uint64_t points_llh_num;
    sapPoint3f* local_pts;
    uint64_t local_pts_num;
};

/** 定位信息 */
struct sapLocalizationInfo {
    /** 定位时间戳 */
    uint64_t ts = 0;
    /** 定位坐标 */
    sapPointLLH point_llh;
};