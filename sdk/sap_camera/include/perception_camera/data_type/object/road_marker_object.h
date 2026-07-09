/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

#include <string.h>

#include "perception_camera/sap_camera_define.h"
#include "perception_camera/data_type/object/base_camera_attribute.h"
#include "perception_camera/data_type/object/base_object.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 地面道路标记
 */
typedef struct {
    /** 基本信息 */
    sapBaseObject base_info;
    /** 目标2D框信息 */
    sapCameraBBox2DInfo camera_bbox_info;
    /** 类型置信度 */
    float type_confidence;
} sapRoadMarker;

/**
 * @brief 初始化sapRoadMarker结构体
 * @param[in,out] roadmarker 地面道路标记信息结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapRoadMarker(sapRoadMarker* roadmarker);

#ifdef __cplusplus
}
#endif
