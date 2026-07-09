/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

#include "perception_camera/data_type/object/base_object.h"
#include "perception_camera/data_type/object/base_camera_attribute.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 非机动车目标信息
 */
typedef struct {
    /** 目标基本信息 */
    sapBaseObject base_info;
    /** 目标2D框 */
    sapCameraBBox2DInfo camera_bbox_info;
    /** 目标运动信息 */
    sapBBox2DMotionInfo bbox_motion_info;
    /** 目标关键点信息 */
    sapBaseCameraAttribute attribute;
} sapNonAutomobileVehicle;

/**
 * @brief 初始化sapNonAutomobileVehicle结构体
 * 
 * @param[in] bike 非机动车目标信息结构体指针
 * @return 成功返回0，否则返回错误码 
 */
SAP_API int32_t InitsapNonAutomobileVehicle(sapNonAutomobileVehicle* bike);

#ifdef __cplusplus
}
#endif
