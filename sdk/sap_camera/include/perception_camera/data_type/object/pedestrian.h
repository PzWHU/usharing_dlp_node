/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

#include "perception_camera/sap_camera_define.h"
#include "perception_camera/data_type/object/base_object.h"
#include "perception_camera/data_type/object/base_camera_attribute.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 行人目标结构体
 *
 */
typedef struct {
    /** 目标基本信息 */
    sapBaseObject base_info;
    /** 目标2D框 */
    sapCameraBBox2DInfo camera_bbox_info;
    /** 目标运动信息 */
    sapBBox2DMotionInfo bbox_motion_info;
    /** 目标关键点 */
    sapBaseCameraAttribute attribute;
    /** 深度类型，value为 HOMOGRAPHY =
     * 0,NEAREST_FACE_WIDTH，WIDTH，LENGTH，HEIGHT，SKYLINE，MIXWH，MONO3D，PREDICT
     */
    int16_t depth_type;
} sapPedestrian;

/**
 * @brief 初始化行人目标结构体
 * @param[in] pPedestrian 行人目标信息结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapPedestrian(sapPedestrian *pPedestrian);

#ifdef __cplusplus
}
#endif
