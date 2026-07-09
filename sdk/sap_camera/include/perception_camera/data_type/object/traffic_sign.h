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
 * @brief 交通标志信息
 */
typedef struct {
    /** 交通标志基本信息 */
    sapBaseObject base_info;
    /** 交通标志2D框 */
    sapCameraBBox2DInfo camera_bbox_info;
    /** 交通标志类别置信度 */
    float type_confidence;
} sapTrafficSign;

/**
 * @brief 初始化交通标志信息
 * 
 * @param[in,out] traffic_sign 交通标志信息结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapTrafficSign(sapTrafficSign* traffic_sign);

#ifdef __cplusplus
}
#endif
