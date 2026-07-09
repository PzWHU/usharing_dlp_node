/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

// #include "traffic_light_info.hpp"
#include <string.h>

#include "perception_camera/data_type/object/base_camera_attribute.h"
#include "perception_camera/data_type/object/base_object.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 交通灯信息
 */
typedef struct {
    /** 目标基本信息 */
    sapBaseObject base_info;
    /** 目标2D框 */
    sapCameraBBox2DInfo camera_bbox_info;
    /** 交通灯倒计时数字信息，默认值是-1表示不显示 */
    int16_t tl_num;
    /** 交通灯颜色分类，value是
        NONE = 0x00,
        GREEN = 0x02,
        YELLOW = 0x04,
        RED = 0x08,
        GREEN_FLASH = 0x10,
        YELLOW_FLASH = 0x20,
        BLACK = 0x40,
        GREEN_NUMBER = 0x80 */
    uint16_t color_label;
    /** 交通灯状态，value有：
        BLACK = 0,
        GREEN = 1,
        YELLOW = 2,
        RED = 3*/
    uint16_t status_label;
    /** 交通灯明亮状态下灯所表达出来的转弯形态/形状，
     * 对应senseAD::perception::GeneralTurnType */
    uint16_t type_label;  // GeneralTurnType::NONE
    /** 交通灯类别置信度 */
    float type_confidence;
    /** 交通灯关注度状态，value有：
        UNKNOWN = 0
        ATTENTION = 1
        NO_ATTENTION = 2
        NON_RELATED = 3*/
    uint16_t focus;
} sapTrafficLight;

/**
 * @brief 初始化交通灯结构体
 *
 * @param[in] traffic_light 交通灯信息结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapTrafficLight(sapTrafficLight *traffic_light);

#ifdef __cplusplus
}
#endif
