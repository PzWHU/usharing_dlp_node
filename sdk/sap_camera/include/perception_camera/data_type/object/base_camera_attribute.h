/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

#include <stdio.h>

#include "perception_camera/sap_camera_define.h"
#include "perception_camera/data_type/object/base_data_struct.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 相机目标包围框信息
 */
typedef struct {
    /** 原始检测框 */
    sapBbox2D raw_detection_box;
    /** 目标跟踪框 */
    sapBbox2D tracked_box;
} sapCameraBBox2DInfo;

/** 2D目标框运动信息 */
typedef struct {
    /** 单位时间内目标的width在帧之间变化的像素个数 */
    float width_velocity;
    /** 单位时间内目标的height在帧之间变化的像素个数 */
    float height_velocity;
    /** 单位时间内目标的对角线在帧之间变化的像素个数 */
    float width_height_ratio_velocity;
    /** 底边中心点x方向的变化像素个数 */
    float bottom_center_x_velocity;
    /** 底边中心点y方向的变化像素个数 */
    float bottom_center_y_velocity;
} sapBBox2DMotionInfo;

/**
 * @brief 2D pvb使用，检测关键点等属性
 */
typedef struct {
    /** 关键点是否有效 */
    int8_t is_landmark_valid;
    /** 关键点数组 */
    sapVector2f* landmark;
    /** 关键点个数 */
    uint64_t landmark_num;
    /** 关键点得分 */
    int8_t* landmark_scores;
    /** 关键点分数数量 */
    uint64_t score_num;
    /** 关键点viewport方向有校性 */
    int8_t is_direction_valid;
    /** viewport方向 */
    sapVector2f viewport_direction;
} sapBaseCameraAttribute;

/**
 * @brief 行人意图类型
 */
typedef enum {
    /** 积极的 */
    INTENTION_POSITIVE,
    /** 消极的 */
    INTENTION_NEGATIVE
} sapIntentionBool;

/**
 * @brief 行人注意力
 */
typedef enum {
    /** 行人注意到当前交通状况 */
    ATTENTION,
    ATTENTION_INFLUENCE,
    /** 行人未注意到当前交通状况 */
    ATTENTION_NEGATIVE
} sapAttentionBool;

#ifdef __cplusplus
}
#endif
