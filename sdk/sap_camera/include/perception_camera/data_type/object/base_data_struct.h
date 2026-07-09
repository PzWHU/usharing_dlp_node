/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

#include "perception_camera/data_type/base_type.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 3d位置坐标
 */
typedef struct {
    /** x坐标 */
    double x;
    /** y坐标 */
    double y;
    /** z坐标 */
    double z;
    /** 权重 */
    float weight;
} sapPoint3d;

/**
 * @brief 3d姿态信息
 */
typedef struct {
    /** 四元数w分量 */
    double qw;
    /** 四元数x分量 */
    double qx;
    /** 四元数y分量 */
    double qy;
    /** 四元数z分量 */
    double qz;
} sapQuaterniond;

/**
 * @brief 二维矩形目标框
 */
typedef struct {
    /** 是否被初始化，0-未初始化，1-初始化 */
    int8_t initialized;
    /** 左上角点x坐标 */
    float top_left_x;
    /** 左上角点y坐标 */
    float top_left_y;
    /** 右下角点x坐标 */
    float bottom_right_x;
    /** 右下角点y坐标 */
    float bottom_right_y;
    /** 矩形框出的目标置信度 */
    float confidence;
    /** 矩形左边是否被截断，0-未截断，1-截断 */
    int8_t is_left_side_truncated;
    /** 矩形右边是否被截断，0-未截断，1-截断 */
    int8_t is_right_side_truncated;
    /** 矩形下边是否被截断，0-未截断，1-截断 */
    int8_t is_bottom_side_truncated;
    /** 矩形上边是否被截断，0-未截断，1-截断 */
    int8_t is_top_side_truncated;
} sapBbox2D;

#ifdef __cplusplus
}
#endif
