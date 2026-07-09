/*
 * Copyright (C) 2024 by SenseTime Group Limited. All rights reserved.
 * Wang Zhaowei <wangzhaowei@senseauto.com>
 */
#pragma once

typedef float float32_t;
typedef double float64_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 单精度浮点类型二维向量
 */
typedef struct {
    /** x向量 */
    float32_t x;
    /** y向量 */
    float32_t y;
} sapVector2f;

/**
 * @brief 单精度浮点类型二维坐标点
 */
typedef struct {
    /** x坐标 */
    float32_t x;
    /** y坐标 */
    float32_t y;
} sapPoint2f;

/**
 * @brief 单精度浮点类型二维点sapPoint2f的扩展版本
 */
typedef struct {
    /** x坐标 */
    float x;
    /** y坐标 */
    float y;
    /** 几何置信度 */
    float geometric_confidence;
} sapPoint2fExtended;

/**
 * @brief 单精度浮点类型三维向量
 */
typedef struct {
    /** x向量 */
    float32_t x;
    /** y向量 */
    float32_t y;
    /** z向量 */
    float32_t z;
} sapVector3f;

typedef sapVector3f sapPoint3f;

/**
 * @brief 3D点坐标
 *
 */

/**
 * @brief 经纬度坐标
 * 定义一个名为PointLLH_t的结构体，包含三个成员变量：
 * lat（float64_t类型，表示纬度）、
 * lon（float64_t类型，表示经度）
 * 和height（float64_t类型，表示海拔）
 */
typedef struct {
    /** lat（float64_t类型，表示纬度）, ranging from -180 to 180 */
    float64_t lat;
    /** lon（float64_t类型，表示经度）, ranging from -90 to 90 */
    float64_t lon;
    /** height（float64_t类型，表示海拔），单位米*/
    float64_t height;
} sapPointLLH_t;
typedef struct {
    /** x, East from the origin, in meters. */
    float64_t x;
    /** y, North from the origin, in meters. */
    float64_t y;
    /** z, Up from the WGS-84 ellipsoid, in meters. */
    float64_t z;
} sapPointENU_t;

typedef sapPointLLH_t sapPointLLH;
typedef sapPointENU_t sapPointENU;

#ifdef __cplusplus
}
#endif