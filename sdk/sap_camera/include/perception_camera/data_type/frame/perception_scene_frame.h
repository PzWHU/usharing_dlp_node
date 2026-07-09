/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "perception_camera/sap_camera_define.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 天气类别
 */
typedef enum {
    /** 未知天气 */
    WEATHER_UNKNOWN = 0,
    /** 晴天 */
    SUNNY,
    /** 多云 */
    CLOUDY,
    /** 雨天 */
    RAINY,
    /** 雪天 */
    SNOWY,
    /** 雾天 */
    FOGGY,
    /** 沙尘天气 */
    SANDY,
    /** 阴天 */
    OVERCAST,
    /** 其它天气 */
    OTHERS
} sapSceneWeatherInfo;

/**
 * @brief 道路路口类型
 */
typedef enum {
    /** 未知 */
    INTER_SRCTION_UNKNOWN = -1,
    /** T型路口 */
    T_SHAPE,
    /** Y型分叉路口 */
    Y_SHAPE,
    /** 十字交叉路口 */
    CROSS_SHAPE,
    /** x型交叉路口 */
    X_SHAPE,
    /** 环形路口 */
    ROUND_SHAPE
} sapRoadIntersection;

/**
 * @brief 场景亮度类型
 */
typedef enum {
    /** 亮度未知 */
    BRIGHT_UNKNOWN = 0,
    /** 白天 */
    DAYTIME,
    /** 暗淡的，表示夜晚有灯光情况，画面较为清晰 */
    NIGHT_BRIGHT,
    /** 黑暗的，表示夜晚无灯的情况 */
    NIGHT_DARK
} sapSceneBrightnessInfo;

/**
 * @brief 道路环境类型
 */
typedef enum {
    /** 未知环境 */
    ENV_UNKNOWN = 0,
    /** 开放道路 */
    OPEN_ROAD,
    /** 封闭道路 */
    CLOSED_ROAD,
    /** 隧道 */
    TUNNEL
} sapSceneEnvironmentInfo;

/**
 * @brief 道路平整度类型
 */
typedef enum {
    /** 未知坡度 */
    SLOP_UNKNOWN = -1,
    /** 路面平坦 */
    SMOOTH,
    /** 路面不平整，有凸起 */
    BUMPS,
    GENTLE
} sapRoadSlop;

/**
 * @brief 路面质量等级
 */
typedef enum {
    /** 未知 */
    QUA_UNKNOWN = -1,
    /** A级路面 */
    LEVEL_A,
    /** B级路面 */
    LEVEL_B,
    /** C级路面 */
    LEVEL_C
} sapRoadQuality;

/**
 * @brief 道路场景信息
 */
typedef struct _sapRoadSceneInfo {
    /** 道路路口类型 */
    sapRoadIntersection road_intersection;
    /** 道路平整度类型 */
    sapRoadSlop road_slop;
    /** 道路质量等级 */
    sapRoadQuality road_qulity;
} sapRoadSceneInfo;

/**
 * @brief 单帧场景信息
 */
typedef struct _sapPerceptionSceneFrame {
    /** 时间戳 */
    uint64_t timestamp_ns;  // = 0;
    /** 帧号 */
    uint64_t frame_id;
    /** 传感器编号 */
    uint16_t sensor_id;
    /** 当前天气情况  */
    sapSceneWeatherInfo scene_weather;
    /** 当前场景亮度情况 */
    sapSceneBrightnessInfo scene_brightness;
    /** 当前路面环境 */
    sapSceneEnvironmentInfo scene_environment;
    /** 道路场景数量 */
    uint64_t road_scene_num;  // = 0;
    /** 道路场景集合 */
    sapRoadSceneInfo* road_scenes;
    uint64_t sensor_send_timestamp_ns;
} sapPerceptionSceneFrame;

/**
 * @brief 初始化场景信息结构体
 * @param[in,out] scene 场景信息结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapPerceptionSceneFrame(sapPerceptionSceneFrame* scene);

/**
 * @brief 释放场景信息结构体内部指针指向的内存
 * @param[in] scene 场景结构体指针
 */
SAP_API void ReleaseSceneFrameMemory(sapPerceptionSceneFrame* scene);

#ifdef __cplusplus
}
#endif
