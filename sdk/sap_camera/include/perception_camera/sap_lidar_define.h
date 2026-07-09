/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

#include <string.h>
#include "perception_camera/sap_camera_define.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief lidar 数据结构
 */
struct LidarPoint {
    /** 点云x坐标 **/
    float x;
    /** 点云y坐标 **/
    float y;
    /** 点云z坐标 **/
    float z;
    /** 激光束返回的强度 **/
    float intensity;
};

/**
 * @brief 输入的lidar帧
 *
 */
typedef struct {
    /** 图像时间戳 */
    uint64_t timestamp_ns;
    uint64_t sensor_send_timestamp_ns;
    uint64_t perception_capture_timestamp_ns;
    /** 帧号，自定义，无绝对意义，一般是顺序号 */
    int64_t frame_no;
    /** sensor编号 */
    int8_t sensor_id;
    /** sensor名称 */
    char sensor_name[MAX_NAME];
    /** lidar 点云数量*/
    uint64_t lidar_point_num;
    /** lidar 数据*/
    LidarPoint* lidar_point;
    /** lidar 数据 NvSciBuf*/
    float* lidar_data;
    uint64_t max_lidar_point_num;
    void* user_data;
} sapLidarFrame;

/**
 * @brief 初始化sapLidarFrame结构体数据
 * @param[in] frame sapLidarFrame类型结构体指针
 * @return 成功返回0, 否则返回错误码
 */
SAP_API int32_t initSapLidarFrame(sapLidarFrame* frame);

#ifdef __cplusplus
}
#endif
