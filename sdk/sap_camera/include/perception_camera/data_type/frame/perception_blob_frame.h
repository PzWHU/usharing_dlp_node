/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "perception_camera/sap_camera_define.h"
#include "perception_camera/data_type/object/base_data_struct.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 光斑类型
 */
typedef enum {
    /** 未知光斑类型 */
    BLOB_TYPE_UNKNOWN = 0,
    /** 汽车前照灯光斑 */
    BLOB_TYPE_HEADLIGHT = 1,
    /** 汽车尾灯光斑 */
    BLOB_TYPE_TAILLIGHT = 2,
    /** 路灯光斑 */
    BLOB_TYPE_STREETLAMP = 3,
} sapPerceptionBlobType;

/**
 * @brief 光斑状态
 */
typedef enum {
    /** 未知光斑状态 */
    BLOB_STATUS_UNKNOWN = 0,
    /** 静态光斑 */
    BLOB_STATUS_STATIC = 1,
    /** 动态光斑 */
    BLOB_STATUS_DYNAMIC = 2,
} sapPerceptionBlobStatus;

/**
 * @brief 光斑场景亮度状态
 *
 */
typedef enum {
    /** 未知亮度 */
    BRIGHTNESS_STATE_UNKNOWN = 0,
    /** 明亮状态 */
    BRIGHTNESS_STATE_BRIGHT,
    /** 较黑暗状态 */
    BRIGHTNESS_STATE_DARK,
} sapPerceptionBlobBrightnessState;

/**
 * @brief 光斑目标属性
 */
typedef struct _sapBlobObjectAttributes {
    /** 光斑类型 */
    sapPerceptionBlobType blob_type;
    /** 类型置信度 */
    float type_confidence;
    /** 光斑状态 */
    sapPerceptionBlobStatus blob_status;
    /** 对向车辆概率，百分数，0-100的整数值 */
    uint8_t probability_oncoming;
    /** 同向车辆概率，百分数，0-100的整数值 */
    uint8_t probability_proceeding;
} sapBlobObjectAttributes;

/**
 * @brief 初始化光斑目标结构体
 * @param[in] attributes 光斑目标属性结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitBlobObjectAttributes(sapBlobObjectAttributes* attributes);

/**
 * @brief 光斑目标位置和运动信息
 */
typedef struct _sapBlobObjectKinematicWorld {
    /** 光斑目标框，图像坐标系 */
    sapBbox2D det_bbox_2d;
    /** 光斑中心点在车体坐标系下的距离信息 */
    float eul_distance;
    /** 光斑中心点在车体坐标系下的坐标(x,y,z)，单位米 */
    float center_pos[3];
    /** 光斑目标框各边中心点向量与光轴夹角，[上,下左,右] */
    float angle_info[4];
} sapBlobObjectKinematicWorld;

/**
 * @brief 光斑基本信息
 */
typedef struct _sapBlobObjectGeneral {
    /** 目标跟踪ID */
    uint64_t object_id;
    /** 配对车灯的跟踪ID */
    uint64_t object_id_paired;
    /** 固定跟踪帧数范围内跟踪成功帧数，track观察窗口16，
     * 一个bit占位光斑的一帧的跟踪状态，先保留 */
    uint16_t object_cycles_traced;
    /** 光强度 */
    uint16_t intensity;
    /** 光通量 */
    float luminous_flux;
} sapBlobObjectGeneral;

/**
 * @brief 光斑目标信息
 */
typedef struct _sapBlobObject {
    /** 光斑目标属性 */
    sapBlobObjectAttributes blob_object_attributes;
    /** 光斑位置信息 */
    sapBlobObjectKinematicWorld blob_object_kinematic_world;
    /** 光斑基本信息 */
    sapBlobObjectGeneral blob_object_general;
} sapBlobObject;

/**
 * @brief 初始化光斑目标信息结构体
 * @param[in] blob 光斑目标结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapBlobObject(sapBlobObject* blob);

/**
 * @brief 光斑集合信息
 */
typedef struct _sapBlobStruct {
    /** 光斑数量 */
    uint32_t blob_info_num;
    /** 最大光斑数量 */
    uint32_t max_blob_info_num;
    /** 光斑集合 */
    sapBlobObject* blob_info_results;
    /** 当前场景亮度信息 */
    sapPerceptionBlobBrightnessState blob_brightness_state;
} sapBlobStruct;

/**
 * @brief 单帧光斑感知结果
 *
 */
typedef struct _sapPerceptionBlobFrame {
    /** 图像帧号 */
    uint64_t frame_id;
    /** 相机编号 */
    uint16_t sensor_id;  // SensorID::UNKNOWN_SENSOR
    /** 相机名称 */
    char sensor_name[30];
    /** 当前帧时间戳 */
    uint64_t timestamp_ns;
    /** 光斑集合 */
    sapBlobStruct blob_struct;
    uint64_t sensor_send_timestamp_ns;
} sapPerceptionBlobFrame;

/**
 * @brief 初始化光斑感知结果结构体
 * @param[in,out] blob 光斑感知结果结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapPerceptionBlobFrame(sapPerceptionBlobFrame* blob);

/**
 * @brief 释放光斑感知结果结构体内部指针指向的内存
 * @param[in,out] blob 光斑感知结果结构体指针
 */
SAP_API void ReleaseBlobFrameMemory(sapPerceptionBlobFrame* blob);

#ifdef __cplusplus
}
#endif
