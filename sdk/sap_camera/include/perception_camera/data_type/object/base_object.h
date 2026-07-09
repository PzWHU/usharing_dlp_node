/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

#include <stdint.h>
#include <string.h>

#include "perception_camera/sap_macros.h"
#include "perception_camera/data_type/object/base_data_struct.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 目标生命周期时间戳
 */
typedef struct {
    /** 传感器图像抓拍时间戳 */
    uint64_t raw_sensor_capture_timestamp_ns;
    /** 感知目标第一次被检测到的时间戳 */
    uint64_t sensor_perception_capture_timestamp_ns;
    /** 感知目标当前帧被跟踪到的时间戳 */
    uint64_t sensor_perception_output_timestamp_ns;
    /** 融合目标第一次被检测到的时间戳 */
    uint64_t fusion_capture_timestamp_ns;
    /** 融合目标当前帧被跟踪到的时间戳 */
    uint64_t fusion_output_timestamp_ns;
    /** 传感器ID */
    uint16_t sensor_id;
    /** 目标track ID */
    int64_t track_id;
} sapPerceptionLifeTimeHistory;

/**
 * @brief 初始化sapPerceptionLifeTimeHistory数据结构
 * @param[in] history 时间戳结构
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t
InitsapPerceptionLifeTimeHistory(sapPerceptionLifeTimeHistory* history);

/**
 * @brief 三维目标包围体
 */
typedef struct {
    /** 是否有效 */
    int8_t is_valid;
    /** 三维包围体[width, length, height]，length是视觉主方向上的长度，单位米 */
    float size[3];
    /** 目标大小的不确定度 */
    float size_uncertainty[3][3];
} sapSizeInfo;

/**
 * @brief 多边形
 */
typedef struct {
    /** 是否有效 */
    int8_t is_valid;
    /** 多边形点 */
    sapVector2f polygon_contour[8];  // TODO(ld): stuff
    /** 下边缘高度 */
    float height_lower_surface;
    /** 上边缘高度 */
    float height_upper_surface;
} sapPolygonBox;

typedef struct {
    uint16_t sensor_id;
    sapBbox2D box;
    // std::string camera_name;
    sapPolygonBox polygon_box;
} SapProjectBBoxNv;

/**
 * @brief 初始化sapPolygonBox数据结构
 * @param[in,out] box 多边形结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapPolygonBox(sapPolygonBox* box);

/**
 * @brief 方向角信息
 */
typedef struct {
    /** 是否有效 */
    int8_t is_valid;
    /** (yaw, pitch, rolle)表示的方向角 */
    float direction_ypr[3];
    /** 航向不确定度 */
    float direction_ypr_uncertainty[3][3];
    /** yaw角度，当前roll和pitch没有考虑，必须 */
    float yaw;
    /** yaw角度不确定度，必须 */
    float yaw_uncertainty;
    /** yaw方向的角速度，弧度/秒 */
    float yaw_rate;
    /** yaw角速度方差 */
    float yaw_rate_uncertainty;
} sapDirectionInfo;

/**
 * @brief 目标运动信息
 */
typedef struct StructMotionInfo {
    /** 是否有效 */
    int8_t is_valid;
    /** 目标中心位置，在自车坐标系下，单位米，必须 */
    float center[3];
    /** 目标中心位置协方差矩阵，必须 */
    float center_uncertainty[3][3];
    /** 三方向上速度，必须 */
    float velocity[3];
    /** 速度协方差矩阵 */
    float velocity_uncertainty[3][3];
    /** 加速度 */
    float acceleration[3];
    /** 加速度协方差矩阵 */
    float acceleration_uncertainty[3][3];
    /** 雅可比矩阵 */
    float jerk[3];
    /** 雅可比协方差矩阵 */
    float jerk_uncertainty[3][3];
    /** 车头方向速度 */
    float velocity_heading;
    /** 车头方向速度方差 */
    float velocity_heading_uncertainty;
    /** 车头方向速度变化率 */
    float velocity_heading_rate;
    /** 车头方向速度变化率方差 */
    float velocity_heading_rate_uncertainty;
    /** 目标矩形框左边缘是否被截断 */
    int8_t is_left_side_truncated;
    /** 目标矩形框右边缘是否被截断 */
    int8_t is_right_side_truncated;
    /** 目标矩形框上边缘是否被截断 */
    int8_t is_top_side_truncated;
    /** 目标矩形框下边缘是否被截断 */
    int8_t is_bottom_side_truncated;
} sapMotionInfo;

/**
 * @brief BV视角轮廓点
 */
typedef struct StructBirdViewCorners {
    /** 是否有效*/
    int8_t is_valid = false;
    /** BV视角轮廓点个数 */
    uint64_t bv_corner_num;
    /** BV视角轮廓点 */
    sapPoint3f* bv_corners;
    /** 轮廓点置信度 */
    float* corner_conf;
} sapBirdViewCorners;

/**
 * @brief 分配BV视角轮廓点内存
 * @param[in,out] bv_corners
 * @param[in] bv_corner_num
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t AllocateBirdViewCornersMemory(sapBirdViewCorners* bv_corners,
                                      uint64_t bv_corner_num);

/**
 * @brief 释放BV视角轮廓点内部内存，不包括bv_corners指针
 *
 * @param[in] bv_corners 输入BV视角轮廓点指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t ReleaseBirdViewCornersMemory(sapBirdViewCorners* bv_corners);

/**
 * @brief 目标运动状态
 */
typedef enum {
    /** 无效状态 */
    MOTION_INVALID_STATUS = -1,
    /** 状态未知 */
    MOTION_STATUS_UNKNOWN = 0,
    /** 目标静止 */
    MOTION_STATUS_STATIC,
    /** 目标停止 */
    MOTION_STATUS_STOPPED,
    /** 目标移动中 */
    MOTION_STATUS_MOVING,
    /** 目标左右横穿中 */
    MOTION_STATUS_CROSSING
} sapMotionStatus;

/**
 * @brief 目标运动状态信息
 */
typedef struct {
    /** 是否有效 */
    int8_t is_valid;
    /** 目标运动状态，必须 */
    sapMotionStatus motion_status;
    /** 未知状态的得分，0~1，越高越确认 */
    float unknown_score;
    /** 静止状态的得分 */
    float static_score;
    /** 停止状态的得分 */
    float stopped_score;
    /** 移动状态的得分 */
    float moving_score;
    /** 横穿状态的得分 */
    float crossing_score;
} sapMotionStatusInfo;

/**
 * @brief 初始化目标状态信息
 * @param[in] status 状态结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapMotionStatusInfo(sapMotionStatusInfo* status);

/**
 * @brief 目标跟踪状态
 */
typedef enum {
    /** 无效状态 */
    INVALID_STATUS = -1,
    /** 跟踪建立 */
    STATE_NEW = 0,
    /** 跟踪删除 */
    STATE_DELETED,
    /** 目标第一次被观测到 */
    STATE_MEASURED,
    /** 融合目标 */
    STATE_MERGE_NEW,
    /** 预测的目标 */
    STATE_PREDICTED,
    /** 融合目标后被删除的源目标 */
    STATE_MERGE_DELETED
} sapMaintenanceStatus;

/**
 * @brief 目标基本信息
 */
typedef struct {
    /** 传感器id */
    uint16_t sensor_id;
    /** 目标时间戳信息 */
    sapPerceptionLifeTimeHistory life_time_history;
    /** 目标类别 */
    int64_t label;
    /** 目标置信度，可选 */
    float existence_confidence;
    /** 目标子类别，可选 */
    int64_t sub_type;
    /** 跟踪ID, -1无效 */
    int64_t track_id;
    /** 当前预测的时间戳 */
    uint64_t current_prediction_timestamp_ns;
    /** 目标大小 */
    sapSizeInfo size_info;
    /** 目标多边形表述，可选 */
    sapPolygonBox polygon_box;
    /** 目标多边形Nv的表述，可选 */
    SapProjectBBoxNv project_bboxs[8];
    /** 目标多边形Nv的数量，可选 */
    int32_t project_bboxs_num;
    /** 目标方向信息 */
    sapDirectionInfo direction_info;
    /** 目标运动信息 */
    sapMotionInfo motion_info;
    /** 鸟瞰图下的目标轮廓点*/
    sapBirdViewCorners bv_corners;
    /** 目标运动状态信息 */
    sapMotionStatusInfo motion_status_info;
    /** 目标跟踪状态 */
    sapMaintenanceStatus maintenance_status;
} sapBaseObject;

/**
 * @brief 初始化目标基本信息
 * @param[in,out] base_object 目标基本状态结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapBaseObject(sapBaseObject* base_object);

/**
 * @brief 带3D位姿信息的目标
 */
typedef struct {
    /** 历史帧时间戳信息 最大5帧*/
    sapPerceptionLifeTimeHistory history_infos[5];  // TODO check max num

    /** 3D目标id */
    uint64_t id;

    /** 检测状态，取值纯感知 = 0 */
    int16_t detection_status;

    /** 置信度，范围[0,1] */
    float confidence;

    /** 3d位置 */
    sapPoint3d centroid;

    /** 3d姿态 */
    sapQuaterniond quaternion;

    /** 3d目标长度 */
    float length;
    /** 3d目标宽度 */
    float width;
    /** 3d目标高度 */
    float height;

    /** 车道、路口绑定信息预留 */
    uint64_t lane_ids[8];  // TODO check max num
    uint64_t junction_id;
} sapObject3D;

/**
 * @brief 初始化3D目标基本信息
 * @param[in,out] object_3d 3D目标基本状态结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapObject3D(sapObject3D* object_3d);

#ifdef __cplusplus
}
#endif
