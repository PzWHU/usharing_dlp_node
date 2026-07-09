/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

#include <stdint.h>
#include <string.h>

#include "perception_camera/sap_camera_define.h"
#include "perception_camera/data_type/object/base_camera_attribute.h"
#include "perception_camera/data_type/object/base_object.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 车辆关键点深度信息，2D PVB使用
 */
typedef struct {
    /** 像素坐标，测距点 */
    float nearest_point_x;
    /** 深度点x坐标 */
    float depth_point_x;
    /** 深度点y坐标 */
    float depth_point_y;
    /** 他车的后车轮中心点的像素坐标 */
    float landmark_rear_point[2];
    /** 他车的前车轮中心点的像素坐标 */
    float landmark_front_point[2];
    /** 方向 */
    int orientation;
    /** 朝向标签, 可选值为枚举
    LATERAL_LEFT = 0 侧面靠左，LATERAL_RIGHT侧面靠右，BACK_STRAIGHT
    正后，BACK_LEFT 后面靠左， BACK_RIGHT 后面靠右，FRONT_STRAIGHT
    正前，FRONT_LEFT 前面靠左，FRONT_RIGHT 前面靠右，ORI_UNKNOWN 未知朝向 */
    int32_t label;
} sapDepthDetPoint;

/**
 * @brief 初始化sapDepthDetPoint结构体
 * @param[in,out] depth_point 车辆关键点结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapDepthDetPoint(sapDepthDetPoint* depth_point);

/**
 * @brief 车灯状态
 */
typedef struct {
    /** 左转，可选值为DEFAULT=-2, UNKNOWN = -1（经过模型分类）, NEGATIVE=0,
     * POSITIVE=1 */
    int16_t left_turn;
    /** 左转置信度 */
    float left_turn_confidence;
    /** 右转，可选值同左转 */
    int16_t right_turn;
    /** 右转置信度 */
    float right_turn_confidence;
    /** 急刹状态 */
    int16_t hazard;
    /** 急刹状态置信度 */
    float hazard_confidence;
    /** 刹车 */
    int16_t brake;
    /** 刹车置信度 */
    float brake_confidence;
    /** 行驶方向，可选值为UNKNOWN=-1, BINARY_DIRECTION_FRONT=1,
     * BINARY_DIRECTION_BACK =2 */
    int binary_direction;
    /** 方向置信度 */
    float binary_direction_confidence;
} sapAutomobileLightStatus;

/**
 * @brief 初始化sapAutomobileLightStatus结构体
 * @param[in,out] status 车灯结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapAutomobileLightStatus(sapAutomobileLightStatus* status);

/**
 * @brief 车门状态
 */
typedef enum {
    /** 未知 */
    UNKNOWN_DOOR_STATE = -1,
    /** 关 */
    CLOSE,
    /** 左开 */
    LEFTOPEN,
    /** 右开 */
    RIGHTOPEN,
    /** 左右都开 */
    OPEN
} sapCarDoorState;

/**
 * @brief still状态
 */
typedef enum {
    /** 未知 */
    UNKNOWN_MOTION_STATE = -1,
    /** 静止 */
    STATIC,
    /** 运动 */
    DYNAMIC
} sapMotionState;

/**
 * @brief 车辆目标信息
 */
typedef struct {
    /** 目标基本信息 */
    sapBaseObject base_info;
    /** 目标2D框 */
    sapCameraBBox2DInfo camera_bbox_info;
    /** 目标运动信息 */
    sapBBox2DMotionInfo bbox_motion_info;
    /** 关键点信息 */
    sapBaseCameraAttribute attribute;
    /** 深度点信息 */
    sapDepthDetPoint depth_point;
    /** 车道行驶信息，value为UNKNOWN_LANE = -1, LEFT_LANE, RIGHT_LANE, EGO_LANE
     */
    int16_t* lane_assignments;
    /** 车道数量 */
    uint64_t lane_assignment_num;
    /** 车灯状态 */
    sapAutomobileLightStatus automobile_light_status;
    /** 深度类型，value为 HOMOGRAPHY =
     * 0,NEAREST_FACE_WIDTH，WIDTH，LENGTH，HEIGHT，SKYLINE，MIXWH，MONO3D，PREDICT
     */
    int16_t depth_type;
    /**
     * still状态，value为UNKNOWN = -1, STATIC = 0, DYNAMIC = 1
     */
    sapMotionState motion_state;
    /**
     *  车门状态，value为UNKNOWN = -1, CLOSE = 0, LEFTOPEN = 1, RIGHTOPEN = 2,
     * OPEN = 3
     */
    sapCarDoorState car_door_state;
} sapAutomobile;

/**
 * @brief 初始化sapAutomobile结构体
 * @param[in,out] automobile 车辆目标信息结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapAutomobile(sapAutomobile* automobile);

#ifdef __cplusplus
}
#endif
