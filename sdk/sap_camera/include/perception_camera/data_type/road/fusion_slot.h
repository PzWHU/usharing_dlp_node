/*
 * Copyright (C) 2025 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <stdint.h>

#include "perception_camera/data_type/base_type.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Pose2f {
    sapPoint2f position;
    float yaw;
} sapPose2f;

typedef struct BoundaryPoint {
    sapPoint3f position;
    int32_t age;
    int32_t confidence;

    // source type
    // invalid = 0
    // fsline = 1
    // lidar = 2
    uint8_t source;
} sapBoundaryPoint;

typedef struct RoadLinkNode {
    sapPoint3f node;
    int32_t type;
    int32_t confidence;
} sapRoadLinkNode;

typedef struct RoadBoundary {
    uint64_t timestamp;

    sapBoundaryPoint *left_boundaries;
    uint32_t left_boundaries_num;
    sapBoundaryPoint *right_boundaries;
    uint32_t right_boundaries_num;

    uint64_t left_boundary_point_num;
    uint64_t right_boundary_point_num;

    sapRoadLinkNode *node_list;
    uint32_t node_list_num;
} sapRoadBoundary;

typedef struct SlotBoundary {
    uint64_t timestamp;

    sapBoundaryPoint *left_boundaries;
    uint32_t left_boundaries_num;
    sapBoundaryPoint *right_boundaries;
    uint32_t right_boundaries_num;

    uint64_t left_boundary_point_num;
    uint64_t right_boundary_point_num;
} sapSlotBoundary;

typedef struct FusionSlot {
    // four corners of the slot
    sapPoint2f *corners;
    uint32_t corners_num;

    // two objects
    sapPoint2f *objects;
    uint32_t objects_num;

    // two object at slot end
    sapPoint2f *point_oblique;
    uint32_t point_oblique_num;

    // reference point
    sapPose2f ref_point;
    // pose of ego car
    sapPose2f start_pose;
    // pose of the slot
    sapPose2f end_pose;

    // from line01 to nearest obstacle - parallel to line01
    float space_depth;
    // perpendicular to line01 - slot useful width
    float space_length;
    // the distance between stop limiter and slot entry line
    float limiter_depth;

    sapRoadBoundary road_bound;
    sapSlotBoundary slot_bound;

    uint32_t id;
    uint32_t apa_id;

    // slot type
    // DEFAULT = 0,
    // HORIZONTAL = 1,
    // VERTICAL = 2,
    // OBLIQUE_30 = 3,
    // OBLIQUE_45 = 4,
    // OBLIQUE_60 = 5,
    // THREE_DIMENSIONAL = 6,
    // SPACE_HORIZONTAL = 7,
    // SPACE_VERTICAL = 8,
    // SPACE_INCLINED = 9,
    // DRAG_VERTICAL = 10,
    // DRAG_HORIZONTAL = 11,
    // DRAG_OBLIQUE = 12
    uint8_t type;

    // slot state
    // OCCUPIED = 0,
    // AVAILABLE = 1,
    // UNKNOWN = 2,
    // ERROR = 3,
    uint8_t state;

    // invalid reason
    // DEFAULT = 0,
    // WELL_DETECTED = 1,
    // SIZE = 2,    0-5预规划模块输出
    // ANGLE = 3,
    // DISTANCE = 4,
    // BOUNDARY = 5,
    // PRE_PLAN = 6, 选中车位后，规划失败
    // DEPTH = 7, 泊车过程中失败
    // OD = 8, 泊车过程中失败
    uint8_t invalid_reason;

    // slot source
    // DEFAULT = 0,
    // IPM = 1,
    // ULTRASONIC = 2,
    // FREESPACE = 3,
    // HMI = 4,
    uint8_t source;

    // [0-100] 70 is good, 80 is better.
    uint8_t confidence;

    // Available parking directions, each bit represents a direction
    // FRONT_IN = 0b10000000,
    // REAR_IN = 0b01000000,
    // FRONT_LEFT_OUT = 0b00100000,
    // FRONT_CENTER_OUT = 0b00010000,
    // FRONT_RIGHT_OUT = 0b00001000,
    // REAR_LEFT_OUT = 0b00000100,
    // REAR_CENTER_OUT = 0b00000010,
    // REAR_RIGHT_OUT = 0b00000001,
    uint8_t parking_direction_available;

    // Recommended parking direction, represented by a single bit
    uint8_t parking_direction_recommend;

    // 0: dont need; 1:need replan
    uint8_t need_replan_flag;

    int16_t region_id;

    float z;
} sapFusionSlot;

typedef struct FusionSlotInfo {
    sapFusionSlot *slots;
    uint32_t slots_num;
    sapFusionSlot *target_slots;
    uint32_t target_slots_num;
} sapFusionSlotInfo;

/**
 * @brief 初始化结构体
 * @param[in] fusion_slot 融合车位结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t InitsapFusionSlotInfo(sapFusionSlotInfo *fusion_slot);

/**
 * @brief 释放结构体指针和构体内部内存
 * @param[in] fusion_slot 融合车位结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t ReleasesapFusionSlotInfo(sapFusionSlotInfo *fusion_slot);

#ifdef __cplusplus
}
#endif