/*
 * Copyright (C) 2024 by SenseTime Group Limited. All rights reserved.
 * Qi Chenyu <qichenyu1@senseauto.com>
 */
#pragma once

#include <string.h>
#include <stdlib.h>

#include "perception_camera/sap_camera_define.h"
#include "perception_camera/sap_lidar_define.h"
#include "perception_camera/data_type/object/base_data_struct.h"

#ifdef __cplusplus
extern "C" {
#endif

// @brief Followings for containing data types
// @brief general object type
typedef enum {
    TYPE_UNKNOWN = 0,
    // id follows kTypeMap in lidar_object_to_fusion_object.cpp
    VEHICLE = 1,
    PEDESTRIAN = 2,
    BIKE = 3,
    VEHICLE_BUS = 4,
    // TYPE_SIZE = 5,
    CONE = 5,
    POLE = 6,
    ISOLATION_BARREL = 7,
    TRIANGLE_WARNING = 8,
    ANIMAL = 9,
    GATE_ROD = 10,           // 1
    BARRIER = 11,            // 2
    BARRICADE = 12,          // 3
    CONSTRUCTION_SIGN = 13,  // 4
    OBSTACLES = 14,          // 5
    TYPE_SIZE = 15,
} sapLidarObjectType;

typedef struct {
    double x;
    double y;
    double z;
} Point3D_t;

typedef struct {
    float length;
    float width;
    float height;
} Size3D_t;

typedef struct {
    float pts[4][3];
    float confidences[4];
    float area;
} MinBox_t;

typedef struct {
    // @brief object id per frame, required
    uint32_t id;

    // @brief object captured timestamp, required
    uint64_t timestamp_ns;

    // @brief convex hull of the object, required
    // TODO(chenshengjie): Point2D+height?
    // polgon for output, it maybe bbox or pts_convhull
    uint64_t polygon_num;
    Point3D_t* polygon;
    int8_t is_polygon_valid;
    uint64_t polygon_visual_masks_num;
    int8_t* polygon_visual_masks;
    float polygon_height;
    float polygon_lower_height;

    uint64_t pts_convhull_num;
    Point3D_t* pts_convhull;
    uint64_t pts_convhull_visual_masks_num;
    int8_t* pts_convhull_visual_masks;

    // oriented bounding box information
    // @brief center of the bounding box, required
    Point3D_t center;
    // @brief covariance matrix of the center uncertainty, optional
    float center_uncertainty[3][3];
    // @brief size of bounding box, in the main direction, required
    Size3D_t size;
    // @brief size variance, optional
    Size3D_t size_variance;
    // @brief the yaw angle, yaw = 0.0 <=> direction(1, 0, 0), required
    float yaw;
    // @brief yaw variance, optional
    float yaw_variance;

    // @brief point cloud of the object, optional
    uint64_t cloud_num;
    LidarPoint* cloud;
    float static_score;
    float detect_confidence;

    MinBox_t min_box;

    // @brief object type, required
    sapLidarObjectType type;
} BaseLidarObject;

// @brief motion state
enum LidarObjectMotionState {
    MOTION_UNKNOWN = 0,
    MOTION_MOVING = 1,
    MOTION_STATIONARY = 2,
};

typedef struct {
    // @brief Followings for containing data types

    // @brief object base message
    BaseLidarObject base_object;
    // @brief tracked id, required
    uint64_t tracked_id;

    // @brief motion state of the tracked object, required
    LidarObjectMotionState motion_state;
    float static_confidence;

    // @brief existence confidence, required
    float confidence;

    // @brief false if the object is predicted from tracking
    int8_t is_observed = true;

    // @brief velocity of the object, required
    Point3D_t velocity;
    // @brief covariance matrix of the velocity uncertainty, required
    float velocity_uncertainty[3][3];
    // @brief acceleration of the object, required
    Point3D_t acceleration;
    // @brief covariance matrix of the acceleration uncertainty, required
    float acceleration_uncertainty[3][3];

    MinBox_t min_box;
    int tracking_age;
} sapLidarObject;

/**
 * @brief 初始化目标基本信息
 * @param[in,out] lidar_object 目标基本状态结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapLidarObject(sapLidarObject* lidar_object);


typedef struct {
    // @brief object id per frame, required
    uint32_t id;
    uint64_t tracked_id;
    uint64_t timestamp_ns;
    // @brief object type, required
    sapLidarObjectType type;
    // @brief size of bounding box, in the main direction, required
    Size3D_t size;
    // @brief center of the bounding box, required
    Point3D_t center;
    // @brief the yaw angle, yaw = 0.0 <=> direction(1, 0, 0), required
    float yaw;
    // @brief existence confidence, required
    float confidence;
    // @brief motion state of the tracked object, required
    LidarObjectMotionState motion_state;
    int tracking_age;
    // @brief box_2d[0]:center_camera_fov30, box_2d[1]:center_camera_fov120
    sapBbox2D box_2d[2];
} sapLidarObstacle;

/**
 * @brief 初始化目标基本信息
 * @param[in,out] lidar_object 目标基本状态结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t InitsapLidarObstacle(sapLidarObstacle* lidar_object);


#ifdef __cplusplus
}
#endif
