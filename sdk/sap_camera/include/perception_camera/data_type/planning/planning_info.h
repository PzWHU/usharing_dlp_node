/*
 * Copyright (C) 2025 by SenseTime Group Limited. All rights reserved.
 * Qi Chenyu <qichenyu1@senseauto.com>
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include "perception_camera/data_type/base_type.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PlanningInstruction
 */
typedef struct {
    bool updated_by_vehicle_status;
    bool enter_autodrive_flag;
} sapPlanningInstruction;

/**
 * @brief TrajectoryPoint
 */
typedef struct {
    sapVector2f position;
    sapVector2f direction;
    double velocity;
    float theta;
    float curvature;
    double sum_distance;
    double time_difference;
    double yaw_rate;
    double lon_acc;
    double lat_acc;
    double lat_jerk;
    double front_wheel_angle;
    uint64_t timestamp = 0;
    double theta_error;
} sapTrajectoryPoint;

/**
 * @brief PredictionTrajectoryPoint
 */
typedef struct {
    sapPoint2f* polygon_contour;
    uint64_t polygon_contour_num;

    sapPoint2f position;
    sapPoint2f direction;
    sapPoint2f speed;

    sapPoint2f variance_position;
    sapPoint2f variance_speed;

    float covariance_position;
    float covariance_speed;
} sapPredictionTrajectoryPoint;

/**
 * @brief PredictionTrajectory
 */
typedef struct {
    char predictor_name[30];
    char target_road_id[30];
    char target_lane_id[30];
    uint64_t point_interval_time;  // trajectory point interval, ns
    uint32_t num_points;           // point number points, include current point
    sapPredictionTrajectoryPoint* trajectory_point_array;
    uint64_t trajectory_point_array_num;
    bool is_abnormal;
    bool is_collision;
    bool is_selected;   // final selected trajectory
    float probability;  // trajectory probability
    float score;        // decision scores
    uint64_t intention_type;
} sapPredictionTrajectory;

/**
 * @brief PredictionObject
 */
typedef struct {
    uint64_t id;
    uint64_t type;
    uint64_t object_label;
    float length;
    float width;
    float height;

    sapPoint2f* polygon_contour;
    uint64_t polygon_contour_num;

    sapPoint2f position;
    sapPoint2f speed;
    sapPoint2f acceleration;
    sapPoint2f heading_direction;

    uint64_t info_state;
    uint64_t scene_type;
    uint16_t move_step;
    uint64_t importance_level;

    uint64_t intention_type;
    char target_road_id[30];
    char target_lane_id[30];
    sapPredictionTrajectory* trajectory_array;
    uint64_t trajectory_array_num;
    sapPoint2f* history_trajectory_points;
    uint64_t history_trajectory_points_num;
} sapPredictionObject;

/**
 * @brief PredictionObjectArray
 */
typedef struct {
    uint64_t prediction_time;
    uint64_t fusion_time;
    sapPredictionObject* prediction_object_array;
    uint64_t prediction_object_array_num;
} sapPredictionObjectArray;

#ifdef __cplusplus
}
#endif