/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <vector>
#include <string>
#include <map>

#include "Eigen/Core"

#include "perception_common/base/data_type/bbox_2d.hpp"

namespace senseAD {
namespace perception {

enum class TrackStatus { INVALID = 0, NEW = 1, UPDATED = 2, COASTED = 3 };

struct ObjectTrackInfo {
    TrackStatus track_status;
    uint64_t track_age;
    bool is_tracked{false};
    uint64_t track_id{0};
};

struct CameraBBox2DInfo {
    Bbox2D raw_detection_box;
    Bbox2D tracked_box;
};

struct BBox2DMotionInfo {
    float width_velocity = 0.0f;
    float height_velocity = 0.0f;
    float width_height_ratio_velocity = 0.0f;
    float bottom_center_x_velocity = 0.0f;
    float bottom_center_y_velocity = 0.0f;
};

struct BaseCameraAttribute {
    // landmark
    bool is_landmark_valid = false;
    std::vector<Eigen::Vector2f> landmark;
    std::vector<float> landmark_scores;

    // viewport direction
    bool is_direction_valid = false;
    Eigen::Vector2f viewport_direction;
};

enum class DepthType {
    UNKNOWN = -1,
    HOMOGRAPHY = 0,
    NEAREST_FACE_WIDTH,
    WIDTH,
    LENGTH,
    HEIGHT,
    SKYLINE,
    MIXWH,
    MONO3D,
    PREDICT
};

static const std::map<DepthType, std::string> kDepthTypemap{
    {DepthType::UNKNOWN, "NO"},
    {DepthType::HOMOGRAPHY, "HOM"},
    {DepthType::NEAREST_FACE_WIDTH, "NFW"},
    {DepthType::WIDTH, "WID"},
    {DepthType::LENGTH, "LEN"},
    {DepthType::HEIGHT, "HEI"},
    {DepthType::SKYLINE, "SKY"},
    {DepthType::MIXWH, "MIX"},
    {DepthType::MONO3D, "3D"},
    {DepthType::PREDICT, "PRD"}};

struct DebugInfo {
    DepthType depth_type = DepthType::UNKNOWN;
};

// intention bool status
enum class IntentionBool { POSITIVE, NEGATIVE };
enum class AttentionBool { ATTENTION, INFLUENCE, NEGATIVE };

}  // namespace perception
}  // namespace senseAD
