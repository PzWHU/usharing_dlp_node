/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <vector>
#include <map>
#include <string>

#include "Eigen/Core"

#include "perception_common/data_type/object/base_object.hpp"
#include "perception_common/data_type/object/camera/base_camera_attribute.hpp"

namespace senseAD {
namespace perception {

// // vehicle light status
// enum class BinaryDirection {
//     UNKNOWN = -1,
//     BINARY_DIRECTION_FRONT,
//     BINARY_DIRECTION_BACK
// };
struct AutomobileLightStatus {
    AttrBool left_turn = AttrBool::UNKNOWN;
    float left_turn_confidence = 0.0;
    AttrBool right_turn = AttrBool::UNKNOWN;
    float right_turn_confidence = 0.0;
    AttrBool hazard = AttrBool::UNKNOWN;
    float hazard_confidence = 0.0;
    AttrBool brake = AttrBool::UNKNOWN;
    float brake_confidence = 0.0;
    BinaryDirection binary_direction = BinaryDirection::UNKNOWN;
    float binary_direction_confidence = 0.0;
};
enum class LampClsLabel {
    UNKNOWN = 0,
    TURN_UNKNOWN = 1,
    TURN_NO,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_ALL_SHARK,
    BRAKE_UNKNOWN = 6,
    BRAKE_MOVING,
    BRAKE,
    CAR_HEAD_FRONT = 9,
    CAR_HEAD_RIGHT,
    CAR_HEAD_BACK,
    CAR_HEAD_LEFT,
    SCENE_DAY = 13,
    SCENE_NIGHT,
    TOP_UNKNOWN = 15,
    TOP_LIGHT_OFF,
    TOP_LIGHT_ON,
    LEFT_LIGHT_UNKNOWN = 18,
    LEFT_LIGHT_OFF,
    LEFT_LIGHT_BRAKE_ON,
    LEFT_LIGHT_TURN_ON,
    LEFT_LIGHT_ALL_ON,
    RIGHT_LIGHT_UNKNOWN = 23,
    RIGHT_LIGHT_OFF,
    RIGHT_LIGHT_BRAKE_ON,
    RIGHT_LIGHT_TURN_ON,
    RIGHT_LIGHT_ALL_ON
};

struct LampClsRes {
    LampClsLabel label = LampClsLabel::UNKNOWN;
    std::vector<float> confidence;
};
struct AutomobileLightCls {
    std::vector<LampClsRes> lamp_cls;
};

// Depth Det info
enum class DepthFineOriLabel {
    LATERAL_LEFT = 0,
    LATERAL_RIGHT,
    BACK_STRAIGHT,
    BACK_LEFT,
    BACK_RIGHT,
    FRONT_STRAIGHT,
    FRONT_LEFT,
    FRONT_RIGHT,
    ORI_UNKNOWN
};

static const std::map<DepthFineOriLabel, std::string> kDepthFineOriLabelmap{
    {DepthFineOriLabel::LATERAL_LEFT, "L"},
    {DepthFineOriLabel::LATERAL_RIGHT, "R"},
    {DepthFineOriLabel::BACK_STRAIGHT, "B"},
    {DepthFineOriLabel::BACK_LEFT, "BL"},
    {DepthFineOriLabel::BACK_RIGHT, "BR"},
    {DepthFineOriLabel::FRONT_STRAIGHT, "F"},
    {DepthFineOriLabel::FRONT_LEFT, "FL"},
    {DepthFineOriLabel::FRONT_RIGHT, "FR"},
    {DepthFineOriLabel::ORI_UNKNOWN, "NO"}};

struct DepthDetPoint {
    float nearest_point_x = 0.;
    float depth_point_x = 0.;
    float depth_point_y = 0.;
    Eigen::Vector2f landmark_rear_point;
    Eigen::Vector2f landmark_front_point;
    int orientation = -1;
    DepthFineOriLabel label = DepthFineOriLabel::ORI_UNKNOWN;
};

// lane assignments
enum class LaneAssignType { UNKNOWN = -1, LEFT_LANE, RIGHT_LANE, EGO_LANE };

enum class SideDoorStatus {
    DEFAULT = -2,
    UNKNOWN = -1,
    CLOSED = 0,
    OPEN = 1,
};

enum class RearDoorStatus {
    DEFAULT = -2,
    UNKNOWN = -1,
    CLOSED = 0,
    OPEN_UP_OR_DOWN = 1,
    OPEN_LEFT = 2,
    OPEN_RIGHT = 3,
    OPEN_LEFT_AND_RIGHT = 4,
};

struct AutomobileDoorStatus {
    AttrBool bbox_validity = AttrBool::DEFAULT;
    float bbox_validity_confidence = 0.0;
    SideDoorStatus left_door_status = SideDoorStatus::DEFAULT;
    float left_door_status_confidence = 0.0;
    SideDoorStatus right_door_status = SideDoorStatus::DEFAULT;
    float right_door_status_confidence = 0.0;
    RearDoorStatus rear_door_status = RearDoorStatus::DEFAULT;
    float rear_door_status_confidence = 0.0;
};
#ifndef AUTOMOBILEDOORSTATUS_INT
#define AUTOMOBILEDOORSTATUS_INT
struct AutomobileDoorStatusINT {
    int64_t bboxValidity    = -2;
    int64_t leftDoorStatus  = -2;
    int64_t rightDoorStatus = -2;
    int64_t rearDoorStatus  = -2;
};
#endif
struct Automobile : public BaseObject {
    Automobile() {}
    explicit Automobile(BaseObject* object) : BaseObject() {}
    Automobile* GetThisPtr() override { return this; }

    // track info
    ObjectTrackInfo track_info;

    // Bbox
    CameraBBox2DInfo camera_bbox_info;

    // bbox motion
    BBox2DMotionInfo bbox_motion_info;

    // 3DBbox
    std::vector<Eigen::Vector2f> camera_bbox3d_info;

    // attribute
    BaseCameraAttribute attribute;

    // 2.5d detection
    DepthDetPoint depth_point;

    // lane assignments
    std::vector<LaneAssignType> lane_assignments;

    // light status
    AutomobileLightStatus automobile_light_status;
    float automobile_light_conf;
    AutomobileLightCls automobile_light_cls;

    // door status
    AutomobileDoorStatus automobile_door_status;
    AutomobileDoorStatusINT door_status;


    // debug
    DebugInfo debug_info;
};

}  // namespace perception
}  // namespace senseAD
