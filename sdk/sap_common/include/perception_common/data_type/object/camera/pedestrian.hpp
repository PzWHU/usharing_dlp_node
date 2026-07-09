/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"

#include "perception_common/data_type/object/base_object.hpp"
#include "perception_common/data_type/object/camera/base_camera_attribute.hpp"

namespace senseAD {
namespace perception {

struct PedestrianIntention {
    IntentionBool crossing = IntentionBool::NEGATIVE;
    AttentionBool attention_to_ego_car = AttentionBool::NEGATIVE;
};

struct Pedestrian : public BaseObject {
    Pedestrian() {}
    explicit Pedestrian(BaseObject* object) : BaseObject() {}
    Pedestrian* GetThisPtr() override { return this; }

    // track info
    ObjectTrackInfo track_info;

    // Bbox
    CameraBBox2DInfo camera_bbox_info;

    // bbox motion
    BBox2DMotionInfo bbox_motion_info;

    // action
    AttrBool umbrella = AttrBool::UNKNOWN;
    AttrBool call = AttrBool::UNKNOWN;
    AttrBool phone = AttrBool::UNKNOWN;
    AttrBool wave = AttrBool::UNKNOWN;

    // motion
    AttrBool standing = AttrBool::UNKNOWN;

    // attribute
    BaseCameraAttribute attribute;

    // direction
    Eigen::Vector2f upper_body_direction = Eigen::Vector2f(0., 0.);
    Eigen::Vector2f lower_body_direction = Eigen::Vector2f(0., 0.);
    Eigen::Vector2f face_direction = Eigen::Vector2f(0., 0.);

    // intention
    PedestrianIntention intention;
    bool is_on_bike = false;

    // debug
    DebugInfo debug_info;
};

}  // namespace perception
}  // namespace senseAD
