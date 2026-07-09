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

struct NonAutomobileVehicleLightStatus {
    AttrBool left_turn = AttrBool::UNKNOWN;
    AttrBool right_turn = AttrBool::UNKNOWN;
    AttrBool brake = AttrBool::UNKNOWN;
};

struct NonAutomobileVehicle : public BaseObject {
    NonAutomobileVehicle() {}
    explicit NonAutomobileVehicle(BaseObject* object) : BaseObject() {}
    NonAutomobileVehicle* GetThisPtr() override { return this; }

    // track info
    ObjectTrackInfo track_info;

    // Bbox
    CameraBBox2DInfo camera_bbox_info;

    // bbox motion
    BBox2DMotionInfo bbox_motion_info;

    // attribute
    BaseCameraAttribute attribute;

    // light status
    NonAutomobileVehicleLightStatus non_automobile_vehicle_light_status;

    bool is_with_ped = false;
};

}  // namespace perception
}  // namespace senseAD
