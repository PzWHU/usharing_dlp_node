/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"

#include "perception_common/base/data_type/traffic_light_info.hpp"

#include "perception_common/data_type/object/camera/base_camera_attribute.hpp"
#include "perception_common/data_type/object/base_object.hpp"

namespace senseAD {
namespace perception {

struct TrafficLight : public BaseObject {
    TrafficLight() {}
    explicit TrafficLight(BaseObject* object) : BaseObject() {}
    TrafficLight* GetThisPtr() override { return this; }

    // Bbox
    CameraBBox2DInfo camera_bbox_info;

    // countdown
	int32 countdown_second = -1;
    int16_t tl_num = -1;
    // color_label
    uint16_t color_label = static_cast<uint16_t>(TrafficSignal::NONE);
    uint16_t status_label = static_cast<uint16_t>(TrafficLightStatus::NONE);
    uint16_t type_label = static_cast<uint16_t>(GeneralTurnType::NONE);
};

}  // namespace perception
}  // namespace senseAD
