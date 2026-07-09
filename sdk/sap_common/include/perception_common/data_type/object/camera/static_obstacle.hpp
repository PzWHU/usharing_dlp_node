/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"

#include "perception_common/data_type/object/camera/base_camera_attribute.hpp"
#include "perception_common/data_type/object/base_object.hpp"

namespace senseAD {
namespace perception {

struct StaticObstacle : public BaseObject {
    StaticObstacle() {}
    explicit StaticObstacle(BaseObject* object) : BaseObject() {}
    StaticObstacle* GetThisPtr() override { return this; }

    // Bbox
    CameraBBox2DInfo camera_bbox_info;

    bool is_right_edge_detected = false;
    Eigen::Vector4i right_edge_line;
};

}  // namespace perception
}  // namespace senseAD
