/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"

#include "perception_common/data_type/object/camera/base_camera_attribute.hpp"
#include "perception_common/data_type/object/base_object.hpp"

namespace senseAD {
namespace perception {

struct BoardPole : public BaseObject {
    BoardPole() {}
    explicit BoardPole(BaseObject* object) : BaseObject() {}
    BoardPole* GetThisPtr() override { return this; }

    // Bbox
    CameraBBox2DInfo camera_bbox_info;
};

}  // namespace perception
}  // namespace senseAD
