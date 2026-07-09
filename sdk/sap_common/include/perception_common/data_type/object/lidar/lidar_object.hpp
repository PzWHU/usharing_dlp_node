/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"

#include "perception_common/data_type/object/base_object.hpp"

namespace senseAD {
namespace perception {

struct LidarObject : public BaseObject {
    LidarObject() {}
    explicit LidarObject(BaseObject* object) : BaseObject(*object) {}
    LidarObject* GetThisPtr() override { return this; }
    uint64_t point_cloud_timestamp_ns = 0;
};

}  // namespace perception
}  // namespace senseAD
