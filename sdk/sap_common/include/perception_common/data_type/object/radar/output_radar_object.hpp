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

// @brief output interface
struct OutputRadarObject : public BaseObject {
    OutputRadarObject() {}
    explicit OutputRadarObject(BaseObject* object) : BaseObject() {}
    OutputRadarObject* GetThisPtr() override { return this; }
    /**!
     * @brief the prob of object being of large height that can be ignorable.
     */
    float underpass_prob = 0.0f;
    float track_range = -1.;
    float track_range_rate = -1.;
    float track_range_accel_rate = -1.;
    float track_lat_rate = -1.;
};
typedef std::shared_ptr<OutputRadarObject> OutputRadarObjectPtr;

}  // namespace perception
}  // namespace senseAD
