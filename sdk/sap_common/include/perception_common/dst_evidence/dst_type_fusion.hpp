/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Luo Quan <luoquan@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>

#include "perception_common/dst_evidence/base_dst_fusion.hpp"

namespace senseAD {
namespace perception {

class DSTTypeFusion : public DSTFusion {
 public:
    enum LabelType {
        VEHICLE = 1 << 0,
        VEHICLE_BUS = 1 << 1,
        PEDESTRIAN = 1 << 2,
        CYCLIST = 1 << 3,
        UNKNOWN = 1 << 0 | 1 << 1 | 1 << 2 | 1 << 3
    };
 public:
    explicit DSTTypeFusion(const std::vector<float>& masses,
                                 const uint64_t timestamp);
    explicit DSTTypeFusion(const float label_prob,
                                 const uint64_t timestamp);

    apStatus_t UpdateLabel(const float label_prob,
                           const uint64_t timestamp);

 private:
    static const std::vector<uint64_t> label_types_;
};

typedef std::shared_ptr<DSTTypeFusion> DSTTypeFusionPtr;

}  // namespace perception
}  // namespace senseAD
