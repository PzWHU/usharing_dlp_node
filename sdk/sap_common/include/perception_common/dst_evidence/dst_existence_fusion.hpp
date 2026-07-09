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

class DSTExistenceFusion : public DSTFusion {
 public:
    enum ExistenceType {
        EXIST = 1 << 0,
        NOT_EXIST = 1 << 1,
        UNKNOWN = 1 << 0 | 1 << 1
    };

 public:
    explicit DSTExistenceFusion(const std::vector<float>& masses,
                                const uint64_t timestamp);
    explicit DSTExistenceFusion(const float existence,
                                const uint64_t timestamp);

    apStatus_t UpdateWithoutMeasurement(const uint64_t timestamp);
    apStatus_t UpdateExistence(const float existence, const uint64_t timestamp);

 private:
    static const std::vector<uint64_t> existence_types_;
};

typedef std::shared_ptr<DSTExistenceFusion> DSTExistenceFusionPtr;

}  // namespace perception
}  // namespace senseAD
