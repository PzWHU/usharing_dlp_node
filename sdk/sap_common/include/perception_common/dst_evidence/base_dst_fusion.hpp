/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Luo Quan <luoquan@sensetime.com>
 */

#pragma once

#include <set>
#include <memory>
#include <vector>
#include <iomanip>
#include <iostream>
#include <unordered_map>

#include "perception_common/log/error.hpp"
#include "perception_common/base/data_type/dst_evidence.hpp"

namespace senseAD {
namespace perception {

class Evidence;

class DSTFusion {
 public:
    apStatus_t Init(const std::vector<uint64_t>& type_set,
                    const std::vector<float>& masses,
                    const uint64_t timestamp);

    // All the input mass should be less than 1.0
    apStatus_t UpdateWithMeasurement(const std::vector<uint64_t>& types,
                                     const std::vector<float>& masses,
                                     const uint64_t timestamp);
    virtual apStatus_t UpdateWithoutMeasurement(const uint64_t timestamp);

    apStatus_t GetTypeMass(const uint64_t type, float* mass);
    apStatus_t GetTypePlausability(const uint64_t type, float* plausability);
    apStatus_t GetTypeBelief(const uint64_t type, float* belief);
    apStatus_t GetPredictedType(uint64_t* type);
    apStatus_t PrintEvidence();

    DSTFusion();
    virtual ~DSTFusion();

 protected:
    std::unique_ptr<Evidence> state_;
    std::shared_ptr<Evidence> prototype_;
    std::vector<uint64_t> typeset_;
    uint64_t last_timestamp_;
    bool initialized_;
};

}  // namespace perception
}  // namespace senseAD
