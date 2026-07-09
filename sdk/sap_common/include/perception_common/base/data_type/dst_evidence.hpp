/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Luo Quan <luoquan@sensetime.com>
 */

#pragma once

#include <set>
#include <vector>
#include <memory>
#include <iomanip>
#include <iostream>
#include <unordered_map>

#include "perception_common/log/error.hpp"

namespace senseAD {
namespace perception {

struct DSTData {
    float mass;
    float plausability;
    float belief;
    uint64_t exist_type;

    DSTData() : mass(0.0),
                plausability(-1.0),
                belief(-1.0) {}

    explicit DSTData(uint64_t type) : mass(0.0),
                                      plausability(-1.0),
                                      belief(-1.0) {
        exist_type = type;
    }
};

typedef std::shared_ptr<DSTData> DSTDataPtr;
typedef std::shared_ptr<const DSTData> DSTDataConstPtr;

class Evidence {
 public:
    apStatus_t Init(std::vector<uint64_t> type_set);
    std::shared_ptr<Evidence> Clone() {
        return std::make_shared<Evidence>(*this);
    }

    apStatus_t SetMass(const uint64_t type, const float mass);
    apStatus_t SetMass(const std::vector<uint64_t>& types,
                       const std::vector<float>& masses);

    apStatus_t GetMass(const uint64_t type, float* mass);
    apStatus_t GetPlausability(const uint64_t type, float* plausability);
    apStatus_t GetBelief(const uint64_t type, float* belief);
    apStatus_t GetBestType(uint64_t *type);

    apStatus_t FuseEvidence(const Evidence& evidence);
    apStatus_t Reset();
    apStatus_t printEvidence();
    apStatus_t Check() { return this->SelfCheck(); }

    Evidence operator&(const Evidence& other);

 private:
    apStatus_t SetDST(
        const std::unordered_map<uint64_t, DSTDataPtr>& dst_set);
    apStatus_t SelfCheck();
    float GetPlausability(const uint64_t type);
    float GetBelief(const uint64_t type);

    std::set<uint64_t> type_set_;
    std::unordered_map<uint64_t, DSTDataPtr> dst_map_;
};

typedef std::shared_ptr<Evidence> EvidencePtr;
typedef std::shared_ptr<const Evidence> EvidenceConstPtr;

}  // namespace perception
}  // namespace senseAD
