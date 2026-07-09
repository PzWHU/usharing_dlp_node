/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * Tong yuanyang <tongyuanyang@sensetime.com>
 * He Zheqi <hezheqi@sensetime.com>
 * Luo Quan <luoquan@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>

#include "perception_common/log/error.hpp"

namespace senseAD {
namespace perception {

struct Match {
    int32_t cnt;
    std::vector<int32_t> match;

    explicit Match(int32_t cap) {
        match.resize(cap, -1);
        cnt = 0;
    }
};

class KuhnMunkresMatch {
 public:
    explicit KuhnMunkresMatch(int32_t capacity = 1) : capacity_(capacity) {}
    ~KuhnMunkresMatch() {}

    apStatus_t Process();
    apStatus_t SetGraphSize(const uint32_t& num_x, const uint32_t& num_y);
    apStatus_t SetGraphValue(const uint32_t& x,
                             const uint32_t& y,
                             const float& weight);
    apStatus_t SetCapacity(const int32_t& capacity);
    apStatus_t GetCapacity(int32_t* cap);
    const std::vector<std::vector<float>> GetGraph();
    const std::vector<Match> GetMatch();
    const int32_t GetMatchedNum();
    apStatus_t SolveGraph();

 private:
    std::vector<float> label_x_;
    std::vector<float> label_y_;
    std::vector<float> slack_;
    std::vector<std::vector<float>> graph_;
    std::vector<Match> match_;
    std::vector<bool> visx_;
    std::vector<bool> visy_;
    int32_t num_x_;
    int32_t num_y_;
    int32_t capacity_;

    bool FindPath(int32_t x);
    float FindPath(int32_t x, bool* bFound);
};

typedef std::shared_ptr<KuhnMunkresMatch> KuhnMunkresMatchPtr;
typedef std::shared_ptr<const KuhnMunkresMatch> KuhnMunkresMatchConstPtr;

}  // namespace perception
}  // namespace senseAD
