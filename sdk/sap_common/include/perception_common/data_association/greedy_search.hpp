/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * Xu Jingwei <xujingwei@senseauto.com>
 */

#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <algorithm>
#include <limits>

#include "perception_common/log/error.hpp"

namespace senseAD {
namespace perception {

class GreedyMatch {
 public:
    explicit GreedyMatch(int32_t capacity = 1) : capacity_(capacity) {}
    ~GreedyMatch() {}

    apStatus_t Process();
    apStatus_t SetGraphSize(const uint32_t& num_x, const uint32_t& num_y);
    apStatus_t SetGraphValue(const uint32_t& x,
                             const uint32_t& y,
                             const float& weight);
    const std::vector<std::vector<float>> GetGraph();
    const std::unordered_map<int32_t, int32_t> GetMatch();
    apStatus_t SetCapacity(const int32_t& capacity);
    apStatus_t GetCapacity(int32_t* cap);
    apStatus_t SetAffinityThresh(const float& thresh);
    const std::vector<int32_t> GetUnmatchedSrc();
    const std::vector<int32_t> GetUnmatchedDst();
    const int32_t GetMatchedNum();
    apStatus_t SolveGraph();

 private:
    apStatus_t GetMinValueIndex(std::vector<int32_t>* loc);
    apStatus_t SetUnmatchedIndexesVisited(const std::vector<int32_t>& indexes);

    std::vector<std::vector<float>> graph_;
    std::vector<std::vector<bool>> graph_index_visited_;
    std::unordered_map<int32_t, int32_t> match_;
    std::vector<int32_t> unmatched_src_indexes_;
    std::vector<int32_t> unmatched_dst_indexes_;
    int32_t num_x_;
    int32_t num_y_;
    int32_t capacity_;

    float affinity_thresh_ = 5.0f;
    float ignore_value_ = 1e8;
};

typedef std::shared_ptr<GreedyMatch> GreedyMatchPtr;
typedef std::shared_ptr<const GreedyMatch> GreedyMatchConstPtr;

}  // namespace perception
}  // namespace senseAD
