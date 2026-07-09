/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Xu Jingwei <xujingwei@senseauto.com>
 */

#pragma once

#include <iostream>
#include <memory>
#include <string>
#include <algorithm>
#include <vector>
#include <utility>

#include "perception_common/data_association/bipartite_graph_solver.hpp"
#include "perception_common/data_association/greedy_search.hpp"

namespace senseAD {
namespace perception {

class GreedyBipartiteGraphSolver : public BipartiteGraphSolver {
 public:
    GreedyBipartiteGraphSolver()
        : greedy_match_algorithm_ptr_(std::make_shared<GreedyMatch>()),
          is_set_capacity_(false) {}
    virtual ~GreedyBipartiteGraphSolver() = default;

    apStatus_t Init(const GraphSolverParameter& params) final;

    apStatus_t Process(BipartiteGraph* graph) final;

    std::string Name() final { return "GreedyBipartiteGraphSolver"; }

 private:
    apStatus_t SetMatchCapacity(const int32_t& capacity) {
        is_set_capacity_ = true;
        return this->greedy_match_algorithm_ptr_->SetCapacity(capacity);
    }

 private:
    GreedyMatchPtr greedy_match_algorithm_ptr_;
    GraphSolverParameter param_;
    bool is_set_capacity_;
    float ignore_value_ = 1e8;
    DISABLE_COPY_AND_ASSIGN(GreedyBipartiteGraphSolver);
};  // class GreedyBipartiteGraphSolver

}  // namespace perception
}  // namespace senseAD
