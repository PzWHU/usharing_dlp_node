/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Luo Quan <luoquan@sensetime.com>
 */

#pragma once

#include <string>
#include <memory>

#include "perception_common/data_association/bipartite_graph_solver.hpp"
#include "perception_common/data_association/kuhn_munkres.hpp"

namespace senseAD {
namespace perception {

class KMBipartiteGraphSolver : public BipartiteGraphSolver {
 public:
    KMBipartiteGraphSolver()
        : km_algorithm_ptr_(std::make_shared<KuhnMunkresMatch>()),
          is_set_capacity_(false) {}
    virtual ~KMBipartiteGraphSolver() = default;

    apStatus_t Init(const GraphSolverParameter& params) final;

    apStatus_t Process(BipartiteGraph* graph) final;

    std::string Name() final { return "KMBipartiteGraphSolver"; }

 private:
    apStatus_t SetMatchCapacity(const int32_t& capacity) {
        is_set_capacity_ = true;
        return this->km_algorithm_ptr_->SetCapacity(capacity);
    }

 private:
    KuhnMunkresMatchPtr km_algorithm_ptr_;
    GraphSolverParameter param_;
    bool is_set_capacity_;
    DISABLE_COPY_AND_ASSIGN(KMBipartiteGraphSolver);
};  // class KMBipartiteGraphSolver

}  // namespace perception
}  // namespace senseAD
