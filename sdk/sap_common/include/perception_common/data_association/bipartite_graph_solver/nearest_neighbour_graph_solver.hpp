/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Zhang Shiquan <zhangshiquan@senseauto.com>
 */

#pragma once

#include <string>
#include <memory>

#include "perception_common/data_association/bipartite_graph_solver.hpp"

namespace senseAD {
namespace perception {
/**
 * @brief provide a match method to associate an element in src to several
 * (neighbours) elements in dst
 * @param nearest_neighbors_num neighbours num
 * @param neighbor_max_distance_meter the max distance set for valid association
 */
class NearestNeighbourGraphSolver : public BipartiteGraphSolver {
 public:
    NearestNeighbourGraphSolver()
        : nearest_neighbors_num_(1), neighbor_max_distance_meter_(0.5) {}
    virtual ~NearestNeighbourGraphSolver() = default;

    apStatus_t Init(const GraphSolverParameter& params) final;

    apStatus_t Process(BipartiteGraph* graph) final;

    std::string Name() final { return "NearestNeighbourGraphSolver"; }

 private:
    GraphSolverParameter param_;
    uint32_t nearest_neighbors_num_;
    float neighbor_max_distance_meter_;

    DISABLE_COPY_AND_ASSIGN(NearestNeighbourGraphSolver);
};  // class NearestNeighbourGraphSolver

}  // namespace perception
}  // namespace senseAD
