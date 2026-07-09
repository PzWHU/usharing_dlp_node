/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Zhang Shiquan <zhangshiquan@sensetime.com>
 */

#pragma once

#include <memory>
#include <vector>
#include <string>

#include "perception_common/base/class_helper.hpp"
#include "perception_common/log/error.hpp"
#include "perception_common/base/factory.hpp"
#include "proto/bipartite_graph_solver.pb.h"  // NOLINT

#include "perception_common/base/data_type/match_data.hpp"

namespace senseAD {
namespace perception {

class BipartiteGraphSolver {
 public:
    BipartiteGraphSolver() = default;
    virtual ~BipartiteGraphSolver() = default;

    virtual apStatus_t Init(const GraphSolverParameter& params) = 0;

    virtual apStatus_t Process(BipartiteGraph* graph) = 0;

    virtual std::string Name() = 0;

 private:
    DISABLE_COPY_AND_ASSIGN(BipartiteGraphSolver);
};  // class BipartiteGraphSolver

REGISTER_FACTORY(Common, BipartiteGraphSolver);

#define REGISTER_BIPARTITE_GRAPH_SOLVER(name) \
    REGISTER_CLASS(Common, BipartiteGraphSolver, name)

}  // namespace perception
}  // namespace senseAD
