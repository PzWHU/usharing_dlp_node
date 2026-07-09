/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Zhang Shiquan <zhangshiquan@sensetime.com>
 */

#pragma once

#include "perception_common/data_association/bipartite_graph_solver/km_bipartite_graph_solver.hpp"
#include "perception_common/data_association/bipartite_graph_solver/greedy_bipartite_graph_solver.hpp"
#include "perception_common/data_association/bipartite_graph_solver/nearest_neighbour_graph_solver.hpp"

namespace senseAD {
namespace perception {

REGISTER_BIPARTITE_GRAPH_SOLVER(KMBipartiteGraphSolver);
REGISTER_BIPARTITE_GRAPH_SOLVER(GreedyBipartiteGraphSolver);
REGISTER_BIPARTITE_GRAPH_SOLVER(NearestNeighbourGraphSolver);

}  // namespace perception
}  // namespace senseAD
