/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Ma Quanmeng <maquanmeng@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>
#include <string>

#include "eigen3/Eigen/Dense"
#ifdef WITH_PCL
#include "pcl/point_cloud.h"
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/console/print.h"
#else
#include "perception_common/math_helper/nn_serch_util.hpp"  // NOLINT
#endif

namespace senseAD {
namespace perception {
// base object
typedef struct StructDecisionTreeNode {
    StructDecisionTreeNode(const int32_t& node_id_in,
                           const int32_t left_child_in,
                           const int32_t right_child_in,
                           const uint32_t feature_index_in,
                           const float threshold_in,
                           const std::string& name_in)
        : node_id(node_id_in),
          left_child(left_child_in),
          right_child(right_child_in),
          feature_index(feature_index_in),
          threshold(threshold_in),
          feature_name(name_in),
          kd_tree(nullptr) {}
    StructDecisionTreeNode()
        : node_id(-1),
          left_child(-1),
          right_child(-1),
          feature_index(-1),
          threshold(0),
          kd_tree(nullptr) {}
    int32_t node_id;
    int32_t left_child;
    int32_t right_child;
    uint32_t feature_index;
    float threshold;
    std::string feature_name;
    std::vector<Eigen::MatrixXf> covariances;
    std::vector<Eigen::VectorXf> means;
#ifdef WITH_PCL
    pcl::KdTreeFLANN<pcl::PointXY>::Ptr kd_tree;
#else
    std::shared_ptr<nanoflann::KdTreeFLANNFloatDim2> kd_tree;
#endif
} DecisionTreeNode;

constexpr int32_t TREE_LEAF = -1;

}  // namespace perception
}  // namespace senseAD
