/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * Zhang Shiquan <zhangshiquan@sensetime.com>
 */
#pragma once

#include <vector>
#include <string>
#include <memory>

#include "perception_common/base/class_helper.hpp"
#include "perception_common/log/error.hpp"

namespace senseAD {
namespace perception {

class ConnectedComponent {
 public:
    ConnectedComponent() = default;
    ~ConnectedComponent() = default;

    apStatus_t ClearAll();
    apStatus_t ClearConnections();
    apStatus_t SetSize(const size_t element_size);
    size_t GetSize() const { return connected_relation_.size(); }

    apStatus_t AddConnection(const size_t query_index, const size_t dst_index);

    apStatus_t GetConnectedComponentAll(
        std::vector<std::vector<size_t>>* connected_relation);
    apStatus_t GetConnectedRelationByIndex(
        const size_t query_index, std::vector<size_t>* connected_relation);

    apStatus_t GetConnectIndexList(std::vector<size_t>* index_list);
    apStatus_t GetUnConnectIndexList(std::vector<size_t>* index_list);

 private:
    std::vector<std::vector<size_t>> connected_relation_;
};  // class ConnectedComponent

typedef std::shared_ptr<ConnectedComponent> ConnectedComponentPtr;
typedef std::shared_ptr<const ConnectedComponent> ConnectedComponentConstPtr;

class BipartiteGraph {
 public:
    BipartiteGraph()
        : is_initialized_(false),
          is_solved_(false),
          valid_value_threshold_(0),
          src_element_size_(0),
          dst_element_size_(0),
          src_connected_component_ptr_(std::make_shared<ConnectedComponent>()),
          dst_connected_component_ptr_(std::make_shared<ConnectedComponent>()) {
    }

    ~BipartiteGraph() = default;

    bool IsInitialized() const { return is_initialized_; }
    bool IsSolved() const { return is_solved_; }
    void SetSolved() { is_solved_ = true; }
    apStatus_t Reset(const size_t src_element_size,
                     const size_t dst_element_size);
    void SetValidValueThreshold(const float valid_value_threshold) {
        valid_value_threshold_ = valid_value_threshold;
    }
    float GetValidValueThreshold() const { return valid_value_threshold_; }

    apStatus_t ClearAll();

    apStatus_t SetGraphValue(const size_t src_element_index,
                             const size_t dst_element_index,
                             const float value);

    apStatus_t GetGraphValue(const size_t src_element_index,
                             const size_t dst_element_index,
                             float* value) const;
    size_t GetSrcSize() const;
    size_t GetDstSize() const;

    apStatus_t AddConnection(const size_t src_index, const size_t dst_index);
    apStatus_t ClearSolvedResults();

    const ConnectedComponentPtr GetSrcConnectivity() const;
    const ConnectedComponentPtr GetDstConnectivity() const;

 private:
    bool is_initialized_;
    bool is_solved_;
    float valid_value_threshold_;
    size_t src_element_size_;
    size_t dst_element_size_;
    ConnectedComponentPtr src_connected_component_ptr_;
    ConnectedComponentPtr dst_connected_component_ptr_;
    std::vector<std::vector<float>> graph_;
    std::mutex set_graph_value_mutex_;
};  // class BipartiteGraph

typedef std::shared_ptr<BipartiteGraph> BipartiteGraphPtr;
typedef std::shared_ptr<const BipartiteGraph> BipartiteGraphConstPtr;

}  // namespace perception
}  // namespace senseAD
