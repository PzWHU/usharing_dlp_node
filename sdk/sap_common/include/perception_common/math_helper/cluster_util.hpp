/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Ma QuanMeng <maquanmeng@sensetime.com>
 */
#pragma once

#include <vector>
#include <memory>
#include <unordered_map>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "perception_common/log/error.hpp"
#include "perception_common/log/common_log.hpp"

namespace senseAD {
namespace perception {

class DBSCAN {
 public:
    DBSCAN(float min_adj_size,
           float x_thresh,
           float y_thresh,
           float mal_sum_thresh)
        : epsilon_(0.1),
          min_adj_size_(min_adj_size),
          x_thresh_(x_thresh),
          y_thresh_(y_thresh),
          mal_sum_thresh_(mal_sum_thresh) {}

    apStatus_t Init(const std::vector<Eigen::Vector3f>& points);

    apStatus_t Cluster(std::unordered_map<int, std::vector<int>>* clusters);

 private:
    apStatus_t DfsOfDBSCAN(int index, int cluster_id);

    float MalDistance(const Eigen::Vector3f& point1,
                      const Eigen::Vector3f& point2);

 private:
    float epsilon_ = 0.1;
    int min_adj_size_ = 1;
    float x_thresh_ = 6;
    float y_thresh_ = 2;
    float mal_sum_thresh_ = 8;
    const int NOISE = -2;
    const int NOT_CLASSIFIED = -1;
    std::vector<Eigen::Vector3f> cluster_points_;
    std::vector<int> cluster_adj_number_;
    std::vector<int> cluster_attr_;
    std::vector<std::vector<int>> adj_indeces_;
};

apStatus_t ClusterByDBSCAN(const std::vector<Eigen::Vector3f>& points,
                           float min_adj_size,
                           float x_thresh,
                           float y_thresh,
                           float mal_sum_thresh,
                           std::unordered_map<int, std::vector<int>>* clusters);

}  // namespace perception
}  // namespace senseAD
