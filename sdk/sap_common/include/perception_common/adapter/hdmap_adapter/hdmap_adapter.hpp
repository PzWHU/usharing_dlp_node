/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Zhangshiquan <zhangshiquan@sensetime.com>
 */
#pragma once

#ifndef BUILD_ADAS

#include <mutex>
#include <memory>
#include <map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Dense"

#include <opencv2/opencv.hpp>
#include "perception_common/log/error.hpp"
#include "perception_common/base/class_helper.hpp"
#include "perception_common/base/data_type/freespace_info.hpp"
#include "perception_common/base/data_type/traffic_light_info.hpp"

namespace senseAD {
namespace perception {

class HDMapAdapter {
 public:
    HDMapAdapter() = default;
    ~HDMapAdapter() = default;

    apStatus_t SetMapTrafficLightInfo(const uint64_t timestamp_ns,
                                      const MapTrafficLightFrame& map_tl_frame);

    apStatus_t SetMapEgoCarDis2StoplineInfo(const uint64_t timestamp_ns,
                                            const double& ego_car_dis2stopline);

    apStatus_t GetMapTrafficLightInfo(const uint64_t timestamp_ns,
                                      MapTrafficLightFrame* map_tl_frame);

    apStatus_t GetMapEgoCarDis2StoplineInfo(const uint64_t timestamp_ns,
                                            double* ego_car_dis2stopline);

    apStatus_t SetFreespace(const uint64_t timestamp_ns,
                            const StructFreespaceInfo& freespace_info);

    apStatus_t GetFreespace(const uint64_t timestamp_ns,
                            StructFreespaceInfo* freespace_info);

    apStatus_t GetFreespace(const std::vector<uint64_t>& timestamp_ns_vec,
                            std::vector<FreespaceInfo>* freespace_info_vec);

    apStatus_t SetFreespace(
        const std::vector<uint64_t>& timestamp_ns_vec,
        const std::vector<FreespaceInfo>& freespace_info_vec);

    apStatus_t ClearFreespace();

    static HDMapAdapter& Instance();

 private:
    std::mutex freespace_info_mutex_;
    std::mutex map_tl_frame_mutex_;

 private:
    std::map<uint64_t, FreespaceInfo> timestamp_ns_to_freespace_;
    std::map<uint64_t, MapTrafficLightFrame> timestamp_ns_to_tl_frame_;
    std::map<uint64_t, double> timestamp_ns_to_ego_car_dis2stopline_;

    DISABLE_COPY_AND_ASSIGN(HDMapAdapter);
};

extern HDMapAdapter& g_hdmap_adapter;

}  // namespace perception
}  // namespace senseAD

#endif  // BUILD_ADAS
