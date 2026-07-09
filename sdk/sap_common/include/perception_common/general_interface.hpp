/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <memory>
#include <vector>
#include <string>

#include "perception_common/adapter/hdmap_adapter/hdmap_adapter.hpp"
#include "perception_common/adapter/localization_adapter/localization_querier.hpp"
#include "perception_common/adapter/localization_adapter/ved_localization_querier.hpp"
#include "perception_common/adapter/ego_car_info_adapter.hpp"
#include "perception_common/log/error.hpp"

#ifndef WITH_MDC
#include "ad_scm/ad_scm.hpp"
#endif

namespace senseAD {
namespace perception {

class GeneralInterface {
 public:
    GeneralInterface();
    ~GeneralInterface();
    apStatus_t Init();

 public:  // input
    // @brief input frame
    apStatus_t CarPoseDataList(const std::vector<uint64_t>& timestamp_ns,
                               std::vector<CarPoseExt>* car_pose_list);

    apStatus_t CarVelocityDataList(const std::vector<uint64_t>& timestamp_ns,
                                   std::vector<VelocityExt>* car_velocity_list);
    apStatus_t CarAngularSpeedDataList(
        const std::vector<uint64_t>& timestamp_ns,
        std::vector<AngularSpeedExt>* angular_speed_list);

    apStatus_t CarAccelerationDataList(
        const std::vector<uint64_t>& timestamp_ns,
        std::vector<AccelerationExt>* acceleration_list);

    apStatus_t SetLocalizationInfo(const uint64_t timestamp_ns,
                                   const OdometryInfo& info,
                                   const LMSFStatus status);
    apStatus_t SetVedLocalizationInfo(const uint64_t timestamp_ns,
                                      const OdometryInfo& info,
                                      const LMSFStatus status);
    apStatus_t SetLocalLocalizationInfo(
        const uint64_t timestamp_ns,
        const LocalLocalizationInfo& local_localization);

    apStatus_t GetFreespace(const std::vector<uint64_t>& timestamp_ns_vec,
                            std::vector<FreespaceInfo>* freespace_info_vec);

    apStatus_t SetFreespace(
        const std::vector<uint64_t>& timestamp_ns_vec,
        const std::vector<FreespaceInfo>& freespace_info_vec);

    apStatus_t SetMapTrafficLightFrame(
        const std::vector<uint64_t>& timestamp_ns_vec,
        const std::vector<MapTrafficLightFrame>& map_tl_info_vec);

    apStatus_t SetMapEgoCarDis2Stopline(
        const std::vector<uint64_t>& timestamp_ns_vec,
        const double& ego_car_dis2stopline);

    apStatus_t GetRelativeCarTrans(const uint64_t& from_time,
                                   const uint64_t& to_time,
                                   Eigen::Matrix4f* transform_ptr);
    apStatus_t SetFusSlotPos(const uint64_t timestamp_ns,
                             const LocalLocalizationInfo& local_localization);
};

}  // namespace perception
}  // namespace senseAD
