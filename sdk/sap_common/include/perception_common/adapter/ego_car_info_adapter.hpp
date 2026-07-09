/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Zhang Shiquan <zhangshiquan@sensetime.com>
 * Luo Quan <luoquan@sensetime.com>
 * Zhao Deze <zhaodeze@senseauto.com>
 */
#pragma once

#include <mutex>
#include <map>
#include <vector>
#include <utility>

#include "eigen3/Eigen/Dense"

#include "perception_common/log/error.hpp"

#ifdef WITH_AD_COMMON
#include "perception_common/base/data_type/localization_info.hpp"
#include "perception_common/adapter/localization_adapter/localization_querier.hpp"
#include "perception_common/adapter/localization_adapter/ved_localization_querier.hpp"
#endif  // WITH_AD_COMMON

namespace senseAD {
namespace perception {

#ifdef WITH_AD_COMMON

class CarPoseQuerier {
 public:
    CarPoseQuerier();

    explicit CarPoseQuerier(float clear_old_than);

    static CarPoseQuerier &GetInstance();

    apStatus_t Init(float clear_old_than);

    apStatus_t InsertVelocity(const uint64_t &timestamp_ns,
                              const Eigen::Vector3f &velocity);

    apStatus_t InsertAcceleration(const uint64_t &timestamp_ns,
                                  const Eigen::Vector3f &acceleration);

    apStatus_t QuerierVelocity(const uint64_t &timestamp_ns,
                               Eigen::Vector3f *velocity);

    apStatus_t QuerierAcceleration(const uint64_t &timestamp_ns,
                                   Eigen::Vector3f *acceleration);

    apStatus_t InsertAngular(const uint64_t &timestamp_ns,
                             const Eigen::Vector3f &angular);

    apStatus_t QuerierAngular(const uint64_t &timestamp_ns,
                              Eigen::Vector3f *angular);

    apStatus_t InsertCarPose(const uint64_t &timestamp_ns,
                             const Eigen::Matrix4f &car_pose);

    apStatus_t QuerierCarPose(const uint64_t &timestamp_ns,
                              Eigen::Matrix4f *car_pose);

    apStatus_t InsertUnifiedEgoTransform(
        const uint64_t &timestamp_ns, const Eigen::Matrix4f &transform_matrix);

    apStatus_t QuerierUnifiedEgoTransform(const uint64_t &timestamp_ns,
                                          Eigen::Matrix4f *transform_matrix);

    apStatus_t ClearVelocityMapOldState(uint64_t time_gap);

    apStatus_t ClearAccelerationMapOldState(uint64_t time_gap);

    apStatus_t ClearAngularMapOldState(uint64_t time_gap);

    apStatus_t ClearCarPoseMapOldState(uint64_t time_gap);

    apStatus_t ClearUnifiedEgoTransformMapOldState(uint64_t time_gap);

    Eigen::Matrix4f YPRXY2Matrix(const float yaw,
                                 const float pitch,
                                 const float roll,
                                 const float px,
                                 const float py,
                                 const float pz);

 private:
    std::mutex timestamp_to_carpose_mutex_;
    std::mutex timestamp_to_carangular_mutex_;
    std::mutex timestamp_to_carvelocity_mutex_;
    std::mutex timestamp_to_caracceleration_mutex_;
    std::mutex timestamp_to_unified_t_mutex_;

 private:
    float clear_old_than_;
    uint64_t clear_old_than_ns_;
    uint64_t s2ns_;
    std::map<uint64_t, Eigen::Vector3f> timestamp_to_carvelocity_;
    std::map<uint64_t, Eigen::Vector3f> timestamp_to_caracceleration_;
    std::map<uint64_t, Eigen::Vector3f> timestamp_to_carangular_;
    std::map<uint64_t, Eigen::Matrix4f> timestamp_to_carpose_;
    std::map<uint64_t, Eigen::Matrix4f> timestamp_to_unified_t_;
};

void QUERY_RELATIVE_CAR_TRANSFORM(const uint64_t &from_time,
                                  const uint64_t &to_time,
                                  Eigen::Matrix4f *transform_ptr,
                                  bool *ret);

void QUERY_RELATIVE_CAR_TRANSFORM_BY_VED(const uint64_t &from_time,
                                         const uint64_t &to_time,
                                         Eigen::Matrix4f *transform_ptr,
                                         bool *ret);

void QUERY_CURRENT_CAR_VELOCITY(const uint64_t &time_stamp,
                                Eigen::Vector3f *linear_velocity_ptr,
                                bool *ret);

void QUERY_CURRENT_CAR_VELOCITY_BY_VED(const uint64_t &time_stamp,
                                       Eigen::Vector3f *linear_velocity_ptr,
                                       bool *ret);

void QUERY_CURRENT_CAR_ACCELERATION(const uint64_t &time_stamp,
                                    Eigen::Vector3f *linear_acceleration_ptr,
                                    bool *ret);

void QUERY_CURRENT_CAR_ACCELERATION_BY_VED(
    const uint64_t &time_stamp,
    Eigen::Vector3f *linear_acceleration_ptr,
    bool *ret);

void QUERY_CURRENT_CAR_ANGULAR_VELOCITY(const uint64_t &time_stamp,
                                        Eigen::Vector3f *angular_velocity_ptr,
                                        bool *ret);

void QUERY_CURRENT_CAR_ANGULAR_VELOCITY_BY_VED(
    const uint64_t &time_stamp,
    Eigen::Vector3f *angular_velocity_ptr,
    bool *ret);

void QUERY_CURRENT_CAR_POSE(const uint64_t &time_stamp,
                            Eigen::Matrix3f *R_mat_ptr,
                            Eigen::Vector2f *T_mat_ptr,
                            bool *ret);

void QUERY_CURRENT_CAR_POSE_BY_VED(const uint64_t &time_stamp,
                                   Eigen::Matrix3f *R_mat_ptr,
                                   Eigen::Vector2f *T_mat_ptr,
                                   bool *ret);

void QUERY_CURRENT_CAR_POSE_BY_VED(const uint64_t &time_stamp,
                                   Eigen::Matrix3f *R_mat_ptr,
                                   Eigen::Vector3f *T_mat_ptr,
                                   bool *ret);

void QUERY_UNIFIED_EGO_TRANS_MATRIX(const uint64_t &timestamp_ns,
                                    Eigen::Matrix4f *transform_matrix_ptr,
                                    bool *ret);

#define GET_EGO_VEHICLE_INFO(timestamp_ns, ego_vehicle_info,                   \
                             current_plane_pitch)                              \
    {                                                                          \
        AP_LDEBUG(GET_EGO_VEHICLE_INFO)                                        \
            << "Start get vehicle info from ved_localization_querier! ";       \
        ego_vehicle_info->timestamp_ns = timestamp_ns;                         \
        std::vector<uint64_t> time_ns{timestamp_ns};                           \
        std::vector<AngularSpeedExt> angular_velocity_vector;                  \
        auto flag_g = g_localization_querier.AnguarSpeedQuery(                 \
            time_ns, &angular_velocity_vector);                                \
        if (AP_SUCCESS != flag_g || 1 != angular_velocity_vector.size() ||     \
            senseAD::perception::LocalizationDataStatus::INVALID ==            \
                angular_velocity_vector[0].status) {                           \
            angular_velocity_vector.clear();                                   \
            ved_localization_querier.AnguarSpeedQuery(                         \
                time_ns, &angular_velocity_vector);                            \
        }                                                                      \
        std::vector<VelocityExt> linear_velocity_vector;                       \
        flag_g = g_localization_querier.VelocityQuery(                         \
            time_ns, &linear_velocity_vector);                                 \
        if (AP_SUCCESS != flag_g || 1 != linear_velocity_vector.size() ||      \
            senseAD::perception::LocalizationDataStatus::INVALID ==            \
                linear_velocity_vector[0].status) {                            \
            linear_velocity_vector.clear();                                    \
            ved_localization_querier.VelocityQuery(time_ns,                    \
                                                   &linear_velocity_vector);   \
        }                                                                      \
        std::vector<AccelerationExt> linear_acceleration_vector;               \
        flag_g = g_localization_querier.AccelerationQuery(                     \
            time_ns, &linear_acceleration_vector);                             \
        if (AP_SUCCESS != flag_g || 1 != linear_acceleration_vector.size() ||  \
            senseAD::perception::LocalizationDataStatus::INVALID ==            \
                linear_acceleration_vector[0].status) {                        \
            linear_acceleration_vector.clear();                                \
            ved_localization_querier.AccelerationQuery(                        \
                time_ns, &linear_acceleration_vector);                         \
        }                                                                      \
        std::vector<CarPoseExt> car_pose_vector;                               \
        flag_g =                                                               \
            g_localization_querier.CarPoseDataList(time_ns, &car_pose_vector); \
        if (AP_SUCCESS != flag_g || 1 != car_pose_vector.size() ||             \
            senseAD::perception::LocalizationDataStatus::INVALID ==            \
                car_pose_vector[0].status) {                                   \
            car_pose_vector.clear();                                           \
            ved_localization_querier.CarPoseDataList(time_ns,                  \
                                                     &car_pose_vector);        \
        }                                                                      \
        if (angular_velocity_vector.size() != 1 ||                             \
            linear_velocity_vector.size() != 1 ||                              \
            linear_acceleration_vector.size() != 1 ||                          \
            car_pose_vector.size() != 1) {                                     \
            AP_LWARN(GET_EGO_VEHICLE_INFO)                                     \
                << "g_localization_querier get invalid size!";                 \
            return AP_SUCCESS;                                                 \
        }                                                                      \
        ego_vehicle_info->angular_speed                                        \
            << angular_velocity_vector.at(0).angular_speed.x,                  \
            angular_velocity_vector.at(0).angular_speed.y,                     \
            angular_velocity_vector.at(0).angular_speed.z;                     \
        ego_vehicle_info->velocity << linear_velocity_vector.at(0).velocity.x, \
            linear_velocity_vector.at(0).velocity.y,                           \
            linear_velocity_vector.at(0).velocity.z;                           \
        ego_vehicle_info->acceleration                                         \
            << linear_acceleration_vector.at(0).acceleration.x,                \
            linear_acceleration_vector.at(0).acceleration.y,                   \
            linear_acceleration_vector.at(0).acceleration.z;                   \
        ego_vehicle_info->angular                                              \
            << car_pose_vector.at(0).car_pose.attitude.pitch -                 \
                   current_plane_pitch,                                        \
            car_pose_vector.at(0).car_pose.attitude.roll,                      \
            car_pose_vector.at(0).car_pose.attitude.yaw;                       \
        ego_vehicle_info->is_velocity_valid = true;                            \
        ego_vehicle_info->is_acceleration_valid = true;                        \
        ego_vehicle_info->is_angular_speed_valid = true;                       \
        ego_vehicle_info->is_angular_valid = true;                             \
        AP_LDEBUG(GET_EGO_VEHICLE_INFO)                                        \
            << "End get speed from g_localization_querier! ";                  \
    }

#endif  // WITH_AD_COMMON

#ifdef BUILD_ADAS

class CarPoseQuerier {
 public:
    static CarPoseQuerier &GetInstance();

    CarPoseQuerier() : s2ns_(1e9), min_gap_(0.1), clear_old_than_(10);

    explicit CarPoseQuerier(float clear_old_than, double min_gap);

    apStatus_t Init(float clear_old_than, double min_gap = 0.1);

    apStatus_t InsertVehicleInfo(
        const uint64_t timestamp_ns const Eigen::Vector3f &velocity,
        const Eigen::Vector3f &angular);

    apStatus_t InsertAngular(
        const uint64_t timestamp_ns const Eigen::Vector3f &angular);

    apStatus_t GetAngularFromWindow(
        const uint64_t timestamp Eigen::Vector3f *angular);

    apStatus_t InsertVelocity(
        const uint64_t timestamp_ns const Eigen::Vector3f &velocity);

    apStatus_t GetVelocityFromWindow(
        const uint64_t timestamp Eigen::Vector3f *velocity);

    apStatus_t InsertCarPose(
        const uint64_t timestamp_ns const Eigen::Matrix4f &car_pose);

    apStatus_t GetCarPoseFromWindow(
        const uint64_t timestamp Eigen::Matrix4f *pose);

 private:
    apStatus_t ExtrapolateCarpose(const Eigen::Matrix4f &pose0;
                                  const uint64_t timestamp0,
                                  const Eigen::Matrix4f &pose1,
                                  const uint64_t timestamp1,
                                  const uint64_t timestamp,
                                  Eigen::Matrix4f *out_pose);

    apStatus_t InterpolateCarpose(const Eigen::Matrix4f &pose0 const uint64_t
                                      timestamp0,
                                  const Eigen::Matrix4f &pose1,
                                  const uint64_t timestamp1,
                                  const uint64_t timestamp,
                                  Eigen::Matrix4f *out_pose);

    apStatus_t IntegralCarPose(const uint64_t timestamp Eigen::Matrix4f *pose);

    apStatus_t ClearVelocityMapOldState(uint64_t time_gap);

    apStatus_t ClearAngularMapOldState(uint64_t time_gap);

    apStatus_t ClearCarPoseMapOldState(uint64_t time_gap);

    std::mutex carpose_window_mutex_;
    std::mutex angular_window_mutex_;
    std::mutex velocity_window_mutex_;

    uint64_t s2ns_;
    double min_gap_;
    float clear_old_than_;
    uint64_t clear_old_than_ns_;

    std::map<uint64_t, Eigen::Vector3f> velocity_window_;
    std::map<uint64_t, Eigen::Vector3f> angular_window_;
    std::map<uint64_t, Eigen::Matrix4f> carpose_window_;
};

#define GET_EGO_VEHICLE_INFO(timestamp_ns, ego_vehicle_info, \
                             current_plane_pitch)            \
    { AP_LDEBUG(GET_EGO_POSE_TRANSFORM) << "Not implemented!"; }

void QUERY_RELATIVE_CAR_TRANSFORM(const uint64_t &from_time,
                                  const uint64_t &to_time,
                                  Eigen::Matrix4f *transform_ptr,
                                  bool *ret);

void QUERY_CURRENT_CAR_VELOCITY(const uint64_t &time_stamp,
                                Eigen::Vector3f *linear_velocity_ptr,
                                bool *ret);

void QUERY_CURRENT_CAR_ANGULAR_VELOCITY(const uint64_t &time_stamp,
                                        Eigen::Vector3f *angular_velocity_ptr,
                                        bool *ret);

#endif  // BUILD_ADAS

}  // namespace perception
}  // namespace senseAD
