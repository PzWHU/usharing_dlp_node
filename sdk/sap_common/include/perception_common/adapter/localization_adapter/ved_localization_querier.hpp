/*
 * Copyright (C) 2019-20 by SenseTime Group Limited. All rights reserved.
 * Liang Yu <liangyu@sensetime.com>
 * Wang Xiaofeng<wangxiaofeng@sensetime.com>
 * Guo Zhichong <guozhichong@sensetime.com>
 */

#pragma once

#ifndef BUILD_ADAS

#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <functional>
#include <memory>
#include <thread>  // NOLINT
#include <mutex>   // NOLINT
#include <atomic>
#include <map>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "perception_common/log/error.hpp"
#include "perception_common/util/semaphore.hpp"
#include "perception_common/util/shared_memory.hpp"
#include "perception_common/util/type_traits.hpp"
#include "perception_common/util/inner_process_memory.hpp"
#include "perception_common/util/inner_process_binary_semaphore.hpp"
#include "perception_common/base/data_type/localization_info.hpp"
#include "perception_common/math_helper/timestamp_util.hpp"
#include "perception_common/math_helper/transform_util.hpp"
#include "perception_common/adapter/localization_adapter/localization_querier.hpp"

namespace senseAD {
namespace perception {

template <bool InterProcess = false>
class VedLocalizationQuerier {
 public:
    using Lock =
        typename type_traits::EnableIfElse<InterProcess,
                                           ecl::Semaphore,
                                           InnerProcessBinarySemaphore>::type;
    using Memory = typename type_traits::EnableIfElse<
        InterProcess,
        ecl::SharedMemory<LocalizationInfo>,
        InnerProcessMemory<LocalizationInfo>>::type;

    VedLocalizationQuerier() : inited_(false) {
        // log_.open("./ved_query.log", std::ios::out |
        // std::ios::trunc);
    }

    ~VedLocalizationQuerier() {
        if (inited_ && AUTO_UPDATE == mode_ && tid_.joinable()) {
            stop_autoupdate_mode_ = true;
            tid_.join();
        }
        // log_.close();
    }

    /*
     * @brief Singleton localization querier instance
     * @return Reference of singleton object of localization querier
     */
    static VedLocalizationQuerier& Instance();

    enum Mode {
        // auto update at 100hz. if process request > 100hz, please using this.
        AUTO_UPDATE,
        // request once every call api
        PASSIVE_UPDATE
    };

    /*
     * @brief Init shared memcpy and wait localization node start
     * @param topic_name[in] id
     * @param mode [in] AUTO_UPDATE/PASSIVE_UPDATE
     * @param time_out_s [in] init and wait wirte
     * @return error code if fail
     */
    apStatus_t Init(std::string topic_name,
                    Mode mode = PASSIVE_UPDATE,
                    uint32_t time_out_s = 60);
    apStatus_t Init(Mode mode = PASSIVE_UPDATE, uint32_t time_out_s = 60);
    /*
     * @brief add a incoming odometry to buffer
     * @param timestamp_ns
     * @param odometry
     * @return error code if fail
     */
    apStatus_t UpdateLocalizationInfo(const uint64_t timestamp_ns,
                                      const OdometryInfo& odometry,
                                      const LMSFStatus& localization_status);

    apStatus_t UpdateLocalLocalizationInfo(
        const uint64_t timestamp_ns,
        const LocalLocalizationInfo& local_localization);

    /*
     * @brief explicit to check localization service is available or not
     */
    bool IsInited() { return inited_; }

    /*
     * @brief for node reset
     */
    bool Reset();

    /*
     * @brief get latest localization info
     * @param odometry [out] latest odometry info. acc/linear/angular...
     * @param car_pose [out] latest car pose
     * @param timestamp_ns [out] latest timestamp
     * @param fusion_status [out] latest fusion_status
     * @return error code if fail
     */
    apStatus_t LatestInfo(OdometryInfo* odometry,
                          CarPose* car_pose = nullptr,
                          uint64_t* timestamp_ns = nullptr,
                          LMSFStatus* local_status = nullptr);

    /*
     * @brief get prediction pose ahead of window or latest pose
     * @param odometry [out] predicted or latest odometry info.
     * @param car_pose [out] predicted or latest car pose
     * @param timstamp_ns [in|out] latest timestamp
     * @param query_freq_hz [in] the localization message query frequency in hz
     * @return error code if fail
     */
    apStatus_t PredictLatestInfo(OdometryInfo* odometry,
                                 uint64_t* timestamp_ns,
                                 CarPose* car_pose = nullptr,
                                 uint64_t query_freq_hz = 50);

    /*
     * @brief get car pose list at timestamp_ns
     * @param timestamp_ns [in] latest carpose if timestamp_ns[i] == 0
     * @param car_pose_list [out] packet of queried car pose, if fail, flag
     * invalid
     * @return error code if fail
     */
    apStatus_t CarPoseDataList(const std::vector<uint64_t>& timestamp_ns,
                               std::vector<CarPoseExt>* car_pose_list);

    apStatus_t CarPoseData(const uint64_t& timestamp_ns,
                           CarPoseExt* car_pose_ext);

    apStatus_t CarPoseDataNearest(const uint64_t& timestamp_ns,
                                  CarPoseExt* car_pose_ext);
    /*
     * @brief query the designated timestamp velocity
     * @param timestamp_ns [in] latest velocity if timestamp_ns[i] == 0
     * @param velocity_vector [in] velocity vector, if fail, flag invalid
     * @return status if abnormal state
     */
    apStatus_t VelocityQuery(const std::vector<uint64_t>& timestamp_ns,
                             std::vector<VelocityExt>* velocity_vector);

    apStatus_t VelocityQuery(const uint64_t& timestamp_ns,
                             VelocityExt* velocity_ext);
    /*
     * @brief query the designated timestamp acceleration
     * @param timestamp_ns [in] latest acceleration if timestamp_ns[i] == 0
     * @param acceleration_vector [in] acceleration vector, if fail, flag
     * invalid
     * @return status if abnormal state
     */
    apStatus_t AccelerationQuery(
        const std::vector<uint64_t>& timestamp_ns,
        std::vector<AccelerationExt>* acceleration_vector);

    apStatus_t AccelerationQuery(const uint64_t& timestamp_ns,
                                 AccelerationExt* acceleration_ext);
    /*
     * @brief query the designated timestamp angular speed
     * @param timestamp_ns [in] latest angular speed if timestamp_ns[i] == 0
     * @param angular_speed_vector [out] angular speed vector, if fail, flag
     * invalid
     * @return error if abnormal state
     */
    apStatus_t AnguarSpeedQuery(
        const std::vector<uint64_t>& timestamp_ns,
        std::vector<AngularSpeedExt>* angular_speed_vector);

    apStatus_t AnguarSpeedQuery(const uint64_t& timestamp_ns,
                                AngularSpeedExt* angular_speed_ext);
    /*
     * @brief get the 3D transform from from_timestamp_ns to to_timestamp_ns
     * @param from_timestamp_ns [in] latest timestamp if == 0
     * @param to_timestamp_ns [in] latest timestamp if == 0
     * @param trans [out] 4*4 transform matrix
     * @return status if abnormal state
     */
    apStatus_t Warp3D(const uint64_t from_timestamp_ns,
                      const uint64_t to_timestamp_ns,
                      Eigen::Matrix4d* trans);

    /*
     * @brief get the 3D transform from base_time to Multiple Points
     * @param base_time [in] base timestamp, latest timestamp if == 0
     * @param timestamp_ns [in] multiple input timestamps, latest timestamp if
     * == 0
     * @param trans [out] a vector of 4*4 transform matrix, if failed, flag
     * invalid
     * @return status if abnormal state
     */
    apStatus_t Warp3DMP(const uint64_t base_time,
                        const std::vector<uint64_t>& timestamp_ns,
                        std::vector<Trans3DExt>* trans);

    /*
     * @brief get the 2D transform from from_timestamp_ns to to_timestamp_ns
     * @param from_timestamp_ns [in] latest timestamp if == 0
     * @param to_timestamp_ns [in] latest timestamp if == 0
     * @param trans_2d [out] 3*3 transform matrix
     * @return status if abnormal state
     */
    apStatus_t Warp2D(const uint64_t from_timestamp_ns,
                      const uint64_t to_timestamp_ns,
                      Eigen::Matrix3d* trans_2d);

    /*
     * @brief get the 2D transform from base_time to Multiple Points
     * @param base_time [in] base timestamp, latest timestamp if == 0
     * @param timestamp_ns [in] multiple input timestamps, latest timestamp if
     * == 0
     * @param trans_2d [out] a vector of 3*3 transform matrix, if failed, flag
     * invalid
     * @return status if abnormal state
     */
    apStatus_t Warp2DMP(const uint64_t base_time,
                        const std::vector<uint64_t>& timestamp_ns,
                        std::vector<Trans2DExt>* trans_2d);

 private:
    void TransferSE3ToCarPose(const CarPose& car_pose, SE3Pose* se3_pose);
    apStatus_t InsertSE3Pose(const uint64_t& timestamp_ns,
                             const SE3Pose& se3_pose);
    apStatus_t ClearOldStateInSE3PoseMap(uint64_t time_gap);
    apStatus_t InsertOdometry(const uint64_t& timestamp_ns,
                              const OdometryInfo& odom_info);
    apStatus_t ClearOldStateInOdomMap(uint64_t time_gap);
    void TransformToSE3Pose(const Eigen::Matrix4d& car_pose, SE3Pose* se3_pose);
    apStatus_t IntegralCarPose(const uint64_t timestamp,
                               const OdometryInfo& odometry,
                               Eigen::Matrix4d* pose);
    apStatus_t GetVelocityFromWindow(const uint64_t timestamp,
                                     Point3D_t* velocity);
    apStatus_t GetAccelerationFromWindow(const uint64_t timestamp,
                                         Point3D_t* acceleration);
    apStatus_t GetAngularFromWindow(const uint64_t timestamp,
                                    Point3D_t* angular);
    void SE3PoseToTransform(const SE3Pose se3_pose, Eigen::Matrix4d* trans);

    apStatus_t InsertOdometryBK(const uint64_t& timestamp_ns,
                                const OdometryInfo& odom_info);
    apStatus_t ClearOldStateInOdomMapBK(uint64_t time_gap);
    void OdometryInfoToTransform(const OdometryInfo& odom_info,
                                 Eigen::Matrix4d* trans);
    void RK4Integrator(const Eigen::Vector3d& prev_position,
                       const Eigen::Quaterniond& prev_rotation,
                       const Eigen::Vector3d& prev_linear_velocity,
                       const Eigen::Vector3d& prev_angular_velocity,
                       const Eigen::Vector3d& current_linear_velocity,
                       const Eigen::Vector3d& current_angular_velocity,
                       double dt,
                       Eigen::Vector3d* current_position,
                       Eigen::Quaterniond* current_rotation);
    Eigen::Quaterniond expAndTheta(Eigen::Vector3d const& omega, double* theta);
    void HighOrderIntegrator(const Eigen::Vector3d& prev_position,
                             const Eigen::Quaterniond& prev_rotation,
                             const Eigen::Vector3d& prev_linear_velocity,
                             const Eigen::Vector3d& prev_angular_velocity,
                             const Eigen::Vector3d& current_linear_velocity,
                             const Eigen::Vector3d& current_angular_velocity,
                             double dt,
                             Eigen::Vector3d* current_position,
                             Eigen::Quaterniond* current_rotation);
    void LuoquanIntegrator(const Eigen::Vector3d& prev_position,
                           const Eigen::Quaterniond& prev_rotation,
                           const Eigen::Vector3d& prev_linear_velocity,
                           const Eigen::Vector3d& prev_angular_velocity,
                           const Eigen::Vector3d& current_linear_velocity,
                           const Eigen::Vector3d& current_angular_velocity,
                           double dt,
                           Eigen::Vector3d* current_position,
                           Eigen::Quaterniond* current_rotation);

 public:
    void SetFuSlotPos(const uint64_t& timestamp, const SE3Pose& se3_pose) {
        std::lock_guard<std::mutex> lock(slot_pos_mutex_);
        if (fus_slot_pos_.size() > 1000) {
            fus_slot_pos_.erase(fus_slot_pos_.begin());
        }
        fus_slot_pos_[timestamp] = se3_pose;
    }
    apStatus_t GetFuSlotPos(const uint64_t& timestamp, SE3Pose* se3_pose) {
        std::lock_guard<std::mutex> lock(slot_pos_mutex_);

        auto iter = fus_slot_pos_.lower_bound(timestamp);
        if (iter != fus_slot_pos_.end()) {
            *se3_pose = iter->second;
            fus_slot_pos_.erase(fus_slot_pos_.begin(), iter);
        } else {
            AP_LERROR(GetFuSlotPos) << "GetFuSlotPos window empty";
            return AP_LOCALIZATION_ERROR;
        }

        return AP_SUCCESS;
    }

 private:
    std::shared_ptr<Memory> shared_memory_ = nullptr;
    std::shared_ptr<Lock> sem_ = nullptr;
    std::atomic<bool> inited_;
    std::atomic<bool> reset_odom_flag_{false};
    std::atomic<bool> reset_se3pose_flag_{false};
    Mode mode_ = PASSIVE_UPDATE;

    // for AUTO_UPDATE mode
    std::thread tid_;
    bool stop_autoupdate_mode_ = false;
    // LocalizationInfo info_;
    static constexpr double kNanoSecToSec = 1.0e-9;
    static constexpr float32_t autoupdate_hz = 100.0;
    static constexpr float32_t window_interval_time_s = 10.0;  // s
    static constexpr float32_t min_gap = 0.1;
    std::map<uint64_t, OdometryInfo> time_to_odom_;
    std::map<uint64_t, OdometryInfo> time_to_odom_bk;
    std::map<uint64_t, SE3Pose> time_to_se3pose_;
    std::map<uint64_t, SE3Pose> fus_slot_pos_;
    std::mutex slot_pos_mutex_;
    std::mutex mutex_;
    std::mutex odom_map_mutex_;
    std::mutex deque_info_mutex_;
    uint64_t word_timestamp;
    DequeLocalizationInfo deque_info_;
    // std::ofstream log_;
};

extern template class VedLocalizationQuerier<true>;
extern template class VedLocalizationQuerier<false>;

extern VedLocalizationQuerier<false>& ved_localization_querier;
extern VedLocalizationQuerier<true>& ved_localization_querier_ipc;

}  // namespace perception
}  // namespace senseAD

#endif  // BUILD_ADAS
