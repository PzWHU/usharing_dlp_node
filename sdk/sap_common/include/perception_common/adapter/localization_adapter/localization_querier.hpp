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

namespace senseAD {
namespace perception {

enum class LocalizationDataStatus {
    INVALID = 0,
    VALID = 1,
};

struct VelocityExt {
    LocalizationDataStatus status;
    Point3D_t velocity;
};

struct AccelerationExt {
    LocalizationDataStatus status;
    Point3D_t acceleration;
};

struct AngularSpeedExt {
    LocalizationDataStatus status;
    Point3D_t angular_speed;
};

struct CarPoseExt {
    LocalizationDataStatus status;
    CarPose car_pose;
};

struct Trans2DExt {
    LocalizationDataStatus status;
    Eigen::Matrix3d trans_2d;
};

struct Trans3DExt {
    LocalizationDataStatus status;
    Eigen::Matrix4d trans;
};

template <bool InterProcess = false>
class LocalizationQuerier {
 public:
    using Lock =
        typename type_traits::EnableIfElse<InterProcess,
                                           ecl::Semaphore,
                                           InnerProcessBinarySemaphore>::type;
    using Memory = typename type_traits::EnableIfElse<
        InterProcess,
        ecl::SharedMemory<LocalizationInfo>,
        InnerProcessMemory<LocalizationInfo>>::type;

    LocalizationQuerier() : inited_(false) {}

    ~LocalizationQuerier() {
        if (inited_ && AUTO_UPDATE == mode_ && tid_.joinable()) {
            stop_autoupdate_mode_ = true;
            tid_.join();
        }
    }

    /*
     * @brief Singleton localization querier instance
     * @return Reference of singleton object of localization querier
     */
    static LocalizationQuerier& Instance();

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

    /*
     * @brief query the designated timestamp velocity
     * @param timestamp_ns [in] latest velocity if timestamp_ns[i] == 0
     * @param velocity_vector [in] velocity vector, if fail, flag invalid
     * @return status if abnormal state
     */
    apStatus_t VelocityQuery(const std::vector<uint64_t>& timestamp_ns,
                             std::vector<VelocityExt>* velocity_vector);

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
    LocalizationInfo info_;
    static constexpr double kNanoSecToSec = 1.0e-9;
    static constexpr float32_t autoupdate_hz = 100.0;
    static constexpr float32_t window_interval_time_s = 10.0;  // s
    std::map<uint64_t, OdometryInfo> time_to_odom_;
    std::map<uint64_t, SE3Pose> time_to_se3pose_;
    std::mutex mutex_;
    std::mutex odom_map_mutex_;
};

extern template class LocalizationQuerier<true>;
extern template class LocalizationQuerier<false>;

extern LocalizationQuerier<false>& g_localization_querier;
extern LocalizationQuerier<true>& g_localization_querier_ipc;

}  // namespace perception
}  // namespace senseAD

#endif  // BUILD_ADAS
