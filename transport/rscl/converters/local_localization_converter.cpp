// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_local_localization_convertor.cpp
// Purpose:
//   Convert OdomStateInfo to sapLocalLocalizationInfo without adding Eigen or
//   middleware dependencies to the standalone planning node.

#include "transport/rscl/converters/local_localization_converter.hpp"

#include <cmath>
#include <memory>

namespace usharing_dlp_node {
namespace senseauto {

using OdomStateInfo = ::senseAD::msg::localization::OdomStateInfo;

namespace {

void SetQuaternionFromRollPitchYaw(double roll, double pitch, double yaw,
                                   sapQuaternion_t* quat) {
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  // Source Eigen expression:
  //   AngleAxis(roll, X) * AngleAxis(pitch, Y) * AngleAxis(yaw, Z)
  quat->qw = cr * cp * cy - sr * sp * sy;
  quat->qx = sr * cp * cy + cr * sp * sy;
  quat->qy = cr * sp * cy - sr * cp * sy;
  quat->qz = cr * cp * sy + sr * sp * cy;
}

}  // namespace

bool ConvertSenseAutoOdomStateToLocalLocalization(
    OdomStateInfo::Reader msg, SapLocalLocalizationPtr* output) {
  if (output == nullptr) {
    return false;
  }

  auto local_info = std::make_shared<sapLocalLocalizationInfo>();
  InitsapLocalLocalizationInfo(local_info.get());

  // 旧 convertor 将 measurement_time_ns 同时写入结构体顶层和 local_pose，
  // SDK 使用该时间对齐其他输入帧。
  local_info->measurement_time_ns = msg.getMeasurementTimeNs();
  local_info->local_pose.timestamp_ns = msg.getMeasurementTimeNs();
  local_info->local_pose.position.x = msg.getPositionFlu().getX();
  local_info->local_pose.position.y = msg.getPositionFlu().getY();
  local_info->local_pose.position.z = msg.getPositionFlu().getZ();
  SetQuaternionFromRollPitchYaw(msg.getRoll(), msg.getPitch(), msg.getYaw(),
                                &local_info->local_pose.quat);

  // SenseAuto 消息给出标准差，sap DTO 中同时需要标准差和协方差；
  // 协方差按旧实现使用 std^2。
  const auto position_std = msg.getPositionFluStdDev();
  local_info->local_translation_covariance.x =
      position_std.getX() * position_std.getX();
  local_info->local_translation_covariance.y =
      position_std.getY() * position_std.getY();
  local_info->local_translation_covariance.z =
      position_std.getZ() * position_std.getZ();
  local_info->position_std.x = position_std.getX();
  local_info->position_std.y = position_std.getY();
  local_info->position_std.z = position_std.getZ();

  const auto attitude_std = msg.getAttitudeStdDev();
  local_info->local_rotation_covariance.x =
      attitude_std.getX() * attitude_std.getX();
  local_info->local_rotation_covariance.y =
      attitude_std.getY() * attitude_std.getY();
  local_info->local_rotation_covariance.z =
      attitude_std.getZ() * attitude_std.getZ();
  local_info->attitude_std.x = attitude_std.getX();
  local_info->attitude_std.y = attitude_std.getY();
  local_info->attitude_std.z = attitude_std.getZ();

  local_info->euler_angle.roll = msg.getRoll();
  local_info->euler_angle.pitch = msg.getPitch();
  local_info->euler_angle.yaw = msg.getYaw();

  const auto linear_velocity = msg.getLinearVelocity();
  local_info->ego_car_linear_velocity.x = linear_velocity.getX();
  local_info->ego_car_linear_velocity.y = linear_velocity.getY();
  local_info->ego_car_linear_velocity.z = linear_velocity.getZ();

  const auto linear_acceleration = msg.getLinearAcceleration();
  local_info->ego_car_linear_acceleration.x = linear_acceleration.getX();
  local_info->ego_car_linear_acceleration.y = linear_acceleration.getY();
  local_info->ego_car_linear_acceleration.z = linear_acceleration.getZ();

  const auto angular_velocity = msg.getAngularVelocity();
  local_info->ego_car_angular_velocity.x = angular_velocity.getX();
  local_info->ego_car_angular_velocity.y = angular_velocity.getY();
  local_info->ego_car_angular_velocity.z = angular_velocity.getZ();

  // 状态字符串到精度等级的映射保持旧链路行为：GOOD/OK 为高精度，
  // CONVERGING/UNSTABLE 为可用但未完全收敛，其余视为不可用。
  if (msg.getStatusInfo() == "GOOD" || msg.getStatusInfo() == "OK") {
    local_info->accuracy_level = 2;
  } else if (msg.getStatusInfo() == "CONVERGING" ||
             msg.getStatusInfo() == "UNSTABLE") {
    local_info->accuracy_level = 1;
  } else {
    local_info->accuracy_level = 0;
  }

  *output = local_info;
  return true;
}

}  // namespace senseauto
}  // namespace usharing_dlp_node
