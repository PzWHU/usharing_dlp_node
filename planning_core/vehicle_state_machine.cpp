// Ported from:
//   PerceptionCameraSdkWorkshop::OnVehicleInfo()
//   adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp
// Purpose:
//   Preserve DLP manual-signal behavior without SWCFL Proxy or ROS dependencies.

#include "planning_core/vehicle_state_machine.hpp"

#include <cmath>

namespace usharing_dlp_node {

namespace {
constexpr float kPi = 3.14159265358979323846f;
}

void VehicleStateMachine::UpdateChassisState(
    const sapChassisState& chassis_state) {
  std::lock_guard<std::mutex> lock(mutex_);
  // Vehicleprocessing 先到时只更新状态机缓存；真正发给 SDK 的
  // sapManualSignal 会在下一帧 VehicleReport 中综合生成。
  turn_signal_ = static_cast<TurnSignal>(chassis_state.turn_signal);
  set_speed_ = chassis_state.set_speed;
}

SapManualSignalPtr VehicleStateMachine::OnVehicleInfo(
    const VehicleInfo& vehicle_info) {
  std::lock_guard<std::mutex> lock(mutex_);
  ego_speed_ = static_cast<int>(vehicle_info.vehicle_speed * 100);
  ego_acc_can_ = static_cast<int>(vehicle_info.imu_accel_x * 100);

  // ACC 开关逻辑沿用旧 OnVehicleInfo()：在计数窗口内检测到两次 ACC
  // 按键动作后进入自动模式；cancel 或接管会退出自动/横向自动模式。
  if (vehicle_info.btn_acc) {
    if (count_acc_on_off_ > 0) {
      count_acc_on_ = 2;
      count_acc_on_off_ = 0;
    }
    if (count_acc_on_ == 0) {
      count_acc_on_ = 1;
    }
  }
  if (count_acc_on_ == 1 && !vehicle_info.btn_acc) {
    ++count_acc_on_off_;
  }
  if (count_acc_on_off_ > 500) {
    count_acc_on_ = 0;
    count_acc_on_off_ = 0;
  }
  if (count_acc_on_ >= 2) {
    count_acc_on_ = 0;
    btn_debug_button_ = 1;
    auto_mode_status_ = true;
    lat_auto_mode_status_ = true;
    force_turn_signal_ = 0;
    force_velocity_signal_ = 0;
  }
  if (vehicle_info.btn_acc_cancel) {
    btn_debug_button_ = 0;
    auto_mode_status_ = false;
    force_turn_signal_ = 0;
    force_velocity_signal_ = 0;
  }
  if (!auto_mode_status_) {
    lat_auto_mode_status_ = false;
  }

  // set/resume 按键先记录最近 10 帧，等连续窗口内没有新的按键后再
  // 更新 expected_speed_，避免单次按键在多帧 CAN 报文中被重复计数。
  if (vehicle_info.btn_acc_set) {
    ++force_velocity_signal_;
    is_start_set_plus_ = true;
    velocity_signals_.push_back(1);
  } else if (vehicle_info.btn_acc_resume) {
    --force_velocity_signal_;
    is_start_set_minus_ = true;
    velocity_signals_.push_back(-1);
    if (force_velocity_signal_ < 0) {
      force_velocity_signal_ = 0;
    }
  } else {
    velocity_signals_.push_back(0);
  }
  while (velocity_signals_.size() > 10) {
    velocity_signals_.pop_front();
  }

  take_over_status_ = vehicle_info.take_over_status;
  lat_take_over_status_ = vehicle_info.lat_take_over_status;
  if (take_over_status_) {
    auto_mode_status_ = false;
  }
  if (lat_take_over_status_) {
    lat_auto_mode_status_ = false;
  }
  if (!lat_auto_mode_status_ && auto_mode_status_ && vehicle_info.is_back) {
    lat_auto_mode_status_ = true;
  }

  // 转向灯信息同时来自 VehicleInfo 和 ChassisState。VehicleInfo 的灯光
  // 历史用于判断人工拨杆状态；ChassisState 的 turn_signal 用于最终
  // 写入 sapManualSignal 的 TurnSignal/DLPEgolight。
  if (vehicle_info.turn_left_lamp) {
    force_turn_signal_ = 1;
  } else if (vehicle_info.turn_right_lamp) {
    force_turn_signal_ = 2;
  } else {
    force_turn_signal_ = 0;
  }
  turn_signals_.push_back(force_turn_signal_);
  while (turn_signals_.size() > 10) {
    turn_signals_.pop_front();
  }

  is_in_turn_signal_ = 0;
  for (const auto signal : turn_signals_) {
    if (signal != 0) {
      is_in_turn_signal_ = signal;
    }
  }

  dlp_ego_lights_.push_front(force_turn_signal_);
  while (dlp_ego_lights_.size() > 51) {
    dlp_ego_lights_.pop_back();
  }

  float dlp_ego_light = 1.0f;
  int turn_light_signal = 0;
  switch (turn_signal_) {
    case TurnSignal::kLeft:
      turn_light_signal = 1;
      dlp_ego_light = 2.0f;
      break;
    case TurnSignal::kRight:
      turn_light_signal = 2;
      dlp_ego_light = 3.0f;
      break;
    case TurnSignal::kNeutral:
    default:
      turn_light_signal = 0;
      dlp_ego_light = 1.0f;
      break;
  }

  bool is_set_done = true;
  for (const auto velocity_signal : velocity_signals_) {
    if (velocity_signal != 0) {
      is_set_done = false;
    }
  }
  if (is_set_done) {
    if (is_start_set_plus_) {
      expected_speed_ += 5;
      is_start_set_plus_ = false;
      is_start_set_minus_ = false;
    } else if (is_start_set_minus_) {
      expected_speed_ -= 5;
      is_start_set_plus_ = false;
      is_start_set_minus_ = false;
    }
  }
  if (expected_speed_ < 10) {
    expected_speed_ = 10;
  } else if (expected_speed_ > 50) {
    expected_speed_ = 50;
  }

  auto manual_signal = std::make_shared<sapManualSignal>();
  // timestamp 由 SapPlanningEngine::PushManualSignal() 统一写入，避免
  // 状态机依赖具体中间件时间源。
  manual_signal->timestamp = 0;
  manual_signal->ExpectedSpeed = set_speed_ * 3.6f;
  manual_signal->TurnSignal = static_cast<uint16_t>(turn_light_signal);
  manual_signal->DLPEgolight = dlp_ego_light;
  manual_signal->front_wheel_angle =
      vehicle_info.steering_angle * kPi / 180.0f / 12.88f;
  return manual_signal;
}

PerceptionPlanningInfo VehicleStateMachine::planning_info() const {
  // PlanningDebug 输出需要这些状态；它们不直接来自 SDK，而是旧 workshop
  // 在 OnVehicleInfo() 中维护的业务状态。
  PerceptionPlanningInfo info;
  info.auto_mode_status = auto_mode_status_;
  info.lat_auto_mode_status = lat_auto_mode_status_;
  info.ego_acc_can = ego_acc_can_;
  info.ego_speed = ego_speed_;
  return info;
}

}  // namespace usharing_dlp_node
