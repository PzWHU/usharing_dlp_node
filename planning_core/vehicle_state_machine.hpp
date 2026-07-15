// Ported from:
//   PerceptionCameraSdkWorkshop::OnVehicleInfo()
//   adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp
// Purpose:
//   Keep vehicle button/status state outside ROS and outside SWCFL Proxy.
//
// 中文说明：
//   VehicleStateMachine 承接原 workshop 中和车辆状态相关的状态机逻辑：
//   - 从 chassis/vehicle report 更新自动驾驶状态、速度、接管状态；
//   - 生成 sapManualSignal 给 SDK；
//   - 维护 PerceptionPlanningInfo，供 PlanningDebug 输出携带车辆状态。
//   该类只依赖 planning_core/planning_sap 类型，不能依赖 RSCL/ROS。
#pragma once

#include <atomic>
#include <deque>
#include <mutex>

#include "planning_core/diagnostics.hpp"
#include "planning_sap/sap_types.hpp"

namespace usharing_dlp_node {

enum class TurnSignal { kNeutral = 0, kLeft = 1, kRight = 2 };

class VehicleStateMachine {
 public:
  // 更新底盘状态缓存，主要用于 PlanningDebug 的 auto/lat mode 状态。
  void UpdateChassisState(const sapChassisState& chassis_state);
  // 根据车辆按键/状态生成 SDK 需要的 sapManualSignal。
  SapManualSignalPtr OnVehicleInfo(const VehicleInfo& vehicle_info);
  // 返回随 PlanningDebug 发布的车辆状态快照。
  PerceptionPlanningInfo planning_info() const;

 private:
  mutable std::mutex mutex_;
  std::atomic_bool auto_mode_status_{false};
  std::atomic_bool lat_auto_mode_status_{false};
  std::atomic_int ego_speed_{0};
  std::atomic_int ego_acc_can_{0};
  std::atomic_int force_turn_signal_{0};
  std::atomic_int force_velocity_signal_{0};
  std::atomic_int is_in_turn_signal_{0};
  std::atomic_bool take_over_status_{false};
  std::atomic_bool lat_take_over_status_{false};
  int btn_debug_button_{0};
  int expected_speed_{30};
  TurnSignal turn_signal_{TurnSignal::kNeutral};
  float set_speed_{8.3f};
  std::deque<int> velocity_signals_;
  std::deque<int> turn_signals_;
  std::deque<int> dlp_ego_lights_;
  int count_acc_on_{0};
  int count_acc_on_off_{0};
  bool is_start_set_plus_{false};
  bool is_start_set_minus_{false};
};

}  // namespace usharing_dlp_node
