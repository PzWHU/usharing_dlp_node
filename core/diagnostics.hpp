// Ported from:
//   adapter/perception_camera/include/perception_camera_helper.hpp
// Purpose:
//   Keep small planning helper structs without RSCL/PAL dependencies.
#pragma once

#include <cstdint>
#include <vector>

namespace usharing_dlp_node {

// PlanningDebug 输出中除 SDK debug 字符串外，还需要携带旧 workshop
// 维护的车辆/自动驾驶状态。
struct PerceptionPlanningInfo {
  bool auto_mode_status{false};
  bool lat_auto_mode_status{false};
  int ego_speed{0};
  int ego_acc_can{0};
};

// VehicleReport 转换后的中间结构。它不是 sap_camera SDK DTO，而是
// VehicleStateMachine 生成 sapManualSignal 所需的最小输入集合。
struct VehicleInfo {
  float steering_angle{0.0f};
  float vehicle_speed{0.0f};
  float imu_accel_x{0.0f};
  int32_t wheel_pulse_rl{0};
  bool btn_acc{false};
  bool btn_acc_cancel{false};
  bool btn_acc_set{false};
  bool btn_acc_resume{false};
  bool take_over_status{false};
  bool lat_take_over_status{false};
  bool is_back{false};
  bool turn_left_lamp{false};
  bool turn_right_lamp{false};
};

// DLP 诊断输出使用的 fault code 集合，最终由 transport 转成对应消息。
using FaultCodes = std::vector<int>;

}  // namespace usharing_dlp_node
