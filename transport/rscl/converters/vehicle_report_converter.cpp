// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_vehicle_report_to_can_bus_convertor.cpp
// Purpose:
//   Convert VehicleReport to VehicleInfo while keeping takeover counters in
//   caller-owned state instead of SWCFL convertor statics.

#include "transport/rscl/converters/vehicle_report_converter.hpp"

#include <cstdint>

namespace usharing_dlp_node {
namespace senseauto {

using VehicleReport = ::senseAD::msg::vehicle::VehicleReport;

namespace {

constexpr double kRadiansToDegrees = 57.295779513082320876;

}  // namespace

bool ConvertSenseAutoVehicleReportToVehicleInfo(
    VehicleReport::Reader msg, VehicleReportConversionState* state,
    VehicleInfo* output, uint64_t* timestamp_ns) {
  if (state == nullptr || output == nullptr || timestamp_ns == nullptr) {
    return false;
  }

  const int brake_count_threshold = 3;
  const int throttle_count_threshold = 10;
  const int lateral_takeover_threshold = 10;

  const int brake_open = msg.getChassis().getWheelPulseFl();
  const auto reserved = msg.getMisc().getReserved();
  const bool throttle_over = reserved.size() > 0 ? reserved[0] : false;

  // 接管判断沿用旧 convertor 的跨帧计数：制动连续触发 3 帧认为纵向接管，
  // reserved[0] 的油门接管计数保留但当前只限幅不直接置 take_over。
  if (brake_open == 0) {
    state->brake_takeover_count = 0;
  } else {
    ++state->brake_takeover_count;
  }
  if (!throttle_over) {
    state->throttle_takeover_count = 0;
  } else {
    ++state->throttle_takeover_count;
  }

  bool take_over = false;
  if (state->brake_takeover_count >= brake_count_threshold) {
    take_over = true;
    state->brake_takeover_count = brake_count_threshold;
  }
  if (state->throttle_takeover_count >= throttle_count_threshold) {
    state->throttle_takeover_count = throttle_count_threshold;
  }

  const float steering_torque = msg.getSteering().getTorqueActual();
  bool is_back = false;
  // 横向恢复和横向接管都依赖方向盘扭矩的连续帧计数。
  if (steering_torque < 2.5f) {
    ++state->lateral_back_count;
  } else {
    state->lateral_back_count = 0;
  }
  if (state->lateral_back_count > 200) {
    state->lateral_back_count = 0;
    is_back = true;
  }

  bool lateral_take_over = false;
  if (steering_torque < 2.5f) {
    state->lateral_takeover_count = 0;
  } else {
    ++state->lateral_takeover_count;
  }
  if (state->lateral_takeover_count >= lateral_takeover_threshold) {
    lateral_take_over = true;
    state->lateral_takeover_count = lateral_takeover_threshold;
  }

  *output = VehicleInfo{};
  // VehicleInfo 只保存状态机需要的中立字段；sapManualSignal 在 core
  // VehicleStateMachine 中统一生成。
  output->vehicle_speed = msg.getChassis().getVehicleMps();
  output->imu_accel_x = msg.getImu().getAccelX();
  output->steering_angle =
      static_cast<float>(msg.getSteering().getAngleActual() *
                         kRadiansToDegrees);
  output->btn_acc = msg.getMisc().getBtnAccOnoff();
  output->btn_acc_cancel = msg.getMisc().getBtnAccCancel();
  output->btn_acc_set = msg.getMisc().getBtnAccSet();
  output->btn_acc_resume = msg.getMisc().getBtnAccResume();
  output->take_over_status = take_over;
  output->lat_take_over_status = lateral_take_over;
  output->is_back = is_back;
  output->wheel_pulse_rl = msg.getChassis().getWheelPulseRl();
  output->turn_left_lamp = msg.getBeam().getTurnLeftLamp();
  output->turn_right_lamp = msg.getBeam().getTurnRightLamp();

  *timestamp_ns = msg.getHeader().getTime().getNanoSec();
  return true;
}

}  // namespace senseauto
}  // namespace usharing_dlp_node
