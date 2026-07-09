// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_vehicle_processing_to_chassis_convertor.cpp
// Purpose:
//   Convert Vehicleprocessing to sapChassisState without RSCL/SWCFL Proxy.

#include "transport/rscl/converters/vehicle_processing_converter.hpp"

#include <cstdint>
#include <memory>

namespace usharing_dlp_node {
namespace senseauto {

using Vehicleprocessing = ::senseAD::msg::planning::Vehicleprocessing;

bool ConvertSenseAutoVehicleProcessingToChassisState(
    Vehicleprocessing::Reader msg, SapChassisStatePtr* output,
    uint64_t* timestamp_ns) {
  if (output == nullptr || timestamp_ns == nullptr) {
    return false;
  }

  auto chassis_state = std::make_shared<sapChassisState>();
  chassis_state->timestamp = msg.getTimeStamp();

  // 源代码差异说明：VehicleProcessingConvertor 中大部分 chassis 字段原本就
  // 被注释掉了。这里只保留旧链路实际写入 SDK 的 turn_signal/set_speed
  // 以及 no_impl_turn_signal 状态。
  if (!chassis_state->lateral_engaged && !chassis_state->pre_msg_turn_signal &&
      static_cast<bool>(msg.getTurningLightSignal())) {
    chassis_state->no_impl_turn_signal = true;
  }
  if (chassis_state->pre_msg_turn_signal &&
      !static_cast<bool>(msg.getTurningLightSignal())) {
    chassis_state->no_impl_turn_signal = false;
  }
  chassis_state->pre_msg_turn_signal =
      static_cast<int>(msg.getTurningLightSignal());
  chassis_state->turn_signal =
      static_cast<uint64_t>(msg.getTurningLightSignal());
  chassis_state->set_speed = msg.getSetSpeed() + 0.8f;

  *timestamp_ns = msg.getHeader().getTime().getNanoSec();
  *output = chassis_state;
  return true;
}

}  // namespace senseauto
}  // namespace usharing_dlp_node
