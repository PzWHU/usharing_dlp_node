// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_vehicle_report_to_can_bus_convertor.cpp
// Purpose:
//   Convert SenseAuto VehicleReport messages to middleware-neutral VehicleInfo.
#pragma once

#include <cstdint>

#include <ad_msg_idl/ad_vehicle/vehicle.capnp.h>

#include "planning_core/diagnostics.hpp"

namespace usharing_dlp_node {
namespace senseauto {

struct VehicleReportConversionState {
  // 以下计数器来自旧 convertor 的静态变量。移植后由 subscriber 持有，
  // 避免转换函数隐藏全局状态。
  int brake_takeover_count{0};
  int throttle_takeover_count{0};
  int lateral_back_count{0};
  int lateral_takeover_count{0};
};

// Source mapping:
//   VehicleReportConvertor::Convert()
//   VehicleReportConvertor::isTakeOver()
//   VehicleReportConvertor::isLatControl()
//   VehicleReportConvertor::isLatTakeOver()
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_vehicle_report_to_can_bus_convertor.cpp
// 输出 VehicleInfo 而不是 sapManualSignal；manual signal 生成属于
// VehicleStateMachine 的跨帧业务逻辑。
bool ConvertSenseAutoVehicleReportToVehicleInfo(
    ::senseAD::msg::vehicle::VehicleReport::Reader msg,
    VehicleReportConversionState* state, VehicleInfo* output,
    uint64_t* timestamp_ns);

}  // namespace senseauto
}  // namespace usharing_dlp_node
