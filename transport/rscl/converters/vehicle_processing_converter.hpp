// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_vehicle_processing_to_chassis_convertor.cpp
// Purpose:
//   Convert SenseAuto Vehicleprocessing messages to sapChassisState.
#pragma once

#include <cstdint>

#include <ad_msg_idl/ad_planning/planning.capnp.h>

#include "planning_sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace senseauto {

// Source mapping:
//   VehicleProcessingConvertor::ConvertVehicleProcessing()
//   VehicleProcessingConvertor::Convert()
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_vehicle_processing_to_chassis_convertor.cpp
// 输出 sapChassisState，并返回原始消息时间戳。
bool ConvertSenseAutoVehicleProcessingToChassisState(
    ::senseAD::msg::planning::Vehicleprocessing::Reader msg,
    SapChassisStatePtr* output, uint64_t* timestamp_ns);

}  // namespace senseauto
}  // namespace usharing_dlp_node
