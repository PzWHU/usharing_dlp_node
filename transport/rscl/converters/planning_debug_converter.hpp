// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/send_convertors/planning_debug_to_msg_convertor.cpp
// Purpose:
//   Convert sap planning debug output to SenseAuto UniADPlanningDebug messages.
#pragma once

#include <string>

#include <ad_msg_idl/perception/perception.capnp.h>

#include "core/diagnostics.hpp"
#include "sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace senseauto {

// Source mapping:
//   PlanningDebugConvertor::Convert()
//   adapter/perception_camera/src/convertors/perception_camera/send_convertors/planning_debug_to_msg_convertor.cpp
// 构造 debug 文本，包含 SDK debug_info 和 VehicleStateMachine 状态。
std::string BuildPlanningDebugText(
    const sapPerceptionPlanningDebugInfo& debug_info,
    const PerceptionPlanningInfo& info);

// Source mapping:
//   PlanningDebugConvertor::Convert()
//   adapter/perception_camera/src/convertors/perception_camera/send_convertors/planning_debug_to_msg_convertor.cpp
// 将 debug 文本写入 UniADPlanningDebug builder。
void ConvertPlanningDebugToSenseAuto(
    const sapPerceptionPlanningDebugInfo& debug_info,
    const PerceptionPlanningInfo& info,
    ::senseAD::msg::perception::UniADPlanningDebug::Builder builder);

}  // namespace senseauto
}  // namespace usharing_dlp_node
