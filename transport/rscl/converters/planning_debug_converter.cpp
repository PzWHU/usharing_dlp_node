// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/send_convertors/planning_debug_to_msg_convertor.cpp
// Purpose:
//   Preserve Pilot DLP planning debug text conversion without SWCFL Sender.

#include "transport/rscl/converters/planning_debug_converter.hpp"

#include <sstream>

namespace usharing_dlp_node {
namespace senseauto {

std::string BuildPlanningDebugText(
    const sapPerceptionPlanningDebugInfo& debug_info,
    const PerceptionPlanningInfo& info) {
  std::stringstream ss;
  // 文本格式保持旧 PlanningDebugConvertor，便于与原链路 debug UI 对齐。
  ss << "auto_status: " << info.auto_mode_status
     << "\n lat_auto_status: " << info.lat_auto_mode_status
     << "\n ego_acc: " << info.ego_acc_can * 0.01
     << "\n ego_velo: " << static_cast<float>(info.ego_speed) / 100.0f
     << "\n ";
  if (debug_info.string != nullptr) {
    ss << debug_info.string;
  }
  return ss.str();
}

void ConvertPlanningDebugToSenseAuto(
    const sapPerceptionPlanningDebugInfo& debug_info,
    const PerceptionPlanningInfo& info,
    ::senseAD::msg::perception::UniADPlanningDebug::Builder builder) {
  // UniADPlanningDebug 当前只使用 textboxString 字段。
  builder.setTextboxString(BuildPlanningDebugText(debug_info, info));
}

}  // namespace senseauto
}  // namespace usharing_dlp_node
