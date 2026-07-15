// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_fusion_object_convertor.cpp
// Purpose:
//   Convert SenseAuto fusion ObjectFrame messages to sapPerceptionFusionObjectFrame.
#pragma once

#include <ad_msg_idl/perception/perception.capnp.h>

#include "planning_sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace senseauto {

// Source mapping:
//   FusionObjectFrameMsgConvertor::ConvertFusionObjectFrame()
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_fusion_object_convertor.cpp
// 输入：融合目标 ObjectFrame。
// 输出：sapPerceptionFusionObjectFrame，释放规则使用 SDK ReleaseFusionObjectFrameMemory。
bool ConvertSenseAutoFusionObjectFrameToSap(
    ::senseAD::msg::perception::ObjectFrame::Reader msg,
    SapFusionObjectFramePtr* output);

}  // namespace senseauto
}  // namespace usharing_dlp_node
