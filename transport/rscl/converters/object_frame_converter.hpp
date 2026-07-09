// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_object_convertor.cpp
// Purpose:
//   Convert SenseAuto ObjectFrame messages to sapPerceptionObjectFrame.
#pragma once

#include <ad_msg_idl/perception/perception.capnp.h>

#include "sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace senseauto {

// Source mapping:
//   ObjectFrameMsgConvertor::ConvertObjectFrame()
//   ObjectFrameMsgConvertor::Convert()
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_object_convertor.cpp
// 输入：SenseAuto perception::ObjectFrame::Reader。
// 输出：sapPerceptionObjectFrame shared_ptr，内部数组由 sap_raii 释放。
bool ConvertSenseAutoObjectFrameToSap(
    ::senseAD::msg::perception::ObjectFrame::Reader msg,
    SapObjectFramePtr* output);

}  // namespace senseauto
}  // namespace usharing_dlp_node
