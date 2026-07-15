// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_occ_convertor.cpp
// Purpose:
//   Convert SenseAuto OccupancyInfo messages to sapPerceptionOccFrame.
#pragma once

#include <ad_msg_idl/perception/perception.capnp.h>

#include "planning_sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace senseauto {

// Source mapping:
//   OccFrameMsgConvertor::ConvertOccFrame()
//   OccFrameMsgConvertor::ConvertOccInfo()
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_occ_convertor.cpp
// 输入：SenseAuto OccupancyInfo。
// 输出：sapPerceptionOccFrame，动态内存由 sap_raii 中的 OCC deleter 释放。
bool ConvertSenseAutoOccupancyInfoToSap(
    ::senseAD::msg::perception::OccupancyInfo::Reader msg,
    SapOccFramePtr* output);

}  // namespace senseauto
}  // namespace usharing_dlp_node
