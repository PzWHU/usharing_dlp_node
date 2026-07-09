// Ported from:
//   adapter/perception_camera/src/convertors/perception/lidar_gop_convertor.cpp
// Purpose:
//   Convert 0434 Lidar GOP ObjectFrame messages to sapPerceptionLidarGopFrame.
#pragma once

#include <cstdint>

#include <ad_msg_idl/perception/perception.capnp.h>

#include "sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace senseauto {

// Source mapping:
//   LidarGopFrameMsgConvertor::ConvertLidarGopFrame()
//   LidarGopFrameMsgConvertor::Convert()
//   adapter/perception_camera/src/convertors/perception/lidar_gop_convertor.cpp
// 输入：/perception/lidar_gop/object 上的 ObjectFrame。
// 输出：sapPerceptionLidarGopFrame shared_ptr，timestamp_ns 对应旧 Proxy
// Timestamp(frame_sp->timestamp_ns)。
bool ConvertSenseAutoLidarGopFrameToSap(
    ::senseAD::msg::perception::ObjectFrame::Reader msg,
    SapLidarGopFramePtr* output,
    uint64_t* timestamp_ns);

}  // namespace senseauto
}  // namespace usharing_dlp_node
