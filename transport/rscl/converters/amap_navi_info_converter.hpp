// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_amap_navi_info_convertor.cpp
// Purpose:
//   Convert SenseAuto AmapNaviInfo messages to sapAmapNaviInfo.
#pragma once

#include <cstdint>

#include <ad_msg_idl/ad_navi/Amap_navi_msgs.capnp.h>

#include "planning_sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace senseauto {

// Source mapping:
//   AmapNaviInfoConvertor::ConvertAmapNaviInfo()
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_amap_navi_info_convertor.cpp
// 输出 sapAmapNaviInfo 和原始导航消息时间戳。
bool ConvertSenseAutoAmapNaviInfoToSap(
    ::senseAD::msg::amap_navigation::AmapNaviInfo::Reader msg,
    SapAmapNaviInfoPtr* output, uint64_t* timestamp_ns);

}  // namespace senseauto
}  // namespace usharing_dlp_node
