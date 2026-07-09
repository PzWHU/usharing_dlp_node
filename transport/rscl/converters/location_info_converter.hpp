// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_global_localization_convertor.cpp
// Purpose:
//   Convert SenseAuto Ins messages to sapLocationInfo.
#pragma once

#include <cstdint>

#include <ad_msg_idl/ad_sensor/sensor.capnp.h>

#include "sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace senseauto {

// Source mapping:
//   GlobalLocalizationConvertor::Convert()
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_global_localization_convertor.cpp
// 输入：INS 消息。
// 输出：sapLocationInfo 和用于 sapPushLocationInfo 的 timestamp_ns。
bool ConvertSenseAutoInsToLocationInfo(
    ::senseAD::msg::sensor::Ins::Reader msg, SapLocationInfoPtr* output,
    uint64_t* timestamp_ns);

}  // namespace senseauto
}  // namespace usharing_dlp_node
