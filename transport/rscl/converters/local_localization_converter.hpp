// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_local_localization_convertor.cpp
// Purpose:
//   Convert SenseAuto OdomStateInfo messages to sapLocalLocalizationInfo.
#pragma once

#include <ad_msg_idl/ad_localization/localization.capnp.h>

#include "planning_sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace senseauto {

// Source mapping:
//   LocalLocalizationConvertor::Convert()
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_local_localization_convertor.cpp
// 输入：OdomStateInfo。
// 输出：sapLocalLocalizationInfo，measurement_time_ns 用作 SDK push 时间。
bool ConvertSenseAutoOdomStateToLocalLocalization(
    ::senseAD::msg::localization::OdomStateInfo::Reader msg,
    SapLocalLocalizationPtr* output);

}  // namespace senseauto
}  // namespace usharing_dlp_node
