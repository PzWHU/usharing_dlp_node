// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_global_localization_convertor.cpp
// Purpose:
//   Convert Ins to sapLocationInfo without depending on SWCFL Proxy.

#include "transport/rscl/converters/location_info_converter.hpp"

#include <memory>

namespace usharing_dlp_node {
namespace senseauto {

using Ins = ::senseAD::msg::sensor::Ins;

bool ConvertSenseAutoInsToLocationInfo(Ins::Reader msg,
                                       SapLocationInfoPtr* output,
                                       uint64_t* timestamp_ns) {
  if (output == nullptr || timestamp_ns == nullptr) {
    return false;
  }

  auto location = std::make_shared<sapLocationInfo>();
  // 全局定位输入当前只取 DLP 使用到的字段，保持旧
  // GlobalLocalizationConvertor 的最小映射。
  location->timestamp = msg.getActualTimeNs();
  location->longitude = msg.getLongitude();
  location->latitude = msg.getLatitude();
  location->yaw = msg.getYaw();

  *timestamp_ns = msg.getActualTimeNs();
  *output = location;
  return true;
}

}  // namespace senseauto
}  // namespace usharing_dlp_node
