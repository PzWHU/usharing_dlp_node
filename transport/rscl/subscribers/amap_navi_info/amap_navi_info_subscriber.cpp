// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_amap_navi_info_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   Subscribe to AmapNaviInfo RSCL input and forward converted sap navigation data.

#include "transport/rscl/subscribers/amap_navi_info/amap_navi_info_subscriber.hpp"

#include <utility>

#include "transport/rscl/converters/amap_navi_info_converter.hpp"
#include "rscl_access.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

void AmapNaviInfoSubscriber::SetParam(const TopicConfig& param) {
  param_ = param;
}

void AmapNaviInfoSubscriber::SetCallback(
    std::function<void(SapAmapNaviInfoPtr, uint64_t)> callback) {
  callback_ = std::move(callback);
}

bool AmapNaviInfoSubscriber::Init() {
  if (param_.topic.empty()) {
    return false;
  }
  // 高德导航输入单独订阅，对应 SDK 的 sapPushAmapNaviInfo。
  return CRsclAccess::Instance()
             ->subscribe<senseAD::msg::amap_navigation::AmapNaviInfo>(
                 param_.topic, kRsclDefaultQueueSize,
                 &AmapNaviInfoSubscriber::AmapNaviInfoCallback, this) == 0;
}

void AmapNaviInfoSubscriber::AmapNaviInfoCallback(
    const std::shared_ptr<
        ReceivedMsg<senseAD::msg::amap_navigation::AmapNaviInfo>>& msg) {
  if (!msg || !callback_) {
    return;
  }

  SapAmapNaviInfoPtr output;
  uint64_t timestamp_ns = 0;
  if (senseauto::ConvertSenseAutoAmapNaviInfoToSap(msg->Msg(), &output,
                                                   &timestamp_ns)) {
    // output 中的动态数组由 converter 自定义 deleter 释放。
    callback_(std::move(output), timestamp_ns);
  }
}

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
