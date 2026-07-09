// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_fusion_object_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   Subscribe to fusion object RSCL input and forward converted sap fusion objects.

#include "transport/rscl/subscribers/fusion_object_frame/fusion_object_frame_subscriber.hpp"

#include <utility>

#include "transport/rscl/converters/fusion_object_frame_converter.hpp"
#include "rscl_access.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

void FusionObjectFrameSubscriber::SetParam(const TopicConfig& param) {
  param_ = param;
}

void FusionObjectFrameSubscriber::SetCallback(
    std::function<void(SapFusionObjectFramePtr)> callback) {
  callback_ = std::move(callback);
}

bool FusionObjectFrameSubscriber::Init() {
  if (param_.topic.empty()) {
    return false;
  }
  // 融合目标输入使用 ObjectFrame schema，但配置 topic 通常为
  // /perception/fusion/object。
  return CRsclAccess::Instance()
             ->subscribe<senseAD::msg::perception::ObjectFrame>(
                 param_.topic, kRsclDefaultQueueSize,
                 &FusionObjectFrameSubscriber::FusionObjectFrameCallback, this) == 0;
}

void FusionObjectFrameSubscriber::FusionObjectFrameCallback(
    const std::shared_ptr<
        ReceivedMsg<senseAD::msg::perception::ObjectFrame>>& msg) {
  if (!msg || !callback_) {
    return;
  }

  SapFusionObjectFramePtr output;
  if (senseauto::ConvertSenseAutoFusionObjectFrameToSap(msg->Msg(),
                                                        &output)) {
    // output 已经是 sapPerceptionFusionObjectFrame。
    callback_(std::move(output));
  }
}

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
