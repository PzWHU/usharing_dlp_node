// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_object_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   Subscribe to object frame RSCL input and forward converted sap objects.

#include "transport/rscl/subscribers/object_frame/object_frame_subscriber.hpp"

#include <utility>

#include "transport/rscl/converters/object_frame_converter.hpp"
#include "rscl_access.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

void ObjectFrameSubscriber::SetParam(const TopicConfig& param) {
  param_ = param;
}

void ObjectFrameSubscriber::SetCallback(
    std::function<void(SapObjectFramePtr)> callback) {
  callback_ = std::move(callback);
}

bool ObjectFrameSubscriber::Init() {
  if (param_.topic.empty()) {
    return false;
  }
  // 注册 RSCL 原始消息回调；真正字段转换发生在 ObjectFrameCallback 中。
  return CRsclAccess::Instance()
             ->subscribe<senseAD::msg::perception::ObjectFrame>(
                 param_.topic, kRsclDefaultQueueSize,
                 &ObjectFrameSubscriber::ObjectFrameCallback, this) == 0;
}

void ObjectFrameSubscriber::ObjectFrameCallback(
    const std::shared_ptr<
        ReceivedMsg<senseAD::msg::perception::ObjectFrame>>& msg) {
  if (!msg || !callback_) {
    return;
  }

  SapObjectFramePtr output;
  if (senseauto::ConvertSenseAutoObjectFrameToSap(msg->Msg(), &output)) {
    // output 已经是 sapPerceptionObjectFrame，可直接进入 PlanningService。
    callback_(std::move(output));
  }
}

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
