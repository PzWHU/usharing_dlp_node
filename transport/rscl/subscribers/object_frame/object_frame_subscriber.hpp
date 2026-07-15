// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_object_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   RSCL ObjectFrame subscriber boundary for Pilot DLP planning.
#pragma once

#include <functional>
#include <memory>

#include <ad_msg_idl/perception/perception.capnp.h>
#include <ad_rscl/ad_rscl.h>

#include "config/rscl_config.hpp"
#include "planning_sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

// ObjectFrame 的 RSCL 订阅器。
//
// 职责边界：
//   1. 从 topics_cfg 指定的 RSCL topic 接收 ObjectFrame。
//   2. 调用 object_frame_converter 将 SenseAuto message 转成 sapPerceptionObjectFrame。
//   3. 通过 callback_ 把 sap* DTO 交给 RsclPlanningTransport。
// 不在本类中调用 sapPush*，也不写具体字段转换逻辑。
class ObjectFrameSubscriber {
 public:
  // 保存 topics_cfg 中 object_frame_cfg 的 topic/type/isPub 配置。
  void SetParam(const TopicConfig& param);
  // 注册转换完成后的内部回调。参数已经是 sap*，不是 RSCL message。
  void SetCallback(std::function<void(SapObjectFramePtr)> callback);
  // 向 CRsclAccess 注册 RSCL subscribe 回调。
  bool Init();

 private:
  // RSCL 原始回调入口，只做空指针检查、转换和上抛。
  void ObjectFrameCallback(
      const std::shared_ptr<
          ReceivedMsg<senseAD::msg::perception::ObjectFrame>>& msg);

  TopicConfig param_;
  std::function<void(SapObjectFramePtr)> callback_;
};

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
