// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_fusion_object_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   RSCL fusion ObjectFrame subscriber boundary for Pilot DLP planning.
#pragma once

#include <functional>
#include <memory>

#include <ad_msg_idl/perception/perception.capnp.h>
#include <ad_rscl/ad_rscl.h>

#include "config/rscl_config.hpp"
#include "sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

// 融合 ObjectFrame 的 RSCL 订阅器。输入 schema 仍是 ObjectFrame，
// 但输出 DTO 是 sapPerceptionFusionObjectFrame。
class FusionObjectFrameSubscriber {
 public:
  // 保存 topics_cfg 中 fusion_object_frame_cfg。
  void SetParam(const TopicConfig& param);
  // 注册转换完成后的 fusion sap* 回调。
  void SetCallback(std::function<void(SapFusionObjectFramePtr)> callback);
  // 注册 RSCL subscribe。
  bool Init();

 private:
  // RSCL 回调入口；不直接访问 SDK。
  void FusionObjectFrameCallback(
      const std::shared_ptr<
          ReceivedMsg<senseAD::msg::perception::ObjectFrame>>& msg);

  TopicConfig param_;
  std::function<void(SapFusionObjectFramePtr)> callback_;
};

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
