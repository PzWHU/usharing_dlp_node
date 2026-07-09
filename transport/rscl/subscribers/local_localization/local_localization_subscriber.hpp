// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_local_localization_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   RSCL OdomStateInfo subscriber boundary for Pilot DLP planning.
#pragma once

#include <functional>
#include <memory>

#include <ad_msg_idl/ad_localization/localization.capnp.h>
#include <ad_rscl/ad_rscl.h>

#include "config/rscl_config.hpp"
#include "sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

// OdomStateInfo 的 RSCL 订阅器，输出 sapLocalLocalizationInfo。
class LocalLocalizationSubscriber {
 public:
  // 保存 topics_cfg 中 local_localization_cfg。
  void SetParam(const TopicConfig& param);
  // 注册局部定位 sap* 输出回调。
  void SetCallback(std::function<void(SapLocalLocalizationPtr)> callback);
  // 注册 RSCL subscribe。
  bool Init();

 private:
  // RSCL 回调入口；位姿字段转换由 converter 完成。
  void LocalLocalizationCallback(
      const std::shared_ptr<
          ReceivedMsg<senseAD::msg::localization::OdomStateInfo>>& msg);

  TopicConfig param_;
  std::function<void(SapLocalLocalizationPtr)> callback_;
};

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
