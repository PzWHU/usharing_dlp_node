// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_amap_navi_info_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   RSCL AmapNaviInfo subscriber boundary for Pilot DLP planning.
#pragma once

#include <functional>
#include <cstdint>
#include <memory>

#include <ad_msg_idl/ad_navi/Amap_navi_msgs.capnp.h>
#include <ad_rscl/ad_rscl.h>

#include "config/rscl_config.hpp"
#include "planning_sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

// 高德导航 AmapNaviInfo 的 RSCL 订阅器，输出 sapAmapNaviInfo。
class AmapNaviInfoSubscriber {
 public:
  // 保存 topics_cfg 中 amap_navi_info_cfg。
  void SetParam(const TopicConfig& param);
  // 注册转换完成后的导航 DTO 回调。
  void SetCallback(
      std::function<void(SapAmapNaviInfoPtr, uint64_t)> callback);
  // 注册 RSCL subscribe。
  bool Init();

 private:
  // RSCL 回调入口；转换失败时本帧不继续传递。
  void AmapNaviInfoCallback(
      const std::shared_ptr<
          ReceivedMsg<senseAD::msg::amap_navigation::AmapNaviInfo>>& msg);

  TopicConfig param_;
  std::function<void(SapAmapNaviInfoPtr, uint64_t)> callback_;
};

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
