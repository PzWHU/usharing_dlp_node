// Ported from:
//   planning_bus/data_rscl/position_data/position_data.cpp
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_global_localization_convertor.cpp
// Purpose:
//   Receive RSCL Ins messages and convert them to the middleware-neutral
//   internal sapLocationInfo DTO before handing data to PlanningService.
#pragma once

#include <functional>
#include <cstdint>
#include <memory>

#include <ad_msg_idl/ad_sensor/sensor.capnp.h>
#include <ad_rscl/ad_rscl.h>

#include "config/rscl_config.hpp"
#include "sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

// INS/global localization 的 RSCL 订阅器，输出 sapLocationInfo 和时间戳。
class LocationInfoSubscriber {
 public:
  // 保存 topics_cfg 中 location_info_cfg。
  void SetParam(const TopicConfig& param);
  // 注册定位转换完成后的回调；timestamp_ns 随 sapLocationInfo 一起上抛。
  void SetCallback(
      std::function<void(SapLocationInfoPtr, uint64_t)> callback);
  // 注册 RSCL subscribe。
  bool Init();

 private:
  // RSCL 回调入口；只负责调用 converter 和 callback_。
  void LocationInfoCallback(
      const std::shared_ptr<ReceivedMsg<senseAD::msg::sensor::Ins>>& msg);

  TopicConfig param_;
  std::function<void(SapLocationInfoPtr, uint64_t)> callback_;
};

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
