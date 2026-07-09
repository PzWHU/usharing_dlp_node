// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_sdmap_convertor.cpp
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_local_map_convertor.cpp
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_navi_topo_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   Subscribe to SceneNaviMap RSCL input and forward converted map sap DTOs.

#include "transport/rscl/subscribers/scene_navi_map/scene_navi_map_subscriber.hpp"

#include <utility>

#include "transport/rscl/converters/scene_navi_map_converter.hpp"
#include "rscl_access.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

void SceneNaviMapSubscriber::SetParam(const TopicConfig& param) {
  param_ = param;
}

void SceneNaviMapSubscriber::SetCallback(
    std::function<void(ConvertedSceneNaviMap)> callback) {
  callback_ = std::move(callback);
}

bool SceneNaviMapSubscriber::Init() {
  if (param_.topic.empty()) {
    return false;
  }
  // 只订阅一次 SceneNaviMap，避免 SDMap/LocalMap/NaviTopo 重复消费同 topic。
  return CRsclAccess::Instance()
             ->subscribe<senseAD::msg::maplesslm::SceneNaviMap>(
                 param_.topic, kRsclDefaultQueueSize,
                 &SceneNaviMapSubscriber::SceneNaviMapCallback, this) == 0;
}

void SceneNaviMapSubscriber::SceneNaviMapCallback(
    const std::shared_ptr<
        ReceivedMsg<senseAD::msg::maplesslm::SceneNaviMap>>& msg) {
  if (!msg || !callback_) {
    return;
  }

  ConvertedSceneNaviMap output;
  if (senseauto::ConvertSenseAutoSceneNaviMapToSap(
          msg->Msg(), &output.sdmap, &output.road_structure,
          &output.cross_info, &output.navi_topo, &output.timestamp_ns)) {
    // ConvertedSceneNaviMap 会在 RsclPlanningTransport 中拆给多个 on_xxx。
    callback_(std::move(output));
  }
}

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
