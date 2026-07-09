// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_sdmap_convertor.cpp
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_local_map_convertor.cpp
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_navi_topo_convertor.cpp
//   planning_bus/data_rscl/*
// Purpose:
//   RSCL SceneNaviMap subscriber boundary for Pilot DLP planning.
#pragma once

#include <functional>
#include <cstdint>
#include <memory>

#include <ad_msg_idl/l2plus_maplesslm/maplesslm_msgs.capnp.h>
#include <ad_rscl/ad_rscl.h>

#include "config/rscl_config.hpp"
#include "sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace rscl {
namespace subscribers {

// SceneNaviMap 的 RSCL 订阅器。旧 pipeline 对同一个 topic 配置了
// SDMap/LocalMap/NaviTopo 多个 convertor；这里合并为一次订阅后拆成
// 多个 sap* DTO，避免重复订阅同一 RSCL topic。
class SceneNaviMapSubscriber {
 public:
  // 单帧 SceneNaviMap 转换出的全部 SDK 输入。
  struct ConvertedSceneNaviMap {
    SapSDMapPtr sdmap;
    SapRoadStructurePtr road_structure;
    SapCrossInfoPtr cross_info;
    SapNaviTopoPtr navi_topo;
    uint64_t timestamp_ns{0};
  };

  // 保存 topics_cfg 中 scene_navi_map_cfg。
  void SetParam(const TopicConfig& param);
  // 注册拆分后的地图/导航 DTO 回调。
  void SetCallback(std::function<void(ConvertedSceneNaviMap)> callback);
  // 注册 RSCL subscribe。
  bool Init();

 private:
  // RSCL 回调入口；converter 负责一帧拆四路 sap*。
  void SceneNaviMapCallback(
      const std::shared_ptr<
          ReceivedMsg<senseAD::msg::maplesslm::SceneNaviMap>>& msg);

  TopicConfig param_;
  std::function<void(ConvertedSceneNaviMap)> callback_;
};

}  // namespace subscribers
}  // namespace rscl
}  // namespace usharing_dlp_node
