// Ported from:
//   planning_bus/data_rscl/sensor.cpp
//   adapter/perception_camera/resource/x86/config/pipeline/pilot_dlp_pipeline.yaml
// Purpose:
//   Publish/subscribe through CRsclAccess while exposing only sap* DTOs to
//   PlanningService.

#include "rscl_planning_transport.hpp"

#include <utility>

#include <ad_msg_idl/ad_framework_sdk/diag_msg.capnp.h>
#include <ad_msg_idl/ad_prediction/prediction.capnp.h>
#include <ad_msg_idl/perception/perception.capnp.h>

#include "common/clock.hpp"
#include "transport/rscl/converters/diag_converter.hpp"
#include "transport/rscl/converters/planning_debug_converter.hpp"
#include "transport/rscl/converters/prediction_converter.hpp"
#include "rscl_access.hpp"

namespace usharing_dlp_node {
namespace rscl {

RsclPlanningTransport::RsclPlanningTransport(RsclPortConfig config)
    : config_(std::move(config)) {}

bool RsclPlanningTransport::Init() {
  // transport 初始化拆成订阅和发布两部分，避免 Init() 同时堆叠所有
  // topic 逻辑。后续新增 ROS/其他 transport 时也应保持相同接口语义。
  return InitSubscribers() && InitPublishers();
}

bool RsclPlanningTransport::InitSubscribers() {
  auto should_subscribe = [](const TopicConfig& topic) {
    return !topic.topic.empty();
  };

  // subscriber 层只关心“从哪个 RSCL topic 接收什么消息”；收到原始消息后
  // 立即调用对应 converter 转成 sap*，再通过 IPlanningTransport 回调上抛。
  object_frame_subscriber_.SetParam(config_.object_frame);
  object_frame_subscriber_.SetCallback([this](SapObjectFramePtr object_frame) {
    if (on_object) {
      on_object(std::move(object_frame));
    }
  });
  if (should_subscribe(config_.object_frame) &&
      !object_frame_subscriber_.Init()) {
    return false;
  }

  road_geometry_frame_subscriber_.SetParam(config_.road_geometry_frame);
  road_geometry_frame_subscriber_.SetCallback([this](SapRoadFramePtr road_frame) {
    if (on_road) {
      on_road(std::move(road_frame));
    }
  });
  if (should_subscribe(config_.road_geometry_frame) &&
      !road_geometry_frame_subscriber_.Init()) {
    return false;
  }

  fusion_object_frame_subscriber_.SetParam(config_.fusion_object_frame);
  fusion_object_frame_subscriber_.SetCallback(
      [this](SapFusionObjectFramePtr fusion_frame) {
        if (on_fusion_object) {
          on_fusion_object(std::move(fusion_frame));
        }
      });
  if (should_subscribe(config_.fusion_object_frame) &&
      !fusion_object_frame_subscriber_.Init()) {
    return false;
  }

  lidar_gop_frame_subscriber_.SetParam(config_.lidar_gop_frame);
  lidar_gop_frame_subscriber_.SetCallback(
      [this](SapLidarGopFramePtr lidar_gop_frame, uint64_t timestamp_ns) {
        if (on_lidar_gop) {
          on_lidar_gop(std::move(lidar_gop_frame), timestamp_ns);
        }
      });
  if (should_subscribe(config_.lidar_gop_frame) &&
      !lidar_gop_frame_subscriber_.Init()) {
    return false;
  }

  occupancy_info_subscriber_.SetParam(config_.occupancy_info);
  occupancy_info_subscriber_.SetCallback([this](SapOccFramePtr occ_frame) {
    if (on_occ) {
      on_occ(std::move(occ_frame));
    }
  });
  if (should_subscribe(config_.occupancy_info) &&
      !occupancy_info_subscriber_.Init()) {
    return false;
  }

  local_localization_subscriber_.SetParam(config_.local_localization);
  local_localization_subscriber_.SetCallback(
      [this](SapLocalLocalizationPtr local_pose) {
        if (on_local_localization) {
          on_local_localization(std::move(local_pose));
        }
      });
  if (should_subscribe(config_.local_localization) &&
      !local_localization_subscriber_.Init()) {
    return false;
  }

  location_info_subscriber_.SetParam(config_.location_info);
  location_info_subscriber_.SetCallback(
      [this](SapLocationInfoPtr location_info, uint64_t timestamp_ns) {
        if (on_location_info) {
          on_location_info(std::move(location_info), timestamp_ns);
        }
      });
  if (should_subscribe(config_.location_info) &&
      !location_info_subscriber_.Init()) {
    return false;
  }

  vehicle_report_subscriber_.SetParam(config_.vehicle_report);
  vehicle_report_subscriber_.SetCallback(
      [this](const VehicleInfo& vehicle_info, uint64_t timestamp_ns) {
        if (on_vehicle_info) {
          on_vehicle_info(vehicle_info, timestamp_ns);
        }
      });
  if (should_subscribe(config_.vehicle_report) &&
      !vehicle_report_subscriber_.Init()) {
    return false;
  }

  vehicle_processing_subscriber_.SetParam(config_.vehicle_processing);
  vehicle_processing_subscriber_.SetCallback(
      [this](SapChassisStatePtr chassis_state, uint64_t timestamp_ns) {
        if (on_chassis_state) {
          on_chassis_state(std::move(chassis_state), timestamp_ns);
        }
      });
  if (should_subscribe(config_.vehicle_processing) &&
      !vehicle_processing_subscriber_.Init()) {
    return false;
  }

  scene_navi_map_subscriber_.SetParam(config_.scene_navi_map);
  scene_navi_map_subscriber_.SetCallback(
      [this](
          subscribers::SceneNaviMapSubscriber::ConvertedSceneNaviMap map_data) {
        // 旧 pipeline 中 SceneNaviMap 一路输入被多个 convertor 拆成
        // SDMap、RoadStructure、CrossInfo、NaviTopo 多路 sap* 输入。
        if (on_sdmap) {
          on_sdmap(std::move(map_data.sdmap), map_data.timestamp_ns);
        }
        if (on_road_structure) {
          on_road_structure(std::move(map_data.road_structure),
                            map_data.timestamp_ns);
        }
        if (on_cross_info) {
          on_cross_info(std::move(map_data.cross_info), map_data.timestamp_ns);
        }
        if (on_navi_topo) {
          on_navi_topo(std::move(map_data.navi_topo), map_data.timestamp_ns);
        }
      });
  if (should_subscribe(config_.scene_navi_map) &&
      !scene_navi_map_subscriber_.Init()) {
    return false;
  }

  amap_navi_info_subscriber_.SetParam(config_.amap_navi_info);
  amap_navi_info_subscriber_.SetCallback(
      [this](SapAmapNaviInfoPtr amap_navi_info, uint64_t timestamp_ns) {
        if (on_amap_navi_info) {
          on_amap_navi_info(std::move(amap_navi_info), timestamp_ns);
        }
      });
  if (should_subscribe(config_.amap_navi_info) &&
      !amap_navi_info_subscriber_.Init()) {
    return false;
  }

  return true;
}

bool RsclPlanningTransport::InitPublishers() {
  // 输出 topic 是否发布由 topics_cfg 中 isPub/topic 控制。这里仅注册
  // RSCL publisher，具体字段填充放在 converters/ 下。
  if (config_.prediction_pub.is_pub &&
      !config_.prediction_pub.topic.empty()) {
    if (CRsclAccess::Instance()
            ->advertise<senseAD::msg::prediction::Prediction>(
                config_.prediction_pub.topic) != 0) {
      return false;
    }
  }

  if (config_.planning_debug_pub.is_pub &&
      !config_.planning_debug_pub.topic.empty()) {
    if (CRsclAccess::Instance()
            ->advertise<senseAD::msg::perception::UniADPlanningDebug>(
                config_.planning_debug_pub.topic) != 0) {
      return false;
    }
  }

  if (config_.diag_pub.is_pub && !config_.diag_pub.topic.empty()) {
    if (CRsclAccess::Instance()
            ->advertise<senseAD::msg::ad_framework_sdk::DiagMsg>(
                config_.diag_pub.topic) != 0) {
      return false;
    }
  }

  return true;
}

void RsclPlanningTransport::Start() {
  // RSCL 的回调线程由 runtime/subscribe 管理，本类没有额外 worker。
}

void RsclPlanningTransport::Stop() {
  // publisher/subscriber 生命周期由 CRsclAccess 单例在进程退出时统一释放。
}

void RsclPlanningTransport::PublishPrediction(
    const sapPerceptionPlanningFrame& planning_frame) {
  if (!config_.prediction_pub.is_pub || config_.prediction_pub.topic.empty()) {
    return;
  }

  // 输出方向与输入方向相反：SapPlanningEngine 产出 sap*，RSCL transport
  // 再转换为 SenseAuto Cap'n Proto message 并发布到配置 topic。
  auto output = CRsclAccess::NewMessage<senseAD::msg::prediction::Prediction>();
  auto builder = output->MsgBuilder();
  SystemClock clock;
  senseauto::ConvertPlanningFrameToSenseAutoPrediction(
      planning_frame, builder, clock.NowNs(), &sequence_state_);
  CRsclAccess::Instance()->publish(config_.prediction_pub.topic, output);
}

void RsclPlanningTransport::PublishPlanningDebug(
    const sapPerceptionPlanningDebugInfo& debug_info,
    const PerceptionPlanningInfo& info) {
  if (!config_.planning_debug_pub.is_pub ||
      config_.planning_debug_pub.topic.empty()) {
    return;
  }

  auto output =
      CRsclAccess::NewMessage<senseAD::msg::perception::UniADPlanningDebug>();
  auto builder = output->MsgBuilder();
  // debug converter 需要 SDK debug string 与车辆状态机中的自动驾驶状态。
  senseauto::ConvertPlanningDebugToSenseAuto(debug_info, info, builder);
  CRsclAccess::Instance()->publish(config_.planning_debug_pub.topic, output);
}

void RsclPlanningTransport::PublishDiag(const FaultCodes& fault_codes) {
  if (!config_.diag_pub.is_pub || config_.diag_pub.topic.empty()) {
    return;
  }

  auto output =
      CRsclAccess::NewMessage<senseAD::msg::ad_framework_sdk::DiagMsg>();
  auto builder = output->MsgBuilder();
  // 诊断消息保留独立序列号状态，和 prediction 输出序列互不影响。
  SystemClock clock;
  senseauto::ConvertFaultCodesToSenseAutoDiag(
      fault_codes, builder, clock.NowNs(), &sequence_state_);
  CRsclAccess::Instance()->publish(config_.diag_pub.topic, output);
}

}  // namespace rscl
}  // namespace usharing_dlp_node
