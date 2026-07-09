// Ported from:
//   planning_bus/data_rscl/sensor.cpp
//   planning_bus/data_rscl/rscl_access/rscl_access.h
// Purpose:
//   RSCL communication boundary for Pilot DLP planning. RSCL message types and
//   CRsclAccess are intentionally kept out of planning core and sap adapter.
//
// 中文说明：
//   RsclPlanningTransport 是 RSCL 版 IPlanningTransport 实现：
//   - subscribers/ 负责 CRsclAccess::subscribe 和原始 RSCL 消息回调；
//   - converters/ 负责 SenseAuto capnp 消息与 sap* 的字段映射；
//   - 本类负责把各 subscriber 的输出接到 IPlanningTransport::on_xxx，
//     以及把 SDK 输出转换回 SenseAuto 消息并 publish。
#pragma once

#include "config/rscl_config.hpp"
#include "transport/rscl/converters/output_sequence_state.hpp"
#include "transport/rscl/subscribers/amap_navi_info/amap_navi_info_subscriber.hpp"
#include "transport/rscl/subscribers/fusion_object_frame/fusion_object_frame_subscriber.hpp"
#include "transport/rscl/subscribers/lidar_gop_frame/lidar_gop_frame_subscriber.hpp"
#include "transport/rscl/subscribers/local_localization/local_localization_subscriber.hpp"
#include "transport/rscl/subscribers/location_info/location_info_subscriber.hpp"
#include "transport/rscl/subscribers/object_frame/object_frame_subscriber.hpp"
#include "transport/rscl/subscribers/occupancy_info/occupancy_info_subscriber.hpp"
#include "transport/rscl/subscribers/road_geometry_frame/road_geometry_frame_subscriber.hpp"
#include "transport/rscl/subscribers/scene_navi_map/scene_navi_map_subscriber.hpp"
#include "transport/rscl/subscribers/vehicle_processing/vehicle_processing_subscriber.hpp"
#include "transport/rscl/subscribers/vehicle_report/vehicle_report_subscriber.hpp"
#include "transport/transport.hpp"

namespace usharing_dlp_node {
namespace rscl {

class RsclPlanningTransport final : public IPlanningTransport {
 public:
  explicit RsclPlanningTransport(RsclPortConfig config);

  bool Init() override;
  void Start() override;
  void Stop() override;

  void PublishPrediction(
      const sapPerceptionPlanningFrame& planning_frame) override;
  void PublishPlanningDebug(
      const sapPerceptionPlanningDebugInfo& debug_info,
                            const PerceptionPlanningInfo& info) override;
  void PublishDiag(const FaultCodes& fault_codes) override;

 private:
  // 初始化所有输入订阅。每个 subscriber 成功 Init 后由 RSCL runtime 触发回调。
  bool InitSubscribers();
  // 初始化输出发布者。只有 topics_cfg 中启用且 topic 非空的输出会 advertise。
  bool InitPublishers();

  RsclPortConfig config_;
  subscribers::ObjectFrameSubscriber object_frame_subscriber_;
  subscribers::RoadGeometryFrameSubscriber road_geometry_frame_subscriber_;
  subscribers::FusionObjectFrameSubscriber fusion_object_frame_subscriber_;
  subscribers::LidarGopFrameSubscriber lidar_gop_frame_subscriber_;
  subscribers::OccupancyInfoSubscriber occupancy_info_subscriber_;
  subscribers::LocalLocalizationSubscriber local_localization_subscriber_;
  subscribers::LocationInfoSubscriber location_info_subscriber_;
  subscribers::VehicleReportSubscriber vehicle_report_subscriber_;
  subscribers::VehicleProcessingSubscriber vehicle_processing_subscriber_;
  subscribers::SceneNaviMapSubscriber scene_navi_map_subscriber_;
  subscribers::AmapNaviInfoSubscriber amap_navi_info_subscriber_;
  senseauto::SenseAutoOutputSequenceState sequence_state_;
};

}  // namespace rscl
}  // namespace usharing_dlp_node
