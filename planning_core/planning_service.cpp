// Ported from:
//   adapter/perception_camera/resource/x86/config/pipeline/pilot_dlp_pipeline.yaml
//   adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp
// Purpose:
//   Replace SWCFL pipeline wiring with explicit middleware-neutral callbacks.

#include "planning_core/planning_service.hpp"

#include <utility>

namespace usharing_dlp_node {
namespace {

void ClearEngineCallbacks(IPlanningEngine* engine) {
  if (engine == nullptr) {
    return;
  }
  engine->on_prediction = nullptr;
  engine->on_debug = nullptr;
  engine->on_diag = nullptr;
}

void ClearTransportCallbacks(IPlanningTransport* transport) {
  if (transport == nullptr) {
    return;
  }
  transport->on_object = nullptr;
  transport->on_road = nullptr;
  transport->on_fusion_object = nullptr;
  transport->on_lidar_gop = nullptr;
  transport->on_occ = nullptr;
  transport->on_local_localization = nullptr;
  transport->on_location_info = nullptr;
  transport->on_vehicle_info = nullptr;
  transport->on_sdmap = nullptr;
  transport->on_manual_signal = nullptr;
  transport->on_cross_info = nullptr;
  transport->on_road_structure = nullptr;
  transport->on_navi_topo = nullptr;
  transport->on_chassis_state = nullptr;
  transport->on_amap_navi_info = nullptr;
}

void ClearCallbacks(IPlanningEngine* engine, IPlanningTransport* transport) {
  ClearEngineCallbacks(engine);
  ClearTransportCallbacks(transport);
}

}  // namespace

bool PlanningService::Init(const PlanningConfig& config,
                           std::unique_ptr<IPlanningEngine> engine,
                           std::unique_ptr<IPlanningTransport> transport) {
  if (!engine || !transport) {
    return false;
  }

  engine_ = std::move(engine);
  transport_ = std::move(transport);

  // SDK 输出回调绑定：SapPlanningEngine 只产出 sap* 结果，不直接知道
  // RSCL/ROS topic；PlanningService 在这里把结果转交给当前 transport 发布。
  engine_->on_prediction = [this](SapPlanningFramePtr planning_frame) {
    if (transport_ && planning_frame) {
      transport_->PublishPrediction(*planning_frame);
    }
  };
  engine_->on_debug = [this](SapPlanningDebugInfoPtr debug_info,
                             PerceptionPlanningInfo info) {
    if (transport_ && debug_info) {
      transport_->PublishPlanningDebug(*debug_info, info);
    }
  };
  engine_->on_diag = [this](const FaultCodes& fault_codes) {
    if (transport_) {
      transport_->PublishDiag(fault_codes);
    }
  };

  // 输入回调绑定：transport 已经完成“中间件消息 -> sap*”转换，
  // 这里仅负责把内部数据传输对象移交给 engine，对应旧代码中的 OnDataArrived()分发逻辑
  transport_->on_object = [this](SapObjectFramePtr object_frame) {
    if (engine_) engine_->PushObject(std::move(object_frame));
  };
  transport_->on_road = [this](SapRoadFramePtr road_frame) {
    if (engine_) engine_->PushRoad(std::move(road_frame));
  };
  transport_->on_fusion_object = [this](SapFusionObjectFramePtr fusion_frame) {
    if (engine_) engine_->PushFusionObject(std::move(fusion_frame));
  };
  transport_->on_lidar_gop = [this](SapLidarGopFramePtr lidar_gop_frame,
                                    uint64_t timestamp_ns) {
    if (engine_) {
      engine_->PushLidarGop(std::move(lidar_gop_frame), timestamp_ns);
    }
  };
  transport_->on_occ = [this](SapOccFramePtr occ_frame) {
    if (engine_) engine_->PushOcc(std::move(occ_frame));
  };
  transport_->on_local_localization =
      [this](SapLocalLocalizationPtr local_pose) {
        if (engine_) engine_->PushLocalLocalization(std::move(local_pose));
      };
  transport_->on_location_info = [this](SapLocationInfoPtr location_info,
                                        uint64_t timestamp_ns) {
    if (engine_) {
      engine_->PushLocationInfo(std::move(location_info), timestamp_ns);
    }
  };
  transport_->on_vehicle_info = [this](const VehicleInfo& vehicle_info,
                                       uint64_t timestamp_ns) {
    if (!engine_) return;
    // VehicleReport 不直接进入 SDK；旧链路先在 OnVehicleInfo() 中维护
    // 自动驾驶状态、灯光、接管等业务状态，再生成 sapManualSignal。
    auto manual_signal = vehicle_state_machine_.OnVehicleInfo(vehicle_info);
    engine_->SetPlanningInfo(vehicle_state_machine_.planning_info());
    engine_->PushManualSignal(std::move(manual_signal), timestamp_ns);
  };
  transport_->on_sdmap = [this](SapSDMapPtr sdmap, uint64_t timestamp_ns) {
    if (engine_) engine_->PushSDMap(std::move(sdmap), timestamp_ns);
  };
  transport_->on_manual_signal = [this](SapManualSignalPtr manual_signal,
                                        uint64_t timestamp_ns) {
    if (engine_) {
      engine_->PushManualSignal(std::move(manual_signal), timestamp_ns);
    }
  };
  transport_->on_cross_info = [this](SapCrossInfoPtr cross_info,
                                     uint64_t timestamp_ns) {
    if (engine_) engine_->PushCrossInfo(std::move(cross_info), timestamp_ns);
  };
  transport_->on_road_structure = [this](SapRoadStructurePtr road_structure,
                                         uint64_t timestamp_ns) {
    if (engine_) {
      engine_->PushRoadStructure(std::move(road_structure), timestamp_ns);
    }
  };
  transport_->on_navi_topo = [this](SapNaviTopoPtr navi_topo,
                                    uint64_t timestamp_ns) {
    if (engine_) engine_->PushNaviTopo(std::move(navi_topo), timestamp_ns);
  };
  transport_->on_chassis_state = [this](SapChassisStatePtr chassis_state,
                                        uint64_t timestamp_ns) {
    if (!engine_ || !chassis_state) return;
    // ChassisState 中的转向灯和 set_speed 会影响后续 VehicleReport
    // 生成的 manual signal/debug 信息，因此先更新状态机再 push 给 SDK。
    vehicle_state_machine_.UpdateChassisState(*chassis_state);
    engine_->SetPlanningInfo(vehicle_state_machine_.planning_info());
    engine_->PushChassisState(std::move(chassis_state), timestamp_ns);
  };
  transport_->on_amap_navi_info = [this](SapAmapNaviInfoPtr amap_navi_info,
                                         uint64_t timestamp_ns) {
    if (engine_) {
      engine_->PushAmapNaviInfo(std::move(amap_navi_info), timestamp_ns);
    }
  };

  // 先初始化 SDK，再注册/启动订阅。RSCL subscribe 后可能立即触发回调，
  // 该顺序可以保证输入到来时 sap_camera handle 已经可用。
  if (!engine_->Init(config)) {
    ClearCallbacks(engine_.get(), transport_.get());
    return false;
  }
  if (!transport_->Init()) {
    transport_->Stop();
    engine_->Stop();
    ClearCallbacks(engine_.get(), transport_.get());
    return false;
  }
  return true;
}

void PlanningService::Start() {
  // 对 RSCL 版本而言 Start 当前为空；保留该入口是为了兼容 ROS 或
  // 其他需要显式 spin/start 的 transport。
  if (transport_) {
    transport_->Start();
  }
}

void PlanningService::Stop() {
  if (transport_) {
    transport_->Stop();
  }
  if (engine_) {
    engine_->Stop();
  }
  ClearCallbacks(engine_.get(), transport_.get());
}

}  // namespace usharing_dlp_node
