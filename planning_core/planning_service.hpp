// Ported from:
//   adapter/perception_camera/resource/x86/config/pipeline/pilot_dlp_pipeline.yaml
//   adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp
// Purpose:
//   Wire transport callbacks to the sap planning engine without RSCL/SWCFL.
//
// 中文说明：
//   PlanningService 是业务编排层，职责是把“通信层事件”和“算法引擎接口”
//   连接起来：
//   - transport 收到输入 sap* 后，通过 on_xxx 回调交给 engine->PushXxx；
//   - engine 产生 prediction/debug/diag 后，通过 on_xxx 回调交给 transport 发布。
//   该层禁止依赖 RSCL/ROS，后续切换中间件时不应修改这里的业务连接关系。
#pragma once

#include <memory>

#include "planning_core/planning_engine.hpp"
#include "transport/transport.hpp"
#include "planning_core/vehicle_state_machine.hpp"

namespace usharing_dlp_node {

class PlanningService {
 public:
  // 注入算法引擎和通信实现，并注册输入/输出回调链。
  // engine 与 transport 的所有权在 Init 后转移给 PlanningService。
  bool Init(const PlanningConfig& config,
            std::unique_ptr<IPlanningEngine> engine,
            std::unique_ptr<IPlanningTransport> transport);
  // 启动通信侧；当前 RSCL 实现中 Start 为空，但 ROS 或其他中间件可能需要 spin。
  void Start();
  // 停止顺序固定为先停通信，再停 SDK，避免 stop 过程中继续向 SDK push 数据。
  void Stop();

 private:
  std::unique_ptr<IPlanningEngine> engine_;
  std::unique_ptr<IPlanningTransport> transport_;
  VehicleStateMachine vehicle_state_machine_;
};

}  // namespace usharing_dlp_node
