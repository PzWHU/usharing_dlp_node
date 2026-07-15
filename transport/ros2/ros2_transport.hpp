// Ported from:
//   adapter/perception_camera/resource/x86/config/workshop/pilot_dlp_workshop.yaml
//   adapter/perception_camera/resource/x86/config/pipeline/pilot_dlp_pipeline.yaml
// Purpose:
//   ROS2 transport boundary. ROS dependencies must not escape this directory.
#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "transport/transport.hpp"

namespace usharing_dlp_node {
namespace ros2 {

// ROS2 transport 占位实现。ROS 相关 include 只能出现在本目录和未来
// main_ros2.cpp/ROS converter 中，不能扩散到 planning_core/planning_sap 层。
class Ros2PlanningTransport final : public IPlanningTransport {
 public:
  // node 由 app 层创建并注入，方便后续在同一进程中组合多个 transport。
  explicit Ros2PlanningTransport(std::shared_ptr<rclcpp::Node> node);

  // 当前只校验 node 有效；具体 ROS 订阅/发布需要在 ROS message 类型确定后补充。
  bool Init() override;
  void Start() override;
  void Stop() override;

  // 以下发布接口保持与 IPlanningTransport 一致，输入都是 sap*，不把
  // sap_camera SDK 或 PlanningService 绑定到 ROS message 类型。
  void PublishPrediction(
      const sapPerceptionPlanningFrame& planning_frame) override;
  void PublishPlanningDebug(
      const sapPerceptionPlanningDebugInfo& debug_info,
                            const PerceptionPlanningInfo& info) override;
  void PublishDiag(const FaultCodes& fault_codes) override;

 private:
  std::shared_ptr<rclcpp::Node> node_;
};

}  // namespace ros2
}  // namespace usharing_dlp_node
