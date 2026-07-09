// Ported from:
//   PredictionSender / PlanningDebugSender / DLPDiagSender in
//   adapter/perception_camera/resource/x86/config/pipeline/pilot_dlp_pipeline.yaml
// Purpose:
//   Keep ROS2 subscriptions/publications at the transport edge.

#include "ros2_transport.hpp"

namespace usharing_dlp_node {
namespace ros2 {

Ros2PlanningTransport::Ros2PlanningTransport(
    std::shared_ptr<rclcpp::Node> node)
    : node_(std::move(node)) {}

bool Ros2PlanningTransport::Init() {
  // 源代码差异说明：
  //   旧链路由 Receiver/Sender workshop 从 YAML 绑定 RSCL topic 和 Cap'n Proto
  //   convertor；ROS 版本应在本 transport 中绑定 ROS topic 和 ROS converter。
  //   当前工程先实现 RSCL transport，因此这里保留接口占位。
  return static_cast<bool>(node_);
}

void Ros2PlanningTransport::Start() {
  // ROS2 版本后续可在 app 层 rclcpp::spin，也可在这里启动 executor。
}

void Ros2PlanningTransport::Stop() {
  // 当前无 ROS subscription/publisher 资源需要显式释放。
}

void Ros2PlanningTransport::PublishPrediction(
    const sapPerceptionPlanningFrame& planning_frame) {
  (void)planning_frame;
  // Source mapping:
  //   PredictionConvertor + PredictionSender -> /prediction/objects_vd.
  // Concrete ROS message publication belongs here, not in SapPlanningEngine.
}

void Ros2PlanningTransport::PublishPlanningDebug(
    const sapPerceptionPlanningDebugInfo& debug_info,
    const PerceptionPlanningInfo& info) {
  (void)debug_info;
  (void)info;
  // Source mapping:
  //   PlanningDebugConvertor + PlanningDebugSender -> /UniAD/PlanningDebug.
}

void Ros2PlanningTransport::PublishDiag(const FaultCodes& fault_codes) {
  (void)fault_codes;
  // Source mapping:
  //   DLPDiagConvertor + DLPDiagSender -> /prediction/fault_codes_dlp.
}

}  // namespace ros2
}  // namespace usharing_dlp_node
