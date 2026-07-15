// Ported from:
//   PerceptionCameraSdkWorkshop::PublishPlanning()
//   adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp
// Purpose:
//   Keep SDK-owned planning output memory release rules in one place.

#include "planning_sap/sap_types.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace usharing_dlp_node {

SapObjectFramePtr MakeObjectFrame() {
  auto* frame = new sapPerceptionObjectFrame();
  InitsapPerceptionObjectFrame(frame);
  // 输入 object frame 的数组由本工程 converter 使用 new[] 构造，不能交给
  // SDK 的输出释放函数；因此使用自定义 deleter 成对 delete[]。
  return SapObjectFramePtr(frame, [](sapPerceptionObjectFrame* value) {
    ReleaseConvertedObjectFrame(value);
    delete value;
  });
}

void ReleaseConvertedObjectFrame(const sapPerceptionObjectFrame* frame) {
  if (frame == nullptr) {
    return;
  }

  // 旧 ObjectFrame convertor 为同一类对象的 landmark、score、lane
  // assignment 分配一块连续内存，再把不同字段指向其中的偏移位置。
  // 释放时只能 delete[] 一次，因此下面选择该连续块的首个非空指针。
  for (std::size_t i = 0; i < frame->veh_num; ++i) {
    const auto& vehicle = frame->vehicle[i];
    if (vehicle.base_info.bv_corners.bv_corners != nullptr) {
      delete[] reinterpret_cast<uint8_t*>(
          vehicle.base_info.bv_corners.bv_corners);
    }
    auto* landmark = reinterpret_cast<uint8_t*>(vehicle.attribute.landmark);
    auto* landmark_scores =
        reinterpret_cast<uint8_t*>(vehicle.attribute.landmark_scores);
    auto* lane_assignments =
        reinterpret_cast<uint8_t*>(vehicle.lane_assignments);
    if (landmark != nullptr || landmark_scores != nullptr ||
        lane_assignments != nullptr) {
      delete[] (landmark != nullptr
                    ? landmark
                    : (landmark_scores != nullptr ? landmark_scores
                                                  : lane_assignments));
    }
  }

  for (std::size_t i = 0; i < frame->non_veh_num; ++i) {
    const auto& non_vehicle = frame->non_vehicle[i];
    if (non_vehicle.base_info.bv_corners.bv_corners != nullptr) {
      delete[] reinterpret_cast<uint8_t*>(
          non_vehicle.base_info.bv_corners.bv_corners);
    }
    auto* landmark = reinterpret_cast<uint8_t*>(non_vehicle.attribute.landmark);
    auto* landmark_scores =
        reinterpret_cast<uint8_t*>(non_vehicle.attribute.landmark_scores);
    if (landmark != nullptr || landmark_scores != nullptr) {
      delete[] (landmark != nullptr ? landmark : landmark_scores);
    }
  }

  for (std::size_t i = 0; i < frame->ped_num; ++i) {
    const auto& pedestrian = frame->pedestrian[i];
    if (pedestrian.base_info.bv_corners.bv_corners != nullptr) {
      delete[] reinterpret_cast<uint8_t*>(
          pedestrian.base_info.bv_corners.bv_corners);
    }
    auto* landmark = reinterpret_cast<uint8_t*>(pedestrian.attribute.landmark);
    auto* landmark_scores =
        reinterpret_cast<uint8_t*>(pedestrian.attribute.landmark_scores);
    if (landmark != nullptr || landmark_scores != nullptr) {
      delete[] (landmark != nullptr ? landmark : landmark_scores);
    }
  }

  // vehicle/non_vehicle/pedestrian 三个数组同样可能来自一块连续内存，
  // 与原 convertor 的分配方式保持配对释放。
  auto* vehicle = reinterpret_cast<uint8_t*>(frame->vehicle);
  auto* non_vehicle = reinterpret_cast<uint8_t*>(frame->non_vehicle);
  auto* pedestrian = reinterpret_cast<uint8_t*>(frame->pedestrian);
  if (vehicle != nullptr || non_vehicle != nullptr || pedestrian != nullptr) {
    delete[] (vehicle != nullptr ? vehicle
                                : (non_vehicle != nullptr ? non_vehicle
                                                          : pedestrian));
  }
}

SapLidarGopFramePtr MakeLidarGopFrame() {
  auto* frame = new sapPerceptionLidarGopFrame();
  InitsapPerceptionLidarGopFrame(frame);
  return SapLidarGopFramePtr(frame, [](sapPerceptionLidarGopFrame* value) {
    ReleaseConvertedLidarGopFrame(value);
    delete value;
  });
}

void ReleaseConvertedLidarGopFrame(
    const sapPerceptionLidarGopFrame* frame) {
  if (frame == nullptr) {
    return;
  }
  delete[] frame->lidar_object;
  delete[] frame->decoded_lidar_object;
}

SapOccFramePtr MakeOccFrame() {
  auto* frame = new sapPerceptionOccFrame();
  std::memset(frame, 0, sizeof(sapPerceptionOccFrame));
  // OCC 输入结构在 converter 内按 SDK DTO 填充，使用本地 deleter 管理
  // partition ranges 和 occ_grids。
  return SapOccFramePtr(frame, [](sapPerceptionOccFrame* value) {
    ReleaseConvertedOccFrame(value);
    delete value;
  });
}

void ReleaseConvertedOccFrame(const sapPerceptionOccFrame* frame) {
  if (frame == nullptr || frame->use_vd_occ) {
    // use_vd_occ 分支没有由本工程分配 occupancy_info_results 内存。
    return;
  }

  const auto& occupancy_info = frame->occ_struct.occupancy_info_results;
  auto* x_ranges =
      reinterpret_cast<uint8_t*>(occupancy_info.x_partition_info.ranges);
  auto* y_ranges =
      reinterpret_cast<uint8_t*>(occupancy_info.y_partition_info.ranges);
  auto* z_ranges =
      reinterpret_cast<uint8_t*>(occupancy_info.z_partition_info.ranges);
  if (x_ranges != nullptr || y_ranges != nullptr || z_ranges != nullptr) {
    delete[] (x_ranges != nullptr ? x_ranges
                                  : (y_ranges != nullptr ? y_ranges
                                                         : z_ranges));
  }

  if (occupancy_info.occ_grids != nullptr) {
    delete[] occupancy_info.occ_grids;
  }
}

SapPlanningFramePtr MakePlanningFrame() {
  auto* frame = new sapPerceptionPlanningFrame();
  InitsapPerceptionPlanningFrame(frame);
  // planning 输出由 sapGetPlanningFrame 填充，必须用 SDK 提供的
  // ReleasePlanningFrameMemory 释放内部数组。
  return SapPlanningFramePtr(frame, [](sapPerceptionPlanningFrame* value) {
    ReleasePlanningFrameMemory(value);
    delete value;
  });
}

SapPlanningDebugInfoPtr MakePlanningDebugInfo() {
  auto* info = new sapPerceptionPlanningDebugInfo();
  InitsapPerceptionPlanningDebugInfo(info);
  // debug 输出同样是 SDK 分配/填充的结构，释放规则集中在这里。
  return SapPlanningDebugInfoPtr(info,
                                 [](sapPerceptionPlanningDebugInfo* value) {
                                   ReleasePlanningDebugInfoMemory(value);
                                   delete value;
                                 });
}

}  // namespace usharing_dlp_node
