// Ported from:
//   adapter/perception_camera/src/convertors/{perception,planning}/*
// Purpose:
//   Test-only sap DTO allocation helpers for RSCL smoke tests without linking
//   the sap_camera runtime library.

#include "planning_sap/sap_types.hpp"

#include <cstring>

extern "C" {

int32_t InitsapPerceptionObjectFrame(sapPerceptionObjectFrame* frame) {
  if (frame == nullptr) {
    return -1;
  }
  std::memset(frame, 0, sizeof(*frame));
  return 0;
}

int32_t InitsapPerceptionLidarGopFrame(sapPerceptionLidarGopFrame* frame) {
  if (frame == nullptr) {
    return -1;
  }
  std::memset(frame, 0, sizeof(*frame));
  return 0;
}

int32_t InitsapPerceptionRoadFrame(sapPerceptionRoadFrame* frame) {
  if (frame == nullptr) {
    return -1;
  }
  std::memset(frame, 0, sizeof(*frame));
  return 0;
}

int32_t InitsapPerceptionFusionObjectFrame(
    sapPerceptionFusionObjectFrame* frame) {
  if (frame == nullptr) {
    return -1;
  }
  std::memset(frame, 0, sizeof(*frame));
  return 0;
}

int32_t InitsapLocalLocalizationInfo(sapLocalLocalizationInfo* info) {
  if (info == nullptr) {
    return -1;
  }
  std::memset(info, 0, sizeof(*info));
  return 0;
}

void ReleaseRoadFrameMemory(sapPerceptionRoadFrame* frame) {
  (void)frame;
}

void ReleaseFusionObjectFrameMemory(sapPerceptionFusionObjectFrame* frame) {
  (void)frame;
}

}  // extern "C"

namespace usharing_dlp_node {

SapObjectFramePtr MakeObjectFrame() {
  auto* frame = new sapPerceptionObjectFrame();
  InitsapPerceptionObjectFrame(frame);
  return SapObjectFramePtr(frame, [](sapPerceptionObjectFrame* value) {
    delete value;
  });
}

void ReleaseConvertedObjectFrame(const sapPerceptionObjectFrame* frame) {
  (void)frame;
}

SapLidarGopFramePtr MakeLidarGopFrame() {
  auto* frame = new sapPerceptionLidarGopFrame();
  InitsapPerceptionLidarGopFrame(frame);
  return SapLidarGopFramePtr(frame, [](sapPerceptionLidarGopFrame* value) {
    delete value;
  });
}

void ReleaseConvertedLidarGopFrame(
    const sapPerceptionLidarGopFrame* frame) {
  (void)frame;
}

SapOccFramePtr MakeOccFrame() {
  auto* frame = new sapPerceptionOccFrame();
  std::memset(frame, 0, sizeof(*frame));
  return SapOccFramePtr(frame, [](sapPerceptionOccFrame* value) {
    delete value;
  });
}

void ReleaseConvertedOccFrame(const sapPerceptionOccFrame* frame) {
  (void)frame;
}

SapPlanningFramePtr MakePlanningFrame() {
  auto* frame = new sapPerceptionPlanningFrame();
  std::memset(frame, 0, sizeof(*frame));
  return SapPlanningFramePtr(frame, [](sapPerceptionPlanningFrame* value) {
    delete value;
  });
}

SapPlanningDebugInfoPtr MakePlanningDebugInfo() {
  auto* info = new sapPerceptionPlanningDebugInfo();
  std::memset(info, 0, sizeof(*info));
  return SapPlanningDebugInfoPtr(
      info, [](sapPerceptionPlanningDebugInfo* value) { delete value; });
}

}  // namespace usharing_dlp_node
