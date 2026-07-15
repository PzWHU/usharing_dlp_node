// Ported from:
//   adapter/perception_camera/include/perception_camera_proxy_traits.hpp
//   adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp
// Purpose:
//   Centralize sap* DTO aliases and SDK-owned output RAII rules.
//
// 中文说明：
//   sap* 是本移植版本的第一阶段内部 DTO。所有 transport/converter 都应该把
//   外部消息转换成这里定义的智能指针类型，再交给 SapPlanningEngine。
//   注意：部分 sap 结构内部包含裸指针数组，必须使用匹配的释放函数。
#pragma once

#include <functional>
#include <memory>

#include "perception_camera/sap_camera.h"

namespace usharing_dlp_node {

using SapObjectFramePtr = std::shared_ptr<sapPerceptionObjectFrame>;
using SapRoadFramePtr = std::shared_ptr<sapPerceptionRoadFrame>;
using SapFusionObjectFramePtr = std::shared_ptr<sapPerceptionFusionObjectFrame>;
using SapLidarGopFramePtr = std::shared_ptr<sapPerceptionLidarGopFrame>;
using SapOccFramePtr = std::shared_ptr<sapPerceptionOccFrame>;
using SapLocalLocalizationPtr = std::shared_ptr<sapLocalLocalizationInfo>;
using SapLocationInfoPtr = std::shared_ptr<sapLocationInfo>;
using SapSDMapPtr = std::shared_ptr<sapSDMap>;
using SapManualSignalPtr = std::shared_ptr<sapManualSignal>;
using SapCrossInfoPtr = std::shared_ptr<sapCrossInfo>;
using SapRoadStructurePtr = std::shared_ptr<sapRoadStructure>;
using SapNaviTopoPtr = std::shared_ptr<sapNaviTopo>;
using SapChassisStatePtr = std::shared_ptr<sapChassisState>;
using SapAmapNaviInfoPtr = std::shared_ptr<sapAmapNaviInfo>;
using SapPlanningFramePtr = std::shared_ptr<sapPerceptionPlanningFrame>;
using SapPlanningDebugInfoPtr = std::shared_ptr<sapPerceptionPlanningDebugInfo>;

SapObjectFramePtr MakeObjectFrame();
// 释放由 converter 分配的 ObjectFrame 内部数组。该函数只释放转换阶段申请的内存，
// 不应混用 SDK 输出释放函数。
void ReleaseConvertedObjectFrame(const sapPerceptionObjectFrame* frame);
SapLidarGopFramePtr MakeLidarGopFrame();
void ReleaseConvertedLidarGopFrame(
    const sapPerceptionLidarGopFrame* frame);
SapOccFramePtr MakeOccFrame();
void ReleaseConvertedOccFrame(const sapPerceptionOccFrame* frame);
// SDK 输出结构使用 SDK 提供的 ReleasePlanningFrameMemory /
// ReleasePlanningDebugInfoMemory 释放内部内存。
SapPlanningFramePtr MakePlanningFrame();
SapPlanningDebugInfoPtr MakePlanningDebugInfo();

template <typename T>
std::shared_ptr<T> MakeSapInputPtr(T* raw, std::function<void(T*)> deleter) {
  return std::shared_ptr<T>(raw, [deleter](T* value) {
    if (deleter) {
      deleter(value);
    }
    delete value;
  });
}

}  // namespace usharing_dlp_node
