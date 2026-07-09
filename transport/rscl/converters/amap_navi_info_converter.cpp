// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_amap_navi_info_convertor.cpp
// Purpose:
//   Convert AmapNaviInfo to sapAmapNaviInfo with standalone ownership cleanup.

#include "transport/rscl/converters/amap_navi_info_converter.hpp"

#include <cstdint>
#include <cstring>

#include "transport/rscl/converters/converter_common.hpp"

namespace usharing_dlp_node {
namespace senseauto {

using AmapNaviInfo = ::senseAD::msg::amap_navigation::AmapNaviInfo;

namespace {

using detail::CopyCapnpText;
using detail::NewArrayOrNull;

void ReleaseConvertedAmapNaviInfo(sapAmapNaviInfo* navi_info) {
  if (navi_info == nullptr) {
    return;
  }

  // AmapNaviInfo 的动态数组全部由本 converter 用 new[] 构造，
  // 因此释放逻辑也放在本文件内，避免依赖 SDK 未明确的释放函数。
  delete[] navi_info->laneInfo.foregroundLane;
  delete[] navi_info->laneInfo.backgroundLane;
  delete[] navi_info->cameraInfo;
  delete[] navi_info->trafficFacilityInfo;
  for (uint64_t i = 0; i < navi_info->trafficLightInfoNums; ++i) {
    delete[] navi_info->trafficLightInfo[i].trafficLightInfo;
  }
  delete[] navi_info->trafficLightInfo;
  for (uint64_t i = 0; i < navi_info->tollGateInfoNums; ++i) {
    delete[] navi_info->tollGateInfo[i].laneTypes;
  }
  delete[] navi_info->tollGateInfo;
  delete[] navi_info->electronicSpeedLimitInfo;
  delete[] navi_info->weatherInfo;
  delete[] navi_info->trafficEventInfo;
}

bool ConvertAmapNaviInfo(const AmapNaviInfo::Reader& msg,
                         sapAmapNaviInfo* navi_info) {
  if (navi_info == nullptr) {
    return false;
  }

  navi_info->header.seq = msg.getHeader().getSeq();
  // 保留高德消息原始 header 和导航状态字段，SDK 根据这些信息判断
  // 当前导航、限速、车道和事件状态。
  navi_info->header.time.nanoSec = msg.getHeader().getTime().getNanoSec();
  CopyCapnpText(msg.getHeader().getFrameId(), navi_info->header.frameId);
  navi_info->isOutOfRoute = msg.getIsOutOfRoute();
  navi_info->naviType = static_cast<uint64_t>(msg.getNaviType());
  navi_info->iconType = static_cast<uint64_t>(msg.getIconType());
  navi_info->curRetainDistance = msg.getCurRetainDistance();
  navi_info->allLength = msg.getAllLength();
  navi_info->pathRetainDistance = msg.getPathRetainDistance();

  const auto lane_info_msg = msg.getLaneInfo();
  const auto foreground_lane = lane_info_msg.getForegroundLane();
  auto& lane_info = navi_info->laneInfo;
  lane_info.foregroundLaneNums = foreground_lane.size();
  lane_info.foregroundLane =
      NewArrayOrNull<uint64_t>(lane_info.foregroundLaneNums);
  for (uint64_t i = 0; i < lane_info.foregroundLaneNums; ++i) {
    lane_info.foregroundLane[i] = static_cast<uint64_t>(foreground_lane[i]);
  }
  const auto background_lane = lane_info_msg.getBackgroundLane();
  lane_info.backgroundLaneNums = background_lane.size();
  lane_info.backgroundLane =
      NewArrayOrNull<uint64_t>(lane_info.backgroundLaneNums);
  for (uint64_t i = 0; i < lane_info.backgroundLaneNums; ++i) {
    lane_info.backgroundLane[i] = static_cast<uint64_t>(background_lane[i]);
  }

  navi_info->curSpeedLimit = msg.getCurSpeedLimit();
  navi_info->curLink = msg.getCurLink();
  navi_info->curStep = msg.getCurStep();
  navi_info->distanceToNextLink = msg.getDistanceToNextLink();
  CopyCapnpText(msg.getCurRoadNameNavi(), navi_info->curRoadNameNavi);
  navi_info->curRoadLevelForNavi =
      static_cast<uint64_t>(msg.getCurRoadLevelForNavi());

  const auto camera_infos = msg.getCameraInfo();
  navi_info->cameraInfoNums = camera_infos.size();
  navi_info->cameraInfo =
      NewArrayOrNull<sapAmapCameraInfo>(navi_info->cameraInfoNums);
  for (uint64_t i = 0; i < navi_info->cameraInfoNums; ++i) {
    navi_info->cameraInfo[i].cameraDistance =
        camera_infos[i].getCameraDistance();
    navi_info->cameraInfo[i].cameraSpeed = camera_infos[i].getCameraSpeed();
    navi_info->cameraInfo[i].cameraType =
        static_cast<uint64_t>(camera_infos[i].getCameraType());
  }

  const auto facility_infos = msg.getTrafficFacilityInfo();
  navi_info->trafficFacilityInfoNums = facility_infos.size();
  navi_info->trafficFacilityInfo =
      NewArrayOrNull<sapAmapTrafficFacilityInfo>(
          navi_info->trafficFacilityInfoNums);
  for (uint64_t i = 0; i < navi_info->trafficFacilityInfoNums; ++i) {
    navi_info->trafficFacilityInfo[i].trafficFacilityDistance =
        facility_infos[i].getTrafficFacilityDistance();
    navi_info->trafficFacilityInfo[i].trafficFacilitySpeed =
        facility_infos[i].getTrafficFacilitySpeed();
    navi_info->trafficFacilityInfo[i].trafficFacilityType =
        static_cast<uint64_t>(facility_infos[i].getTrafficFacilityType());
  }

  const auto traffic_light_infos = msg.getTrafficLightInfo();
  navi_info->trafficLightInfoNums = traffic_light_infos.size();
  navi_info->trafficLightInfo =
      NewArrayOrNull<sapAmapTrafficLightInfo>(
          navi_info->trafficLightInfoNums);
  for (uint64_t i = 0; i < navi_info->trafficLightInfoNums; ++i) {
    navi_info->trafficLightInfo[i].length =
        traffic_light_infos[i].getLength();
    navi_info->trafficLightInfo[i].linkIndex =
        traffic_light_infos[i].getLinkIndex();
    navi_info->trafficLightInfo[i].linkId =
        traffic_light_infos[i].getLinkId();
    const auto traffic_lights =
        traffic_light_infos[i].getTrafficLightInfo();
    navi_info->trafficLightInfo[i].trafficlightNums =
        traffic_lights.size();
    navi_info->trafficLightInfo[i].trafficLightInfo =
        NewArrayOrNull<sapAmapTrafficLight>(traffic_lights.size());
    for (uint64_t j = 0; j < traffic_lights.size(); ++j) {
      navi_info->trafficLightInfo[i].trafficLightInfo[j].startTime =
          traffic_lights[j].getStartTime();
      navi_info->trafficLightInfo[i].trafficLightInfo[j].endTime =
          traffic_lights[j].getEndTime();
      navi_info->trafficLightInfo[i].trafficLightInfo[j].trafficLightState =
          static_cast<uint64_t>(traffic_lights[j].getTrafficLightState());
    }
  }

  navi_info->entranceExit = static_cast<uint64_t>(msg.getEntranceExit());
  CopyCapnpText(msg.getNaviText(), navi_info->naviText);

  const auto traffic_info = msg.getTrafficInfo();
  navi_info->trafficInfo.length = traffic_info.getLength();
  navi_info->trafficInfo.startLinkIdx = traffic_info.getStartLinkIdx();
  navi_info->trafficInfo.endLinkIndex = traffic_info.getEndLinkIndex();
  navi_info->trafficInfo.startSegmentIdx = traffic_info.getStartSegmentIdx();
  navi_info->trafficInfo.endSegmentIdx = traffic_info.getEndSegmentIdx();
  navi_info->trafficInfo.trafficState =
      static_cast<uint64_t>(traffic_info.getTrafficState());
  navi_info->elevatedRoadStatus =
      static_cast<uint64_t>(msg.getElevatedRoadStatus());
  navi_info->parallelRoadStatus =
      static_cast<uint64_t>(msg.getParallelRoadStatus());

  const auto toll_gate_infos = msg.getTollGateInfo();
  navi_info->tollGateInfoNums = toll_gate_infos.size();
  navi_info->tollGateInfo =
      NewArrayOrNull<sapAmapTollGateInfo>(navi_info->tollGateInfoNums);
  for (uint64_t i = 0; i < navi_info->tollGateInfoNums; ++i) {
    navi_info->tollGateInfo[i].distance = toll_gate_infos[i].getDistance();
    const auto lane_types = toll_gate_infos[i].getLaneTypes();
    navi_info->tollGateInfo[i].laneNums = lane_types.size();
    navi_info->tollGateInfo[i].laneTypes =
        NewArrayOrNull<uint64_t>(lane_types.size());
    for (uint64_t j = 0; j < lane_types.size(); ++j) {
      navi_info->tollGateInfo[i].laneTypes[j] =
          static_cast<uint64_t>(lane_types[j]);
    }
  }

  const auto electronic_speed_limit_infos =
      msg.getElectronicSpeedLimitInfo();
  navi_info->electronicSpeedLimitInfoNums =
      electronic_speed_limit_infos.size();
  navi_info->electronicSpeedLimitInfo =
      NewArrayOrNull<sapAmapElectronicSpeedLimitInfo>(
          navi_info->electronicSpeedLimitInfoNums);
  for (uint64_t i = 0; i < navi_info->electronicSpeedLimitInfoNums; ++i) {
    navi_info->electronicSpeedLimitInfo[i].electronicSpeedLimitType =
        static_cast<uint64_t>(
            electronic_speed_limit_infos[i].getElectronicSpeedLimitType());
    navi_info->electronicSpeedLimitInfo[i].speedLimitValue =
        electronic_speed_limit_infos[i].getSpeedLimitValue();
    navi_info->electronicSpeedLimitInfo[i].electronicSpeedLimitDist =
        electronic_speed_limit_infos[i].getElectronicSpeedLimitDist();
  }

  const auto weather_infos = msg.getWeatherInfo();
  navi_info->weatherInfoNums = weather_infos.size();
  navi_info->weatherInfo =
      NewArrayOrNull<sapAmapWeatherInfo>(navi_info->weatherInfoNums);
  for (uint64_t i = 0; i < navi_info->weatherInfoNums; ++i) {
    CopyCapnpText(weather_infos[i].getWeatherName(),
                  navi_info->weatherInfo[i].weatherName);
    CopyCapnpText(weather_infos[i].getAlertLevelName(),
                  navi_info->weatherInfo[i].alertLevelName);
    navi_info->weatherInfo[i].weatherId =
        static_cast<uint64_t>(weather_infos[i].getWeatherId());
    navi_info->weatherInfo[i].alertLevelId =
        static_cast<uint64_t>(weather_infos[i].getAlertLevelId());
    navi_info->weatherInfo[i].distanceToCar =
        weather_infos[i].getDistanceToCar();
  }

  const auto traffic_event_infos = msg.getTrafficEventInfo();
  navi_info->trafficEventInfoNums = traffic_event_infos.size();
  navi_info->trafficEventInfo =
      NewArrayOrNull<sapAmapTrafficEventInfo>(
          navi_info->trafficEventInfoNums);
  for (uint64_t i = 0; i < navi_info->trafficEventInfoNums; ++i) {
    navi_info->trafficEventInfo[i].trafficEventType =
        static_cast<uint64_t>(
            traffic_event_infos[i].getTrafficEventType());
    navi_info->trafficEventInfo[i].occupiedLaneType =
        static_cast<uint64_t>(
            traffic_event_infos[i].getOccupiedLaneType());
    navi_info->trafficEventInfo[i].trafficEventDistance =
        traffic_event_infos[i].getTrafficEventDistance();
  }

  return true;
}

}  // namespace

bool ConvertSenseAutoAmapNaviInfoToSap(AmapNaviInfo::Reader msg,
                                       SapAmapNaviInfoPtr* output,
                                       uint64_t* timestamp_ns) {
  if (output == nullptr || timestamp_ns == nullptr) {
    return false;
  }

  auto frame = MakeSapInputPtr<sapAmapNaviInfo>(
      new sapAmapNaviInfo(), [](sapAmapNaviInfo* value) {
        ReleaseConvertedAmapNaviInfo(value);
      });
  std::memset(frame.get(), 0, sizeof(sapAmapNaviInfo));
  // 使用自定义 deleter 管理动态数组，调用方只持有 shared_ptr。
  if (!ConvertAmapNaviInfo(msg, frame.get())) {
    return false;
  }

  *timestamp_ns = msg.getHeader().getTime().getNanoSec();
  *output = frame;
  return true;
}

}  // namespace senseauto
}  // namespace usharing_dlp_node
