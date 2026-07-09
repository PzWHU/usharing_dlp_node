// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_object_convertor.cpp
// Purpose:
//   Keep object label/subtype and sapBaseObject mapping in one place for
//   ObjectFrame and FusionObjectFrame converters.

#include "transport/rscl/converters/object_common.hpp"

#include <cstddef>
#include <cstdint>

namespace usharing_dlp_node {
namespace senseauto {
namespace detail {

namespace {

using Object = ::senseAD::msg::perception::Object;

// Source mapping:
//   senseAD::rscl_ad::convertMsgLabelToPerceptionLabel()
//   senseAD::rscl_ad::convertMsgSubTypeToPerceptionSubType()
//   adapter/perception_camera/include/message_helper/message_object_label.hpp
// Difference from source:
//   The original helper includes perception_common logging macros and an
//   ObjectLabel revision that is not ABI/header-compatible with the standalone
//   planning port. Keep the planning converter independent by mapping the
//   message numeric labels to the sapBaseObject numeric ObjectLabel contract
//   locally. Unknown or unmapped labels intentionally fall back to UNKNOWN.
enum class SapObjectLabel : int64_t {
  kDefault = -3,
  kPlaceHolder = -2,
  kObjectAll = -1,
  kUnknown = 0,
  kPedestrian = 1,
  kVehicle = 10001,
  kVehicleCar = 10002,
  kVehicleSuv = 10003,
  kVehicleTruck = 10012,
  kVehicleBus = 10008,
  kVehicleTricycle = 10009,
  kVehicleOthers = 10011,
  kVehicleUnpoweredCar = 10034,
  kVehicleEnd = 10035,
  kBike = 20001,
  kBikeBicycle = 20003,
  kBikeMotor = 20006,
  kCyclist = 20007,
  kBikeEnd = 20009,
  kAnimalAll = 30001,
  kAnimalCat = 30002,
  kAnimalDog = 30003,
  kAnimalCow = 30004,
  kAnimalSmall = 30005,
  kAnimalBig = 30006,
  kAnimalEnd = 30007,
  kTrafficLightAll = 40001,
  kTrafficLightOther = 40002,
  kTrafficLightRed = 40003,
  kTrafficLightYellow = 40004,
  kTrafficLightGreen = 40005,
  kTrafficLightBlack = 40006,
  kTrafficLightGreenNumber = 40007,
  kTrafficSignAll = 50001,
  kTrafficSignXParking = 50130,
  kObstacleAll = 60001,
  kGate = 60002,
  kTrain = 60003,
  kWarningPillar = 60004,
  kRoadCone = 60005,
  kWarningTriangle = 60006,
  kWarningFlare = 60007,
  kPole = 60008,
  kObstacleArrowLeft = 60009,
  kObstacleArrowRight = 60010,
  kCushionDrumC = 60011,
  kCushionDrumR = 60012,
  kGuardrail = 60013,
  kRainwayGate = 60014,
  kCart = 60015,
  kConstructionAhead = 60016,
  kConstructionSign = 60017,
  kConstructionSignClosures = 60018,
  kConstructionSignRightLaneClosures = 60019,
  kConstructionSignLeftLaneClosures = 60020,
  kConstructionSignTurnRight = 60021,
  kConstructionSignTurnLeft = 60022,
  kObstacleIsolationBarrel = 60023,
  kObstacleBarrier = 60024,
  kObstacleCone = 60025,
  kObstaclePole = 60026,
  kObstacleWarningTriangle = 60027,
  kObstaclePillar = 60028,
  kObstacleOpenGate = 60029,
  kObstacleCloseGate = 60030,
  kObstacleEnd = 60031,
  kRoadMarkerAll = 70001,
  kRoadMarkerForward = 70002,
  kRoadMarkerForwardLeft = 70003,
  kRoadMarkerForwardRight = 70004,
  kRoadMarkerLeft = 70005,
  kRoadMarkerRight = 70006,
  kRoadMarkerAround = 70007,
  kRoadMarkerBan = 70008,
  kRoadMarkerCrosswalk = 70009,
  kRoadMarkerSpeed40 = 70010,
  kRoadMarkerSpeed50 = 70011,
  kRoadMarkerLeftRight = 70012,
  kRoadMarkerForwardLeftRight = 70013,
  kRoadMarkerForwardLeftAround = 70014,
  kRoadMarkerForwardAround = 70015,
  kRoadMarkerLeftAround = 70016,
  kRoadMarkerLeftCurve = 70017,
  kRoadMarkerRightCurve = 70018,
  kRoadMarkerSlowdown = 70019,
  kRoadMarkerStop = 70020,
  kRoadMarkerSpeedNumber = 70021,
  kRoadMarkerEnd = 70022,
  kLampTypeStart = 80001,
  kLampFrontLeft = 80002,
  kLampFrontRight = 80003,
  kLampBackLeft = 80004,
  kLampBackRight = 80005,
  kLampBackUp = 80006,
  kLampTurnLeft = 80007,
  kLampTurnRight = 80008,
  kLampCls = 80009,
  kLampClsUnknown = 80010,
  kLampClsTurnLeft = 80011,
  kLampClsTurnRight = 80012,
  kLampClsBrake = 80013,
  kLampClsNormalMoving = 80014,
  kLampClsShark = 80015,
  kLampTypeEnd = 80016,
  kStraightPole = 90001,
  kOtherPole = 90002,
  kTrafficBoard = 90003,
};

int64_t ToSapObjectLabel(SapObjectLabel label) {
  return static_cast<int64_t>(label);
}

int64_t ConvertMsgLabelToSapObjectLabel(int16_t label) {
  switch (label) {
    case 0:
      return ToSapObjectLabel(SapObjectLabel::kUnknown);
    case 1:
      return ToSapObjectLabel(SapObjectLabel::kPedestrian);
    case 2:
      return ToSapObjectLabel(SapObjectLabel::kVehicle);
    case 8:
      return ToSapObjectLabel(SapObjectLabel::kVehicleBus);
    case 14:
      return ToSapObjectLabel(SapObjectLabel::kBike);
    case 20:
      return ToSapObjectLabel(SapObjectLabel::kTrafficLightAll);
    case 65:
      return ToSapObjectLabel(SapObjectLabel::kTrafficSignAll);
    case 136:
      return ToSapObjectLabel(SapObjectLabel::kObstacleAll);
    case 140:
      return ToSapObjectLabel(SapObjectLabel::kRoadCone);
    case 143:
      return ToSapObjectLabel(SapObjectLabel::kPole);
    default:
      return ToSapObjectLabel(SapObjectLabel::kUnknown);
  }
}

int64_t ConvertMsgSubtypeToSapObjectLabel(int16_t sub_type) {
  switch (sub_type) {
    case -1:
      return ToSapObjectLabel(SapObjectLabel::kDefault);
    case 0:
      return ToSapObjectLabel(SapObjectLabel::kUnknown);
    case 1:
      return ToSapObjectLabel(SapObjectLabel::kPedestrian);
    case 2:
      return ToSapObjectLabel(SapObjectLabel::kVehicle);
    case 3:
      return ToSapObjectLabel(SapObjectLabel::kVehicleCar);
    case 4:
      return ToSapObjectLabel(SapObjectLabel::kVehicleSuv);
    case 6:
      return ToSapObjectLabel(SapObjectLabel::kVehicleTruck);
    case 8:
      return ToSapObjectLabel(SapObjectLabel::kVehicleBus);
    case 9:
      return ToSapObjectLabel(SapObjectLabel::kVehicleTricycle);
    case 12:
      return ToSapObjectLabel(SapObjectLabel::kVehicleOthers);
    case 13:
      return ToSapObjectLabel(SapObjectLabel::kVehicleEnd);
    case 14:
      return ToSapObjectLabel(SapObjectLabel::kBike);
    case 16:
      return ToSapObjectLabel(SapObjectLabel::kBikeBicycle);
    case 19:
      return ToSapObjectLabel(SapObjectLabel::kBikeEnd);
    case 20:
      return ToSapObjectLabel(SapObjectLabel::kTrafficLightAll);
    case 21:
      return ToSapObjectLabel(SapObjectLabel::kTrafficLightOther);
    case 22:
      return ToSapObjectLabel(SapObjectLabel::kTrafficLightRed);
    case 23:
      return ToSapObjectLabel(SapObjectLabel::kTrafficLightYellow);
    case 24:
      return ToSapObjectLabel(SapObjectLabel::kTrafficLightGreen);
    case 65:
      return ToSapObjectLabel(SapObjectLabel::kTrafficSignAll);
    case 66:
      return ToSapObjectLabel(SapObjectLabel::kTrafficSignXParking);
    case 136:
      return ToSapObjectLabel(SapObjectLabel::kObstacleAll);
    case 137:
      return ToSapObjectLabel(SapObjectLabel::kGate);
    case 138:
      return ToSapObjectLabel(SapObjectLabel::kTrain);
    case 139:
      return ToSapObjectLabel(SapObjectLabel::kWarningPillar);
    case 140:
      return ToSapObjectLabel(SapObjectLabel::kRoadCone);
    case 141:
      return ToSapObjectLabel(SapObjectLabel::kWarningTriangle);
    case 142:
      return ToSapObjectLabel(SapObjectLabel::kWarningFlare);
    case 143:
      return ToSapObjectLabel(SapObjectLabel::kPole);
    case 144:
      return ToSapObjectLabel(SapObjectLabel::kObstacleArrowLeft);
    case 145:
      return ToSapObjectLabel(SapObjectLabel::kObstacleArrowRight);
    case 146:
      return ToSapObjectLabel(SapObjectLabel::kCushionDrumC);
    case 147:
      return ToSapObjectLabel(SapObjectLabel::kCushionDrumR);
    case 148:
      return ToSapObjectLabel(SapObjectLabel::kGuardrail);
    case 149:
      return ToSapObjectLabel(SapObjectLabel::kRainwayGate);
    case 150:
      return ToSapObjectLabel(SapObjectLabel::kObstacleEnd);
    case 151:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerAll);
    case 152:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerForward);
    case 153:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerForwardLeft);
    case 154:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerForwardRight);
    case 155:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerLeft);
    case 156:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerRight);
    case 157:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerAround);
    case 158:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerBan);
    case 159:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerCrosswalk);
    case 160:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerSpeed40);
    case 161:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerSpeed50);
    case 162:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerEnd);
    case 163:
      return ToSapObjectLabel(SapObjectLabel::kPlaceHolder);
    case 164:
      return ToSapObjectLabel(SapObjectLabel::kObjectAll);
    case 176:
      return ToSapObjectLabel(SapObjectLabel::kBikeMotor);
    case 177:
      return ToSapObjectLabel(SapObjectLabel::kCyclist);
    case 178:
      return ToSapObjectLabel(SapObjectLabel::kAnimalAll);
    case 179:
      return ToSapObjectLabel(SapObjectLabel::kAnimalCat);
    case 180:
      return ToSapObjectLabel(SapObjectLabel::kAnimalDog);
    case 181:
      return ToSapObjectLabel(SapObjectLabel::kAnimalCow);
    case 182:
      return ToSapObjectLabel(SapObjectLabel::kAnimalEnd);
    case 183:
      return ToSapObjectLabel(SapObjectLabel::kTrafficLightBlack);
    case 184:
      return ToSapObjectLabel(SapObjectLabel::kTrafficLightGreenNumber);
    case 285:
      return ToSapObjectLabel(SapObjectLabel::kStraightPole);
    case 286:
      return ToSapObjectLabel(SapObjectLabel::kOtherPole);
    case 287:
      return ToSapObjectLabel(SapObjectLabel::kTrafficBoard);
    case 288:
      return ToSapObjectLabel(SapObjectLabel::kConstructionAhead);
    case 289:
      return ToSapObjectLabel(SapObjectLabel::kConstructionSignClosures);
    case 290:
      return ToSapObjectLabel(SapObjectLabel::kConstructionSignRightLaneClosures);
    case 291:
      return ToSapObjectLabel(SapObjectLabel::kConstructionSignLeftLaneClosures);
    case 292:
      return ToSapObjectLabel(SapObjectLabel::kConstructionSignTurnRight);
    case 293:
      return ToSapObjectLabel(SapObjectLabel::kConstructionSignTurnLeft);
    case 294:
      return ToSapObjectLabel(SapObjectLabel::kObstacleIsolationBarrel);
    case 295:
      return ToSapObjectLabel(SapObjectLabel::kObstacleBarrier);
    case 296:
      return ToSapObjectLabel(SapObjectLabel::kObstacleCone);
    case 297:
      return ToSapObjectLabel(SapObjectLabel::kObstaclePole);
    case 298:
      return ToSapObjectLabel(SapObjectLabel::kObstacleWarningTriangle);
    case 299:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerLeftRight);
    case 300:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerForwardLeftRight);
    case 301:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerForwardLeftAround);
    case 302:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerForwardAround);
    case 303:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerLeftAround);
    case 304:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerLeftCurve);
    case 305:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerRightCurve);
    case 306:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerSlowdown);
    case 307:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerStop);
    case 308:
      return ToSapObjectLabel(SapObjectLabel::kRoadMarkerSpeedNumber);
    case 309:
      return ToSapObjectLabel(SapObjectLabel::kLampTypeStart);
    case 310:
      return ToSapObjectLabel(SapObjectLabel::kLampFrontLeft);
    case 311:
      return ToSapObjectLabel(SapObjectLabel::kLampFrontRight);
    case 312:
      return ToSapObjectLabel(SapObjectLabel::kLampBackLeft);
    case 313:
      return ToSapObjectLabel(SapObjectLabel::kLampBackRight);
    case 314:
      return ToSapObjectLabel(SapObjectLabel::kLampBackUp);
    case 315:
      return ToSapObjectLabel(SapObjectLabel::kLampTurnLeft);
    case 316:
      return ToSapObjectLabel(SapObjectLabel::kLampTurnRight);
    case 317:
      return ToSapObjectLabel(SapObjectLabel::kLampCls);
    case 318:
      return ToSapObjectLabel(SapObjectLabel::kLampClsUnknown);
    case 319:
      return ToSapObjectLabel(SapObjectLabel::kLampClsTurnLeft);
    case 320:
      return ToSapObjectLabel(SapObjectLabel::kLampClsTurnRight);
    case 321:
      return ToSapObjectLabel(SapObjectLabel::kLampClsBrake);
    case 322:
      return ToSapObjectLabel(SapObjectLabel::kLampClsNormalMoving);
    case 323:
      return ToSapObjectLabel(SapObjectLabel::kLampClsShark);
    case 324:
      return ToSapObjectLabel(SapObjectLabel::kLampTypeEnd);
    case 325:
      return ToSapObjectLabel(SapObjectLabel::kCart);
    case 326:
      return ToSapObjectLabel(SapObjectLabel::kConstructionSign);
    case 327:
      return ToSapObjectLabel(SapObjectLabel::kObstacleOpenGate);
    case 328:
      return ToSapObjectLabel(SapObjectLabel::kObstacleCloseGate);
    case 329:
      return ToSapObjectLabel(SapObjectLabel::kAnimalBig);
    case 330:
      return ToSapObjectLabel(SapObjectLabel::kAnimalSmall);
    case 331:
      return ToSapObjectLabel(SapObjectLabel::kObstaclePillar);
    case 332:
      return ToSapObjectLabel(SapObjectLabel::kVehicleUnpoweredCar);
    default:
      return ToSapObjectLabel(SapObjectLabel::kUnknown);
  }
}

template <typename Vector3Reader>
void CopyVector3(const Vector3Reader& source, float target[3]) {
  target[0] = source.getX();
  target[1] = source.getY();
  target[2] = source.getZ();
}

template <typename Matrix3Reader>
void CopyMatrix3(const Matrix3Reader& source, float target[3][3]) {
  CopyVector3(source.getX(), target[0]);
  CopyVector3(source.getY(), target[1]);
  CopyVector3(source.getZ(), target[2]);
}

}  // namespace

bool ConvertBaseObject(const Object::Reader& msg, sapBaseObject* object) {
  if (object == nullptr) {
    return false;
  }

  // ObjectFrame 和 FusionObjectFrame 共享基础目标字段，集中在这里映射，
  // 防止两条输入链路对 label、motion、polygon 等字段出现不一致。
  object->sensor_id = msg.getSensorId();
  object->label = ConvertMsgLabelToSapObjectLabel(msg.getLabel());
  object->existence_confidence = msg.getExistenceConfidence();
  object->sub_type = ConvertMsgSubtypeToSapObjectLabel(msg.getSubtype());
  object->track_id = msg.getTrackId();
  object->current_prediction_timestamp_ns =
      msg.getCurrentPredictionTimestampNs();

  const auto size_info = msg.getSizeInfo();
  object->size_info.is_valid = size_info.getIsValid();
  object->size_info.size[0] = size_info.getSize().getX();
  object->size_info.size[1] = size_info.getSize().getY();
  object->size_info.size[2] = size_info.getSize().getZ();
  CopyMatrix3(size_info.getSizeUncertainty(),
              object->size_info.size_uncertainty);

  const auto polygon_box = msg.getPolygonBox();
  object->polygon_box.is_valid = polygon_box.getIsValid();
  object->polygon_box.height_lower_surface =
      polygon_box.getHeightLowerSurface();
  object->polygon_box.height_upper_surface = 0.0f;
  const auto polygon_contour = polygon_box.getPolygonContour();
  for (std::size_t i = 0; i < 8 && i < polygon_contour.size(); ++i) {
    object->polygon_box.polygon_contour[i].x = polygon_contour[i].getX();
    object->polygon_box.polygon_contour[i].y = polygon_contour[i].getY();
  }

  const auto direction_info = msg.getDirectionInfo();
  object->direction_info.is_valid = direction_info.getIsValid();
  const auto direction_ypr = direction_info.getDirectionYpr();
  object->direction_info.direction_ypr[0] = direction_ypr.getX();
  object->direction_info.direction_ypr[1] = direction_ypr.getY();
  object->direction_info.direction_ypr[2] = direction_ypr.getZ();
  CopyMatrix3(direction_info.getDirectionYprUncertainty(),
              object->direction_info.direction_ypr_uncertainty);
  object->direction_info.yaw = direction_info.getYaw();
  object->direction_info.yaw_uncertainty = direction_info.getYawUncertainty();
  object->direction_info.yaw_rate = direction_info.getYawRate();
  object->direction_info.yaw_rate_uncertainty =
      direction_info.getYawRateUncertainty();

  const auto motion_info = msg.getMotionInfo();
  object->motion_info.is_valid = motion_info.getIsValid();
  CopyVector3(motion_info.getCenter(), object->motion_info.center);
  CopyMatrix3(motion_info.getCenterUncertainty(),
              object->motion_info.center_uncertainty);
  CopyVector3(motion_info.getVelocity(), object->motion_info.velocity);
  CopyMatrix3(motion_info.getVelocityUncertainty(),
              object->motion_info.velocity_uncertainty);
  CopyVector3(motion_info.getAcceleration(), object->motion_info.acceleration);
  CopyMatrix3(motion_info.getAccelerationUncertainty(),
              object->motion_info.acceleration_uncertainty);
  CopyVector3(motion_info.getJerk(), object->motion_info.jerk);
  CopyMatrix3(motion_info.getJerkUncertainty(),
              object->motion_info.jerk_uncertainty);
  object->motion_info.velocity_heading = motion_info.getVelocityHeading();
  object->motion_info.velocity_heading_uncertainty =
      motion_info.getVelocityHeadingUncertainty();
  object->motion_info.velocity_heading_rate =
      motion_info.getVelocityHeadingRate();
  object->motion_info.velocity_heading_rate_uncertainty =
      motion_info.getVelocityHeadingRateUncertainty();

  object->motion_status_info.is_valid = 1;
  object->motion_status_info.motion_status =
      static_cast<sapMotionStatus>(msg.getMotionStatus());
  object->motion_status_info.unknown_score = msg.getMotionUnknownScore();
  object->motion_status_info.static_score = msg.getMotionStaticScore();
  object->motion_status_info.stopped_score = msg.getMotionStoppedScore();
  object->motion_status_info.moving_score = msg.getMotionMovingScore();
  object->motion_status_info.crossing_score = msg.getMotionCrossingScore();

  const auto bv_corners = msg.getBvCorners();
  object->bv_corners.is_valid = bv_corners.getIsValid();
  const auto corner_points = bv_corners.getBvCorners();
  const auto corner_scores = bv_corners.getCornerConf();
  const uint64_t corner_num = corner_points.size();
  object->bv_corners.bv_corner_num = corner_num;
  if (corner_num == 0) {
    return true;
  }

  auto* buffer =
      new uint8_t[(sizeof(sapPoint3f) + sizeof(float)) * corner_num];
  object->bv_corners.bv_corners = reinterpret_cast<sapPoint3f*>(buffer);
  object->bv_corners.corner_conf =
      reinterpret_cast<float*>(buffer + sizeof(sapPoint3f) * corner_num);
  for (uint64_t i = 0; i < corner_num; ++i) {
    object->bv_corners.bv_corners[i].x = corner_points[i].getX();
    object->bv_corners.bv_corners[i].y = corner_points[i].getY();
    object->bv_corners.bv_corners[i].z = corner_points[i].getZ();
    object->bv_corners.corner_conf[i] =
        (i < corner_scores.size()) ? corner_scores[i] : 0.0f;
  }
  return true;
}


}  // namespace detail
}  // namespace senseauto
}  // namespace usharing_dlp_node
