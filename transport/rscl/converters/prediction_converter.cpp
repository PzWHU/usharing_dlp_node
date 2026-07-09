// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/send_convertors/prediction_to_msg_convertor.cpp
// Purpose:
//   Preserve Pilot DLP prediction output field mapping without SWCFL Sender.

#include "transport/rscl/converters/prediction_converter.hpp"

#include <cstddef>

namespace usharing_dlp_node {
namespace senseauto {

namespace {

void SetPoint(const sapPoint2f& source,
              ::senseAD::msg::std_msgs::Point::Builder target) {
  target.setX(source.x);
  target.setY(source.y);
}

uint32_t NextSeq(uint32_t* seq) {
  if (seq == nullptr) {
    return 0;
  }
  if (*seq == 100000U) {
    *seq = 0U;
  }
  const uint32_t current = *seq;
  ++(*seq);
  return current;
}


}  // namespace

void ConvertPredictionObjectToSenseAuto(
    const sapPredictionObject& object,
    ::senseAD::msg::prediction::PredictionObject::Builder builder) {
  builder.setId(static_cast<int32_t>(object.id));

  builder.setObstacleTypeEnum(static_cast<uint32_t>(object.type));
  builder.setObjectLabelEnum(static_cast<uint32_t>(object.object_label));

  builder.setLengthM(object.length);
  builder.setWidthM(object.width);
  builder.setHeightM(object.height);

  auto polygon_contour = builder.initPolygonContour(
      static_cast<unsigned int>(object.polygon_contour_num));
  for (uint64_t i = 0; i < object.polygon_contour_num; ++i) {
    SetPoint(object.polygon_contour[i], polygon_contour[i]);
  }

  SetPoint(object.position, builder.initPosition());
  SetPoint(object.speed, builder.initSpeed());
  SetPoint(object.acceleration, builder.initAcceleration());
  SetPoint(object.heading_direction, builder.initHeadingDirection());

  builder.setInfoStateEnum(static_cast<uint32_t>(object.info_state));
  builder.setSceneTypeEnum(static_cast<uint32_t>(object.scene_type));
  // Source behavior: importance analysis was not available, so the old
  // PredictionConvertor published a constant importance level of 1.
  builder.setImportanceLevelEnum(1U);
  builder.setMoveStep(static_cast<uint32_t>(object.move_step));

  builder.setIntentionType(static_cast<uint32_t>(object.intention_type));
  builder.setTargetRoadId(object.target_road_id);
  builder.setTargetLaneId(object.target_lane_id);

  auto trajectories = builder.initTrajectoryArray(
      static_cast<unsigned int>(object.trajectory_array_num));
  // 一条 prediction object 可包含多条候选轨迹；旧输出保留每条轨迹的
  // polygon、速度、方差、碰撞标志和概率分数。
  for (uint64_t i = 0; i < object.trajectory_array_num; ++i) {
    const auto& trajectory = object.trajectory_array[i];
    auto msg_trajectory = trajectories[i];

    msg_trajectory.setPredictorName(trajectory.predictor_name);
    msg_trajectory.setTargetRoadId(trajectory.target_road_id);
    msg_trajectory.setTargetLaneId(trajectory.target_lane_id);

    msg_trajectory.setPointIntervalTimeNs(trajectory.point_interval_time);
    msg_trajectory.setNumPoints(trajectory.num_points);

    auto points = msg_trajectory.initTrajectoryPointArray(
        static_cast<unsigned int>(trajectory.trajectory_point_array_num));
    for (uint64_t j = 0; j < trajectory.trajectory_point_array_num; ++j) {
      const auto& point = trajectory.trajectory_point_array[j];
      auto msg_point = points[j];

      auto msg_polygon = msg_point.initPolygonContour(
          static_cast<unsigned int>(point.polygon_contour_num));
      for (uint64_t k = 0; k < point.polygon_contour_num; ++k) {
        SetPoint(point.polygon_contour[k], msg_polygon[k]);
      }

      SetPoint(point.position, msg_point.initPosition());
      SetPoint(point.speed, msg_point.initSpeed());
      SetPoint(point.direction, msg_point.initDirection());
      SetPoint(point.variance_position, msg_point.initVariancePosition());
      SetPoint(point.variance_speed, msg_point.initVarianceSpeed());

      msg_point.setCovariancePosition(point.covariance_position);
      msg_point.setCovarianceSpeed(point.covariance_speed);
    }

    msg_trajectory.setIsAbnormal(trajectory.is_abnormal);
    msg_trajectory.setIsCollision(trajectory.is_collision);
    msg_trajectory.setIsSelected(trajectory.is_selected);
    msg_trajectory.setProbability(trajectory.probability);
    msg_trajectory.setScore(trajectory.score);
    msg_trajectory.setIntentionType(
        static_cast<uint32_t>(trajectory.intention_type));
  }

  auto history_points = builder.initHistoryTrajectoryPoints(
      static_cast<unsigned int>(object.history_trajectory_points_num));
  for (uint64_t i = 0; i < object.history_trajectory_points_num; ++i) {
    SetPoint(object.history_trajectory_points[i], history_points[i]);
  }
}

void ConvertPlanningFrameToSenseAutoPrediction(
    const sapPerceptionPlanningFrame& frame,
    ::senseAD::msg::prediction::Prediction::Builder builder, uint64_t now_ns,
    SenseAutoOutputSequenceState* sequence_state, const char* frame_id) {
  const auto& objects = frame.traj_object_array;

  auto header = builder.getHeader();
  // 序列号由 transport 持有，避免 converter 内部出现静态状态。
  header.setSeq(NextSeq(sequence_state ? &sequence_state->prediction_seq
                                       : nullptr));
  header.getTime().setNanoSec(now_ns);
  header.setFrameId(frame_id == nullptr ? "" : frame_id);

  builder.setPredictionTimeNs(objects.prediction_time);
  builder.setFusionTimeNs(objects.fusion_time);

  // SDK 输出的 prediction_object_array 直接对应 /prediction/objects_vd
  // 中的 object_list。
  auto object_list = builder.initObjectList(
      static_cast<unsigned int>(objects.prediction_object_array_num));
  for (uint64_t i = 0; i < objects.prediction_object_array_num; ++i) {
    ConvertPredictionObjectToSenseAuto(objects.prediction_object_array[i],
                                       object_list[i]);
  }
}

}  // namespace senseauto
}  // namespace usharing_dlp_node
