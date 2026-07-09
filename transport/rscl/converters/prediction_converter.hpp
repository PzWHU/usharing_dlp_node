// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/send_convertors/prediction_to_msg_convertor.cpp
// Purpose:
//   Convert sap planning prediction output to SenseAuto Prediction messages.
#pragma once

#include <cstdint>

#include <ad_msg_idl/ad_prediction/prediction.capnp.h>

#include "transport/rscl/converters/output_sequence_state.hpp"
#include "sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace senseauto {

// Source mapping:
//   PredictionConvertor::ConvertPredictionObject()
//   adapter/perception_camera/src/convertors/perception_camera/send_convertors/prediction_to_msg_convertor.cpp
// 将 SDK 单个 prediction object 写入 SenseAuto PredictionObject builder。
void ConvertPredictionObjectToSenseAuto(
    const sapPredictionObject& object,
    ::senseAD::msg::prediction::PredictionObject::Builder builder);

// Source mapping:
//   PredictionConvertor::ConvertPlanningFrame()
//   adapter/perception_camera/src/convertors/perception_camera/send_convertors/prediction_to_msg_convertor.cpp
// 将 sapPerceptionPlanningFrame 写入 /prediction/objects_vd 对应消息。
void ConvertPlanningFrameToSenseAutoPrediction(
    const sapPerceptionPlanningFrame& frame,
    ::senseAD::msg::prediction::Prediction::Builder builder, uint64_t now_ns,
    SenseAutoOutputSequenceState* sequence_state,
    const char* frame_id = "nv_cameras");

}  // namespace senseauto
}  // namespace usharing_dlp_node
