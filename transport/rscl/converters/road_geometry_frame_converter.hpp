// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_road_geometry_convertor.cpp
// Purpose:
//   Convert SenseAuto RoadGeometryFrame messages to sapPerceptionRoadFrame.
#pragma once

#include <ad_msg_idl/perception/perception.capnp.h>

#include "sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace senseauto {

// Source mapping:
//   RoadGeometryFrameMsgConvertor::ConvertRoadGeometryFrame()
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_road_geometry_convertor.cpp
// 输入：SenseAuto RoadGeometryFrame。
// 输出：sapPerceptionRoadFrame，供 sapPushRoadFrame 使用。
bool ConvertSenseAutoRoadGeometryFrameToSap(
    ::senseAD::msg::perception::RoadGeometryFrame::Reader msg,
    SapRoadFramePtr* output);

}  // namespace senseauto
}  // namespace usharing_dlp_node
