// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_object_convertor.cpp
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_fusion_object_convertor.cpp
// Purpose:
//   Shared ObjectFrame/FusionObjectFrame base-object field mapping.
#pragma once

#include <ad_msg_idl/perception/perception.capnp.h>

#include "planning_sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace senseauto {
namespace detail {

using SenseAutoObject = ::senseAD::msg::perception::Object;

// Source mapping:
//   ObjectFrameMsgConvertor::ConvertBaseObject()
//   FusionObjectFrameMsgConvertor::ConvertFusionObject()
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_object_convertor.cpp
// ObjectFrame 和 FusionObjectFrame 共享的基础目标字段映射。
bool ConvertBaseObject(const SenseAutoObject::Reader& msg,
                       sapBaseObject* object);

}  // namespace detail
}  // namespace senseauto
}  // namespace usharing_dlp_node
