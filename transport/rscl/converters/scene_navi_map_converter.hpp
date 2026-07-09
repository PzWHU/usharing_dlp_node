// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_sdmap_convertor.cpp
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_local_map_convertor.cpp
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_navi_topo_convertor.cpp
// Purpose:
//   Convert SenseAuto SceneNaviMap messages to sap map/navigation DTOs.
#pragma once

#include <cstdint>

#include <ad_msg_idl/l2plus_maplesslm/maplesslm_msgs.capnp.h>

#include "sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace senseauto {

// Source mapping:
//   LocalSDMapConvertor::Convert()
//   LocalMapConvertor::Convert()
//   NaviTopoConvertor::Convert()
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_sdmap_convertor.cpp
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_local_map_convertor.cpp
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_navi_topo_convertor.cpp
// 单个 SceneNaviMap 同时拆出 SDMap、RoadStructure、CrossInfo、NaviTopo。
// 任一输出指针为空都视为调用错误。
bool ConvertSenseAutoSceneNaviMapToSap(
    ::senseAD::msg::maplesslm::SceneNaviMap::Reader msg,
    SapSDMapPtr* sdmap_output, SapRoadStructurePtr* road_structure_output,
    SapCrossInfoPtr* cross_info_output, SapNaviTopoPtr* navi_topo_output,
    uint64_t* timestamp_ns);

}  // namespace senseauto
}  // namespace usharing_dlp_node
