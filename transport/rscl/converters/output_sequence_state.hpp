// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/send_convertors/prediction_to_msg_convertor.cpp
//   adapter/perception_camera/src/convertors/perception_camera/send_convertors/dlp_diag_convertor.cpp
// Purpose:
//   Caller-owned SenseAuto output sequence counters shared by output converters.
#pragma once

#include <cstdint>

namespace usharing_dlp_node {
namespace senseauto {

// 输出 converter 的序列号状态。由 transport 持有，保证 converter 函数
// 本身无静态状态，便于单元测试和后续多 transport 实例复用。
struct SenseAutoOutputSequenceState {
  uint32_t prediction_seq{0};
  uint32_t diag_seq{0};
};

}  // namespace senseauto
}  // namespace usharing_dlp_node
