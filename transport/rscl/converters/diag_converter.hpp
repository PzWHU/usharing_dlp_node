// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/send_convertors/dlp_diag_convertor.cpp
// Purpose:
//   Convert DLP fault codes to SenseAuto DiagMsg output.
#pragma once

#include <cstdint>

#include <ad_msg_idl/ad_framework_sdk/diag_msg.capnp.h>

#include "transport/rscl/converters/output_sequence_state.hpp"
#include "core/diagnostics.hpp"

namespace usharing_dlp_node {
namespace senseauto {

// Source mapping:
//   DLPDiagConvertor::Convert()
//   adapter/perception_camera/src/convertors/perception_camera/send_convertors/dlp_diag_convertor.cpp
// 将 fault code 列表写入 SenseAuto DiagMsg；sequence_state 由 transport 持有。
void ConvertFaultCodesToSenseAutoDiag(
    const FaultCodes& fault_codes,
    ::senseAD::msg::ad_framework_sdk::DiagMsg::Builder builder,
    uint64_t now_ns, SenseAutoOutputSequenceState* sequence_state,
    const char* frame_id = "dlp_diag", const char* node_name = "dlp_diag");

}  // namespace senseauto
}  // namespace usharing_dlp_node
