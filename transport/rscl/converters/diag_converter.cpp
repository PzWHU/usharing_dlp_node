// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/send_convertors/dlp_diag_convertor.cpp
// Purpose:
//   Preserve DLP diagnostic message conversion while keeping sequence state in
//   the transport boundary.

#include "transport/rscl/converters/diag_converter.hpp"

#include <cstddef>
#include <string>

namespace usharing_dlp_node {
namespace senseauto {

namespace {

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

void ConvertFaultCodesToSenseAutoDiag(
    const FaultCodes& fault_codes,
    ::senseAD::msg::ad_framework_sdk::DiagMsg::Builder builder,
    uint64_t now_ns, SenseAutoOutputSequenceState* sequence_state,
    const char* frame_id, const char* node_name) {
  auto header = builder.getHeader();
  // fault code 输出单独维护 diag_seq，不与 prediction_seq 共用。
  header.setSeq(
      NextSeq(sequence_state ? &sequence_state->diag_seq : nullptr));
  header.setFrameId(frame_id == nullptr ? "" : frame_id);
  header.getTime().setNanoSec(now_ns);

  builder.setNodeName(node_name == nullptr ? "" : node_name);

  auto infos = builder.initInfos(static_cast<unsigned int>(fault_codes.size()));
  for (std::size_t i = 0; i < fault_codes.size(); ++i) {
    // Source behavior: the current DLPDiagConvertor only fills DiagInfo.msg
    // with the numeric fault code string. name/level/active/timestamp remain
    // unset until the source system defines a richer fault schema.
    infos[i].setMsg(std::to_string(fault_codes[i]));
  }
}


}  // namespace senseauto
}  // namespace usharing_dlp_node
