// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_occ_convertor.cpp
// Purpose:
//   Preserve OccupancyInfo -> sapPerceptionOccFrame mapping without
//   RSCL/SWCFL convertor registration dependencies.

#include "transport/rscl/converters/occupancy_info_converter.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>

namespace usharing_dlp_node {
namespace senseauto {

using OccupancyData = ::senseAD::msg::perception::OccupancyData;
using OccupancyInfo = ::senseAD::msg::perception::OccupancyInfo;
using PartitionInfo = ::senseAD::msg::perception::PartitionInfo;

namespace {

std::size_t PartitionByteSize(uint32_t num_partitions) {
  if (num_partitions == 0) {
    return 0;
  }
  return sizeof(float) * (num_partitions + 1) +
         sizeof(float) * num_partitions +
         sizeof(uint32_t) * num_partitions;
}

bool ConvertOccPartition(PartitionInfo::Reader msg,
                         sapPartitionInfo* partition, uint8_t** ptr) {
  const uint32_t num_partitions = msg.getNumPartition();
  partition->num_partions = num_partitions;
  if (num_partitions == 0) {
    return true;
  }

  const auto ranges = msg.getRanges();
  const auto steps = msg.getSteps();
  const auto voxels = msg.getNumVoxelsPerRange();
  if (ranges.size() < num_partitions + 1 || steps.size() < num_partitions ||
      voxels.size() < num_partitions) {
    return false;
  }

  // x/y/z partition 的 ranges/steps/num_voxels 使用同一块连续 buffer，
  // sap_raii 释放时只需要从第一个非空指针 delete[] 一次。
  partition->ranges = reinterpret_cast<float*>(*ptr);
  *ptr += sizeof(float) * (num_partitions + 1);
  for (uint32_t i = 0; i < num_partitions + 1; ++i) {
    partition->ranges[i] = ranges[i];
  }

  partition->steps = reinterpret_cast<float*>(*ptr);
  *ptr += sizeof(float) * num_partitions;
  for (uint32_t i = 0; i < num_partitions; ++i) {
    partition->steps[i] = steps[i];
  }

  partition->num_voxels_per_range = reinterpret_cast<uint32_t*>(*ptr);
  *ptr += sizeof(uint32_t) * num_partitions;
  for (uint32_t i = 0; i < num_partitions; ++i) {
    partition->num_voxels_per_range[i] = voxels[i];
  }
  return true;
}

bool ConvertOccInfo(const OccupancyData::Reader& msg,
                    sapOccupancyInfo* occ_info) {
  if (occ_info == nullptr) {
    return false;
  }

  const auto x_info = msg.getXPartitionInfo();
  const auto y_info = msg.getYPartitionInfo();
  const auto z_info = msg.getZPartitionInfo();
  const uint32_t x_num = x_info.getNumPartition();
  const uint32_t y_num = y_info.getNumPartition();
  const uint32_t z_num = z_info.getNumPartition();
  const std::size_t partition_bytes =
      PartitionByteSize(x_num) + PartitionByteSize(y_num) +
      PartitionByteSize(z_num);

  auto* partition_buffer =
      partition_bytes > 0 ? new uint8_t[partition_bytes]() : nullptr;
  auto* ptr = partition_buffer;
  // 任一分区转换失败都释放已经分配的连续 buffer，避免半初始化结构上抛。
  if (!ConvertOccPartition(x_info, &occ_info->x_partition_info, &ptr) ||
      !ConvertOccPartition(y_info, &occ_info->y_partition_info, &ptr) ||
      !ConvertOccPartition(z_info, &occ_info->z_partition_info, &ptr)) {
    delete[] partition_buffer;
    return false;
  }

  const uint32_t length = msg.getLength();
  const uint32_t width = msg.getWidth();
  const uint64_t expected_grids_num = static_cast<uint64_t>(length) * width;
  const auto idx_low = msg.getIdxLowList();
  const auto idx_high = msg.getIdxHighList();
  const auto occ_type = msg.getOccTypeList();
  const uint64_t grids_num = idx_low.size();
  if (grids_num != expected_grids_num || grids_num != idx_high.size() ||
      grids_num != occ_type.size()) {
    return false;
  }

  occ_info->grids_num = grids_num;
  // occ_grids 单独分配；partition buffer 和 occ_grids 的释放分别在
  // ReleaseConvertedOccFrame 中处理。
  occ_info->occ_grids =
      grids_num > 0 ? new sapOccGrid[grids_num]() : nullptr;
  for (uint64_t i = 0; i < grids_num; ++i) {
    occ_info->occ_grids[i].idx_low = idx_low[i];
    occ_info->occ_grids[i].idx_high = idx_high[i];
    occ_info->occ_grids[i].label = occ_type[i];
    occ_info->occ_grids[i].confidence = 0.0f;
    occ_info->occ_grids[i].x = 0.0f;
    occ_info->occ_grids[i].y = 0.0f;
    occ_info->occ_grids[i].min_height = 0.0f;
    occ_info->occ_grids[i].max_height = 0.0f;
    occ_info->occ_grids[i].grid_size_x = 0.0f;
    occ_info->occ_grids[i].grid_size_y = 0.0f;
  }
  return true;
}

bool ConvertOccFrame(const OccupancyInfo::Reader& msg,
                     sapPerceptionOccFrame* frame) {
  if (frame == nullptr) {
    return false;
  }

  frame->sensor_id = static_cast<uint16_t>(msg.getSensorId());
  frame->timestamp_ns = msg.getFrameTimestampNs();
  frame->sensor_send_timestamp_ns = 0;
  frame->use_vd_occ = false;
  return ConvertOccInfo(msg.getOccupancyData(),
                        &frame->occ_struct.occupancy_info_results);
}

}  // namespace

bool ConvertSenseAutoOccupancyInfoToSap(OccupancyInfo::Reader msg,
                                        SapOccFramePtr* output) {
  if (output == nullptr) {
    return false;
  }

  auto frame = MakeOccFrame();
  // MakeOccFrame 返回带自定义 deleter 的 shared_ptr，后续失败返回时会
  // 自动释放已经填充的内部数组。
  if (!ConvertOccFrame(msg, frame.get())) {
    return false;
  }
  *output = frame;
  return true;
}

}  // namespace senseauto
}  // namespace usharing_dlp_node
