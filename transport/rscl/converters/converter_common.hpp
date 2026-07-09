// Ported from:
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_road_geometry_convertor.cpp
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_sdmap_convertor.cpp
//   adapter/perception_camera/src/convertors/perception_camera/recv_convertors/msg_to_amap_navi_info_convertor.cpp
// Purpose:
//   Shared allocation/copy helpers for per-message SenseAuto -> sap* converters.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "sap/sap_types.hpp"

namespace usharing_dlp_node {
namespace senseauto {
namespace detail {

// 分配一块清零后的连续字节内存。部分 sap* DTO 中多个数组字段会共享
// 这块内存的不同偏移，释放时必须只 delete[] 首地址一次。
inline uint8_t* AllocateBytes(std::size_t size) {
  return size > 0 ? new uint8_t[size]() : nullptr;
}

// 从连续 buffer 中切出 count 个 T，并推进游标。调用方负责保证 buffer
// 总大小足够。
template <typename T>
T* TakeBuffer(uint8_t** ptr, std::size_t count) {
  if (ptr == nullptr || *ptr == nullptr || count == 0) {
    return nullptr;
  }
  auto* value = reinterpret_cast<T*>(*ptr);
  *ptr += sizeof(T) * count;
  return value;
}

// 为 SDK DTO 中的数组字段分配清零数组；count 为 0 时保持 nullptr。
template <typename T>
T* NewArrayOrNull(uint64_t count) {
  return count > 0 ? new T[count]() : nullptr;
}

// Cap'n Proto Text -> SDK 定长 char 数组，统一清零和截断。
template <typename TextReader, std::size_t N>
void CopyCapnpText(const TextReader& source, char (&target)[N]) {
  std::memset(target, 0, N);
  std::strncpy(target, source.cStr(), N - 1);
}

// 常见 3D 点字段映射，避免不同 converter 对 x/y/z 做出不一致处理。
template <typename PointReader>
void CopyPoint3f(const PointReader& source, sapPoint3f* target) {
  target->x = static_cast<float>(source.getX());
  target->y = static_cast<float>(source.getY());
  target->z = static_cast<float>(source.getZ());
}

// SenseAuto timestamp(sec+nsec) 转纳秒。若消息本身只有 nanoSec 字段，
// 对应 converter 直接读取该字段。
template <typename TimestampReader>
uint64_t TimestampToNs(const TimestampReader& timestamp) {
  return static_cast<uint64_t>(timestamp.getSec()) * 1000000000ULL +
         timestamp.getNsec();
}

}  // namespace detail
}  // namespace senseauto
}  // namespace usharing_dlp_node
