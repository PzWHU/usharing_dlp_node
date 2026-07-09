/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhang Shiquan <zhangshiquan@senseauto.com>
 */
#pragma once

namespace senseAD {
namespace perception {

enum class FrameType {
    UNKNOWN = -1,
    // perception related
    PERCEPTION_OBJECT_FRAME = 0,
    // aggregator related
    AGGREGATOR_INIT_CONFIG = 100,
    AGGREGATOR_PROCESS_TIMESTAMP = 101,
    AGGREGATOR_GET_OBJECT_TIMESTAMP = 102,
    // slam related
    LOCALIZATION_DATA = 200,
    VED_DATA = 201
};

typedef struct StructFrameHeader {
    StructFrameHeader() = default;
    virtual ~StructFrameHeader() = default;
    virtual FrameType GetFrameType() = 0;
    uint64_t write_timestamp_ns_{0};
} FrameHeader;

}  // namespace perception
}  // namespace senseAD
