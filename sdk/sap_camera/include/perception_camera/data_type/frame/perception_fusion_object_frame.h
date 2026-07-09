/*
 * Copyright (C) 2024 by SenseTime Group Limited. All rights reserved.
 * Wang Zhaowei <wangzhaowei@senseauto.com>
 */
#pragma once

#include <string.h>
#include <stdlib.h>

#include "perception_camera/sap_radar_define.h"
#include "perception_camera/data_type/object/fusion_object.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /** 图像时间戳 */
    uint64_t timestamp_ns;
    uint64_t sensor_send_timestamp_ns;
    uint64_t perception_capture_timestamp_ns;
    /** 帧号，自定义，无绝对意义，一般是顺序号 */
    int64_t frame_no;
    /** sensor编号 */
    int8_t sensor_id;
    /** sensor名称 */
    char sensor_name[MAX_NAME];
    /** radar 点云数量*/
    uint64_t fusion_objects_num;
    /** radar 数据 */
    sapFusionObject* fusion_objects_list;
} sapPerceptionFusionObjectFrame;

SAP_API
int32_t InitsapPerceptionFusionObjectFrame(sapPerceptionFusionObjectFrame* frame);

SAP_API void ReleaseFusionObjectFrameMemory(sapPerceptionFusionObjectFrame* frame);

#ifdef __cplusplus
}
#endif
