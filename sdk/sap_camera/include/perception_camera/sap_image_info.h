/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 图像数据像素格式
 * 图像像素的内存布局格式
 */
typedef enum {
    /** 未知 */
    IMAGE_FORMAT_NONE = -1,
    /** BGR三通道，内存布局类似：BGRBGRBGR... */
    IMAGE_FORMAT_BGR_HWC = 0,
    /** BGR三通道，内存布局类似：BBB...GGG...RRR... */
    IMAGE_FORMAT_BGR_CHW,
    /** RGB三通道，内存布局类似：RGBRGBRGB... */
    IMAGE_FORMAT_RGB_HWC,
    /** RGB三通道，内存布局类似：RRR...GGG...BBB... */
    IMAGE_FORMAT_RGB_CHW,
    /** YUV420P */
    IMAGE_FORMAT_YUV420p,
    /** YUV420SP, NV12 */
    IMAGE_FORMAT_YUVNV12,
    /** YUV420SP, NV21 */
    IMAGE_FORMAT_YUVNV21,
    /** 灰度图 */
    IMAGE_FORMAT_GRAY8,
    /** 带A通道 */
    IMAGE_FORMAT_BGRA_CHW,
    IMAGE_FORMAT_RGBA_CHW,
    IMAGE_FORMAT_BGRA_HWC,
    IMAGE_FORMAT_RGBA_HWC,
    /** 枚举最大值 */
    IMAGE_FORMAT_MAX
} sapImageFormat;

/**
 * @brief 数据内存类型
 */
typedef enum {
    /** 未知内存类型 */
    SAP_MEM_UNKNOWN = -1,
    /** 主机内存 */
    SAP_MEM_HOST = 0,
    /** 设备内存 */
    SAP_MEM_DEVICE = 1,
    SYNC_SAP_MEM_NVSCI_1D = 2,
    SYNC_SAP_MEM_NVSCI_2D = 3
} sapMemType;

/**
 * @brief 外部crop区域
 */
typedef struct {
    /** crop起始点x坐标 */
    uint32_t start_x;
    /** crop起始点y坐标 */
    uint32_t start_y;
    /** crop区域宽度 */
    uint32_t width;
    /** crop区域高度 */
    uint32_t height;
} sapRoiRect;

typedef struct sapPymSliceImage_t {
    /** 图形数据指针 */
    uint8_t* data;
    /** 图像宽度 */
    uint32_t width;
    /** 图像高度 */
    uint32_t height;
    /** 图像宽度跨距 */
    uint32_t stride;
    /** 外部crop区域 */
    sapRoiRect roi_rect;
} sapPymSliceImage;

typedef struct sapImagePym_t {
    static const int8_t MAX_PYM_LEVEL = 6;
    sapPymSliceImage pym_slices[MAX_PYM_LEVEL];
    uint8_t pym_slice_num;
} sapImagePym;

/**
 * @brief 图像数据结构
 */
typedef struct {
    /** sapImagePym 数据指针 */
    uint8_t* data;
    /** 通道数 */
    uint8_t channel_num;
    /** 原始图形宽度，外部有crop/resize */
    uint32_t ori_width;
    /** 原始图形高度，外部有crop/resize */
    uint32_t ori_height;
    /** 图像像素格式 */
    sapImageFormat format;
    /** 图像数据内存类型 */
    sapMemType memory_type;
    uint64_t sensor_send_timestamp_ns;
    uint64_t perception_capture_timestamp_ns;
} sapRawImage;

#ifdef __cplusplus
}
#endif
