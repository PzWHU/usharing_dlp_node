/*
 * Copyright (C) 2024 by SenseTime Group Limited. All rights reserved.
 * Wang Zhaowei <wangzhaowei@senseauto.com>
 */
#pragma once

#include <cstdint>

#include "perception_camera/sap_camera_define.h"
#include "perception_camera/data_type/base_type.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 单个栅格信息
 *
 */
typedef struct sapOccGrid {
    /* grid索引[idx_low, idx_high），通过ranges和step转换后为实际最小高度
     * 最小高度grid索引，闭区间
     */
    uint8_t idx_low;

    /** 最大高度grid索引，开区间
     */
    uint8_t idx_high;

    /** 网格占据类型 
     * Unknown = 0, Ground, Others
     */
     /** Parking 网格占据类型 (同大卓类型)
     *  UNKNOWN = 0;    //预留接口
        UNKNOWN_MOVABLE = 1;
        UNKNOWN_UNMOVABLE = 2;    //目前统一给到此type
        PEDESTRIAN = 3;  // Pedestrian, usually determined by moving behavior.
        BICYCLE = 4;     // bike, motor bike
        VEHICLE = 5;     // Passenger car or truck.
        FREESPACE = 6;  //预留接口
     */
    uint8_t label;

    /**
     * 置信度
     * 取值范围 [0.0, 1.0]
     */
    float confidence;
    /**
     *  x 坐标 (m)
     */
    float x;
    /**
     *  y 坐标 (m)
     */
    float y;
    /** 最小高度 (m)
     */
    float min_height;
    /** 最大高度 (m)
     */
    float max_height;
    /**
     * grid X 轴方向上的大小（m）
     */
    float grid_size_x;
    /**
     * grid Y 轴方向上的大小（m）
     */
    float grid_size_y;
} sapOccGrid;

/**@brief 当前维度的分区信息结构体*/
typedef struct {
    /** 分区数量 */
    uint32_t num_partions;
    /** 每个分区的voxels数量 */
    uint32_t *num_voxels_per_range;
    /** 连续分区的边界点*/
    float *ranges;
    /** 分区的步长/分辨率，代表每格多少米*/
    float *steps;
} sapPartitionInfo;

/**
 * @brief 感知2.5D OCC输出信息，障碍物占据高度
 *
 */
typedef struct {
    // x/y/z维度上的分区信息
    sapPartitionInfo x_partition_info;
    sapPartitionInfo y_partition_info;
    sapPartitionInfo z_partition_info;
    /** 占据grid */
    sapOccGrid *occ_grids;

    /** 占据grid数目
     */
    uint64_t grids_num;

} sapOccupancyInfo;

typedef struct {
    enum OccLabel {UNKNOWN = 0, GROUND = 1, OTHER = 2, CONST_AERA = 3};
    float range_3d[6];
    uint8_t *occ_info;
    uint64_t occ_info_num;
    float voxel_size[3];
    float grid_size[3];
} sapVDOccInfo;

typedef sapVDOccInfo sapVDOccInfoResult;
typedef sapOccupancyInfo sapOccupancyInfoResult;

/**
 * @brief 初始化占据信息结构体
 * @param[in] parking_slot 占据信息结构体指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API int32_t
InitsapOccupancyInfoResult(sapOccupancyInfoResult *occupancy_info);
SAP_API int32_t
InitsapVDOccInfoResult(sapVDOccInfoResult *occupancy_info);

#ifdef __cplusplus
}
#endif