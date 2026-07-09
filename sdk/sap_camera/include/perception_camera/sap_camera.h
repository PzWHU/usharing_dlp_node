/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Liu GuangXiong <liuguangxiong@senseauto.com>
 */
#pragma once

#include "perception_camera/sap_camera_define.h"
#include "perception_camera/sap_lidar_define.h"
#include "perception_camera/sap_radar_define.h"
#include "perception_camera/data_type/frame/perception_object_frame.h"
#include "perception_camera/data_type/frame/perception_road_geometry_frame.h"
#include "perception_camera/data_type/frame/perception_blob_frame.h"
#include "perception_camera/data_type/frame/perception_scene_frame.h"
#include "perception_camera/data_type/frame/perception_failsafe_frame.h"
#include "perception_camera/data_type/frame/perception_road_semantics.h"
#include "perception_camera/data_type/frame/perception_planning_frame.h"
#include "perception_camera/data_type/frame/perception_gop_frame.h"
#include "perception_camera/data_type/frame/perception_occ_frame.h"
#include "perception_camera/data_type/frame/perception_lidar_frame.h"
#include "perception_camera/data_type/frame/perception_fsline_frame.h"
#include "perception_camera/data_type/planning/location_info.h"
#include "perception_camera/data_type/planning/cross_info.h"
#include "perception_camera/data_type/planning/manual_signal.h"
#include "perception_camera/data_type/planning/local_map.h"
#include "perception_camera/data_type/planning/route_guidance.h"
#include "perception_camera/data_type/planning/route_lane_info.h"
#include "perception_camera/data_type/planning/route_light.h"
#include "perception_camera/data_type/planning/planning_info.h"
#include "perception_camera/data_type/planning/chassis_state.h"
#include "perception_camera/data_type/localization_info.h"
#include "perception_camera/data_type/road/fusion_slot.h"
#include "perception_camera/data_type/planning/navigation_info.h"
#include "perception_camera/data_type/frame/perception_fusion_object_frame.h"
#include "perception_camera/data_type/planning/pred_map.h"
#include "perception_camera/data_type/planning/navi_topo.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 设置回调函数，输出感知结果、和需要释放的原始图形数据
 * @param[in] callback_fun_set 回调函数结构指针
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapSetCallbackFunSet(const sapCallBackFunSet* callback_fun_set);

/**
 * @brief 创建相机感知解析处理句柄
 * @param[in] param 相机和pipeline配置参数
 * @return 成功返回句柄，失败返回NULL
 * @note
 * 本方法只创建内部的一些对象资源，并不进行pipeline的创建，模型的加载等操作，这些在sapInit方法中进行
 */
SAP_API
sapCameraHandle sapCreate(const sapCameraParam* param);

/**
 * @brief 相机处理Pipeline初始化
 * @param[in] handle 相机处理句柄
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapInit(sapCameraHandle handle);

/**
 * @brief 启动pipeline的运行
 * @param[in] handle 相机处理句柄
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapStart(sapCameraHandle handle);

/**
 * @brief 停止pipeline的运行
 * @param[in] handle 相机处理句柄
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapStop(sapCameraHandle handle);

/**
 * @brief 销毁pipeline相关资源
 * @param[in] handle 相机处理句柄
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapDeinit(sapCameraHandle handle);

/**
 * @brief 销毁相机处理句柄，清理资源
 * @param[in] pHandle 相机处理句柄
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapDestroy(sapCameraHandle* pHandle);

/**
 * @brief 输入图像帧进行感知目标解析
 *
 * @param[in] handle 相机处理句柄
 * @param[in] frame 图像帧数据
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapPushFrame(sapCameraHandle handle,
                     const sapCameraFrame* const frame,
                     sapModuleMatchStatus* mudule_status);

/**
 * @brief 输入lidar帧进行感知目标解析
 *
 * @param[in] handle 相机处理句柄
 * @param[in] frame lidar帧数据
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapPushLidarFrame(sapCameraHandle handle,
                          const sapLidarFrame* const frame,
                          sapModuleMatchStatus* mudule_status);

/**
 * @brief 输入radar帧进行感知目标解析
 *
 * @param[in] handle 相机处理句柄
 * @param[in] frame radar帧数据
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapPushRadarFrame(sapCameraHandle handle,
                          const sapRadarFrame* const frame,
                          sapModuleMatchStatus* mudule_status);

/**
 * @brief 输入局部定位数据
 *
 * @param[in] handle 相机处理句柄
 * @param[in] timestamp_ns 时间戳
 * @param[in] sap_local_localization 局部定位数据
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapPushLocalizationInfo(
    sapCameraHandle handle,
    const uint64_t timestamp_ns,
    const sapLocalLocalizationInfo* const sap_local_localization);

SAP_API
int32_t sapPushFusSlotPos(
    sapCameraHandle handle,
    const uint64_t timestamp_ns,
    const sapLocalLocalizationInfo* const sap_local_localization);

/**
 * @brief 输入融合车位数据
 *
 * @param[in] handle 相机处理句柄
 * @param[in] sap_fusion_slot 融合车位数据
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapPushFusionSlotInfo(sapCameraHandle handle,
                              const sapFusionSlotInfo* const sap_fusion_slot);

/**
 * @brief 输入SDMap数据
 *
 * @param[in] handle 相机处理句柄
 * @param[in] timestamp_ns 时间戳
 * @param[in] sap_sdmap SDMap原始数据
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapPushSDMap(sapCameraHandle handle,
                     const uint64_t timestamp_ns,
                     sapSDMap* const sap_sdmap);
SAP_API
int32_t sapPushCrossInfo(sapCameraHandle handle,
                         const uint64_t timestamp_ns,
                         sapCrossInfo* const sap_cross_info);
SAP_API
int32_t sapPushLocationInfo(sapCameraHandle handle,
                            const uint64_t timestamp_ns,
                            sapLocationInfo* const sap_location_info);
SAP_API
int32_t sapPushManualSignal(sapCameraHandle handle,
                            const uint64_t timestamp_ns,
                            sapManualSignal* const sap_manual_signal);
SAP_API
int32_t sapPushRouteGuidance(sapCameraHandle handle,
                             const uint64_t timestamp_ns,
                             sapRouteGuidance* const sap_route_guidance);
SAP_API
int32_t sapPushRouteLaneInfo(sapCameraHandle handle,
                             const uint64_t timestamp_ns,
                             sapRouteLaneInfo* const sap_route_lane_info);
SAP_API
int32_t sapPushRouteLight(sapCameraHandle handle,
                          const uint64_t timestamp_ns,
                          sapRouteLight* const sap_route_light);
SAP_API
int32_t sapPushRoadStructure(sapCameraHandle handle,
                             const uint64_t timestamp_ns,
                             sapRoadStructure* const sap_road_structure);
SAP_API
int32_t sapPushNaviTopo(sapCameraHandle handle,
                            const uint64_t timestamp_ns,
                            sapNaviTopo* const sap_navi_topo);
SAP_API
int32_t sapPushChassisState(sapCameraHandle handle,
                            const uint64_t timestamp_ns,
                            sapChassisState* const sap_chassis_state);
SAP_API
int32_t sapPushAmapNaviInfo(sapCameraHandle handle,
                            const uint64_t timestamp_ns,
                            sapAmapNaviInfo* const amap_nav_info);
SAP_API
int32_t sapPushObjectFrame(sapCameraHandle handle,
                           const uint64_t timestamp_ns,
                           sapPerceptionObjectFrame* const sap_objects);
SAP_API
int32_t sapPushRoadFrame(sapCameraHandle handle,
                         const uint64_t timestamp_ns,
                         sapPerceptionRoadFrame* const sap_roads_state);
SAP_API
int32_t sapPushPredMap(sapCameraHandle handle,
                       const uint64_t timestamp_ns,
                       sapPredMap* const pred_map);

SAP_API
int32_t sapPushFusionObjectFrame(
    sapCameraHandle handle,
    const uint64_t timestamp_ns,
    sapPerceptionFusionObjectFrame* const sap_fusion_objects);

SAP_API
int32_t sapPushOccInfoFrame(sapCameraHandle handle,
                         const uint64_t timestamp_ns,
                         sapPerceptionOccFrame* const sap_occ_info);

/**
 * @brief 获取Road User感知结果
 *
 * @param[in] handle 相机处理句柄
 * @param[in] module_name
 * 按功能划分的pipeline名称，即创建handle时指定的module_name，如road_user_fcos3d
 * @param[in] camera_name
 * 相机名称，如center_camera_fov120
 * @param[in,out] frame
 * 返回的目标信息，类型sapPerceptionObjectFrame。外部使用后，需要调用ReleaseObjectFrameMemory释放内部内存
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapGetObjectFrame(sapCameraHandle handle,
                          const sapPublishResultParam* const pub_param,
                          const char* const camera_name,
                          sapPerceptionObjectFrame* const frame);
/**
 * @brief 获取静态障碍物感知结果
 *
 * @param[in] handle 相机处理句柄
 * @param[in] module_name
 * 按功能划分的pipeline名称，即创建handle时指定的module_name，如gop_fov30
 * @param[in] camera_name
 * 相机名称，如center_camera_fov120
 * @param[in,out] frame frame
 * 返回的目标信息，外部使用后，需要调用ReleaseGopFrameMemory释放内部内存
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapGetGopFrame(sapCameraHandle handle,
                       const sapPublishResultParam* const pub_param,
                       const char* const camera_name,
                       sapPerceptionGopFrame* const frame);

/**
 * @brief 获取Road Geometry结果信息
 *
 * @param[in] handle 相机处理句柄
 * @param[in] module_name
 * 按功能划分的pipeline名称，即创建handle时指定的module_name
 * @param[in] camera_name
 * 相机名称，如center_camera_fov120
 * @param[in,out] frame 返回的Road
 * Geometry信息，类型sapPerceptionRoadFrame。外部使用后，
 * 需要调用ReleaseRoadFrameMemory释放内部内存
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapGetRoadGeometryFrame(sapCameraHandle handle,
                                const sapPublishResultParam* const pub_param,
                                const char* const camera_name,
                                sapPerceptionRoadFrame* frame);

/**
 * @brief 获取融合车道线结果信息
 *
 * @param[in] handle 相机处理句柄
 * @param[in] pub_param
 * 发布参数，包含pipeline名称、帧号等信息
 * @param[in] camera_name
 * 相机名称，如center_camera_fov120
 * @param[in] trt_timestamp
 * 以20fps获取结果时的上一帧时间戳
 * @param[in,out] frame 返回的Road
 * Geometry信息，类型sapPerceptionRoadFrame。外部使用后，
 * 需要调用ReleaseRoadFrameMemory释放内部内存
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapGetFusionLaneFrame(sapCameraHandle handle,
                              const sapPublishResultParam* const pub_param,
                              const char* const camera_name,
                              uint64_t trt_timestamp,
                              sapPerceptionRoadFrame* frame);

/**
 * @brief 获取光斑结果
 *
 * @param[in] handle 相机处理句柄
 * @param[in] module_name
 * 按功能划分的pipeline名称，即创建handle时指定的module_name
 * @param[in] camera_name
 * 相机名称，如center_camera_fov120
 * @param[in,out] frame
 * 返回的光斑信息。外部使用后，需要调用
 * ReleaseBlobFrameMemory释放内部内存
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapGetBlobFrame(sapCameraHandle handle,
                        const sapPublishResultParam* const pub_param,
                        const char* const camera_name,
                        sapPerceptionBlobFrame* const frame);
/**
 * @brief 获取相机遮挡结果
 *
 * @param[in] handle 相机处理句柄
 * @param[in] module_name
 * 按功能划分的pipeline名称，即创建handle时指定的module_name
 * @param[in] camera_name
 * 相机名称，如center_camera_fov120
 * @param[in,out] frame 返回的相机遮挡结果
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapGetFailsafeFrame(sapCameraHandle handle,
                            const sapPublishResultParam* const pub_param,
                            const char* const camera_name,
                            sapPerceptionFailsafeFrame* frame);

/**
 * @brief 获取场景感知信息
 *
 * @param[in] handle 相机处理句柄
 * @param[in] module_name
 * 按功能划分的pipeline名称，即创建handle时指定的module_name
 * @param[in] camera_name
 * 相机名称，如center_camera_fov120
 * @param[in] frame
 * 返回的场景感知结果。外部使用后，需要调用ReleaseSceneFrameMemory释放内部内存
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapGetSceneFrame(sapCameraHandle handle,
                         const sapPublishResultParam* const pub_param,
                         const char* const camera_name,
                         sapPerceptionSceneFrame* frame);

/**
 * @brief 获取路面语义(Road
 * Semantics)信息，如交通灯、交通路牌标注等
 *
 * @param[in] handle 相机处理句柄
 * @param[in] module_name
 * 按功能划分的pipeline名称，即创建handle时指定的module_name
 * @param[in] camera_name
 * 相机名称，如center_camera_fov120
 * @param[in] frame
 * 返回的路面语义结果。外部使用后，需要调用ReleaseRoadSemanticsFrame释放内部内存
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapGetRoadSemanticsFrame(sapCameraHandle handle,
                                 const sapPublishResultParam* const pub_param,
                                 const char* const camera_name,
                                 sapPerceptionRoadSemanticsFrame* frame);

/**
 * @brief 获取Lidar Gop结果信息
 *
 * @param[in] handle 相机处理句柄
 * @param[in] module_name
 * 按功能划分的pipeline名称，即创建handle时指定的module_name
 * @param[in] frame
 * 返回的路面结果。外部使用后，需要调用ReleaseLidarFrameMemory释放内部内存
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapGetLidarGopFrame(sapCameraHandle handle,
                            const sapPublishResultParam* const pub_param,
                            sapPerceptionLidarGopFrame* frame);

/**
 * @brief 获取占用信息
 *
 * @param[in] handle 相机处理句柄
 * @param[in] module_name
 * 按功能划分的pipeline名称，即创建handle时指定的module_name
 * @param[in] camera_name 相机名称，如nv_camera
 * @param[in,out] frame
 * 返回的占用信息，类型sapPerceptionOccFrame。外部使用后，
 * 需要调用ReleaseOccFrameMemory释放内部内存
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapGetOccFrame(sapCameraHandle handle,
                       const sapPublishResultParam* const pub_param,
                       const char* const camera_name,
                       sapPerceptionOccFrame* frame);

/**
 * @brief 获取可行驶区域信息
 *
 * @param[in] handle 相机处理句柄
 * @param[in] module_name
 * 按功能划分的pipeline名称，即创建handle时指定的module_name
 * @param[in] camera_name 相机名称，如nv_cameras
 * @param[in,out] frame
 * 返回的占用信息，类型sapPerceptionFSLineFrame。
 * 外部使用后，需要调用ReleaseFSLineFrameMemory释放内部内存
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapGetFSLineFrame(sapCameraHandle handle,
                          const sapPublishResultParam* const pub_param,
                          const char* const camera_name,
                          sapPerceptionFSLineFrame* frame);

/**
 * @brief 获取Parking Road Geometry结果信息
 *
 * @param[in] handle 相机处理句柄
 * @param[in] module_name
 * 按功能划分的pipeline名称，即创建handle时指定的module_name
 * @param[in] camera_name
 * 相机名称，如center_camera_fov120
 * @param[in,out] frame 返回的Road
 * Geometry信息，类型sapPerceptionRoadFrame。外部使用后，
 * 需要调用ReleaseRoadFrameMemory释放内部内存
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapGetParkingRoadGeometryFrame(
    sapCameraHandle handle,
    const sapPublishResultParam* const pub_param,
    const char* const camera_name,
    sapPerceptionRoadFrame* frame);

/**
 * @brief 获取Parking占用信息
 *
 * @param[in] handle 相机处理句柄
 * @param[in] module_name
 * 按功能划分的pipeline名称，即创建handle时指定的module_name
 * @param[in] camera_name 相机名称，如nv_camera
 * @param[in,out] frame
 * 返回的占用信息，类型sapPerceptionOccFrame。外部使用后，
 * 需要调用ReleaseOccFrameMemory释放内部内存
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapGetParkingOccFrame(sapCameraHandle handle,
                              const sapPublishResultParam* const pub_param,
                              const char* const camera_name,
                              sapPerceptionOccFrame* frame);

/**
 * @brief 获取规控信息
 *
 * @param[in] handle 相机处理句柄
 * @param[in] module_name
 * 按功能划分的pipeline名称，即创建handle时指定的module_name
 * @param[in] camera_name 相机名称，如nv_camera
 * @param[in,out] frame
 * 返回的规控信息，类型sapPerceptionPlanningFrame。外部使用后，
 * 需要调用ReleasePlanningFrameMemory释放内部内存
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapGetPlanningFrame(sapCameraHandle handle,
                            const sapPublishResultParam* const pub_param,
                            const char* const camera_name,
                            sapPerceptionPlanningFrame* frame);

/**
 * @brief 获取占用信息
 *
 * @param[in] handle 相机处理句柄
 * @param[in] info
 * 返回的调试信息，类型sapPerceptionPlanningDebugInfo。外部使用后，
 * 需要调用ReleasePerceptionPlanningDebugInfoMemory释放内部内存
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapGetPlanningDebugInfo(sapCameraHandle handle,
                                sapPerceptionPlanningDebugInfo* info);

/**
 * @brief 获取融合结果信息
 *
 * @param[in] handle 相机处理句柄
 * @param[in] module_name
 * 按功能划分的pipeline名称，即创建handle时指定的module_name
 * @param[in,out] frame
 * 返回的融合结果信息，类型sapPerceptionFusionObjectFrame。外部使用后，
 * 需要调用ReleasePerceptionFusionObjectFrame释放内部内存
 * @return 成功返回0，否则返回错误码
 */
SAP_API
int32_t sapGetFusionObjectFrame(sapCameraHandle handle,
                                const sapPublishResultParam* const pub_param,
                                sapPerceptionFusionObjectFrame* const frame);
// SAP_API
// int32_t sapGetCustomData(std::shared_ptr<void>
// &custom_data);

// SAP_API
// int32_t sapSetCustomData(std::shared_ptr<void>
// custom_data);

#ifdef __cplusplus
}
#endif
