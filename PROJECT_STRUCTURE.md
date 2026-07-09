# usharing_dlp_node 结构说明

本文说明 `/home/lenovo/project/usharing_dlp_node` 的目录职责、它和 0434
源工程的对应关系，以及本工程维护文件的逐文件作用。

## 1. 工程来源

0434 源工程：

```text
/home/lenovo/project/l29_sap_node-refs_tags_SenseAuto_thor_mp_v0.4.34
```

本目录是对 0434 `pilot_dlp` 链路的独立化移植。移植目标不是重写算法，而是保留
`sap_camera` SDK 和 `sap*` 数据结构，把原来的 RSCL/SWCFL DAG、workshop、
Proxy、DataConvertor 注册宏拆成普通 C++ 层次：

```text
RSCL message
  -> transport/rscl/subscribers
  -> transport/rscl/converters
  -> sap* DTO
  -> core/PlanningService
  -> sap/SapPlanningEngine
  -> sap_camera SDK
  -> sap planning output
  -> transport/rscl/converters
  -> RSCL publisher
```

0434 中实际启用的 `pilot_dlp` pipeline 来源：

```text
adapter/perception_camera/resource/x86/config/pipeline/pilot_dlp_pipeline.yaml
adapter/perception_camera/resource/thor/config/pipeline/pilot_dlp_pipeline.yaml
adapter/perception_camera/resource/x86/config/workshop/pilot_dlp_workshop.yaml
adapter/perception_camera/resource/thor/config/workshop/pilot_dlp_workshop.yaml
adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp
```

当前移植目录保留了 RSCL 作为通信边界，但 `core/` 和 `sap/` 不依赖 RSCL。

## 2. 0434 链路对应关系

### 2.1 输入输出 topic

默认配置在 `resource/config/dlp_node/{x86,thor}/topics_cfg`，对应 0434
pipeline 中实际打开的链路。

| 方向 | topic | SenseAuto 消息类型 | 移植后内部输入 |
| --- | --- | --- | --- |
| 输入 | `/perception/nv_cameras/object` | `senseAD::msg::perception::ObjectFrame` | `sapPerceptionObjectFrame` |
| 输入 | `/perception/nv_cameras/road_geometry` | `senseAD::msg::perception::RoadGeometryFrame` | `sapPerceptionRoadFrame` |
| 输入 | `/perception/nv_cameras/occ_info` | `senseAD::msg::perception::OccupancyInfo` | `sapPerceptionOccFrame` |
| 输入 | `/perception/lidar_gop/object` | `senseAD::msg::perception::ObjectFrame` | `sapPerceptionLidarGopFrame` |
| 输入 | `/localization/odomstate_info` | `senseAD::msg::localization::OdomStateInfo` | `sapLocalLocalizationInfo` |
| 输入 | `/canbus/vehicle_report` | `senseAD::msg::vehicle::VehicleReport` | `VehicleInfo -> sapManualSignal` |
| 输入 | `/mcu/vehicle_processing` | `senseAD::msg::planning::Vehicleprocessing` | `sapChassisState` |
| 输入 | `/maplesslm/scene_navi_map` | `senseAD::msg::maplesslm::SceneNaviMap` | `sapSDMap`、`sapRoadStructure`、`sapCrossInfo`、`sapNaviTopo` |
| 输入 | `/amap/nav_realtime/info` | `senseAD::msg::amap_navigation::AmapNaviInfo` | `sapAmapNaviInfo` |
| 输出 | `/prediction/objects_vd` | `senseAD::msg::prediction::Prediction` | SDK `sapPerceptionPlanningFrame` 转出 |
| 输出 | `/UniAD/PlanningDebug` | `senseAD::msg::perception::UniADPlanningDebug` | SDK `sapPerceptionPlanningDebugInfo` 转出 |

`/perception/fusion/object`、`/sensor/ins`、`/prediction/fault_codes_dlp` 在当前
0434 默认配置中置为空 topic，代码仍保留兼容实现，便于后续配置重新启用。

### 2.2 源文件到移植文件

| 0434 源代码/配置 | 本工程文件 | 关系 |
| --- | --- | --- |
| `adapter/perception_camera/resource/x86/dag/pilot_dlp_x86.dag`、`adapter/perception_camera/resource/thor/dag/pilot_dlp.dag`、`adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` | `app/main_rscl.cpp` | 独立 RSCL 进程入口，替代 0434 DAG/SWCFL 组件启动编排 |
| `adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml`、`adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml`、`adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp` 的 `PerceptionCameraSdkWorkshopOption` | `config/rscl_config.*` | 读取 `sys_cfg`、`planning_cfg`、`topics_cfg`，对应旧 workshop/pipeline 配置 |
| `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp` 的初始化/生命周期 | `sap/sap_planning_engine.*` | `sapCreate/init/start/stop` 和 `sapPush*`、`sapGet*` |
| `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp` 的 `OnDataArrived()` 分发 | `core/planning_service.*` | 把 transport 回调接到 engine |
| `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp` 的 `OnVehicleInfo()` | `core/vehicle_state_machine.*` | `VehicleInfo` 到 `sapManualSignal` 的跨帧状态逻辑 |
| `adapter/perception_camera/src/convertors/perception/object_convertor.cpp` | `transport/rscl/converters/object_frame_converter.*`、`object_common.*` | 相机目标输入转换 |
| `adapter/perception_camera/src/convertors/perception/fusion_object_convertor.cpp` | `transport/rscl/converters/fusion_object_frame_converter.*` | fusion 目标兼容转换，默认未启用 |
| `adapter/perception_camera/src/convertors/perception/road_geometry_convertor.cpp` | `transport/rscl/converters/road_geometry_frame_converter.*` | 道路几何输入转换 |
| `adapter/perception_camera/src/convertors/perception/occ_convertor.cpp` | `transport/rscl/converters/occupancy_info_converter.*` | OCC 输入转换 |
| `adapter/perception_camera/src/convertors/perception/lidar_gop_convertor.cpp` | `transport/rscl/converters/lidar_gop_frame_converter.*` | 0434 Lidar GOP 输入转换 |
| `adapter/perception_camera/src/convertors/perception/local_localization_convertor.cpp` | `transport/rscl/converters/local_localization_converter.*` | 局部定位输入转换 |
| `adapter/perception_camera/src/convertors/planning/global_localization_convertor.cpp` | `transport/rscl/converters/location_info_converter.*` | INS/global localization 兼容转换，默认未启用 |
| `adapter/perception_camera/src/convertors/planning/vehicle_report_convertor.cpp` | `transport/rscl/converters/vehicle_report_converter.*` | 车身 CAN 状态转 `VehicleInfo` |
| `adapter/perception_camera/src/convertors/planning/vehicle_processing_convertor.cpp` | `transport/rscl/converters/vehicle_processing_converter.*` | planning 车辆处理状态转 `sapChassisState` |
| `adapter/perception_camera/src/convertors/perception/road_struct_convertor.cpp` | `transport/rscl/converters/scene_navi_map_converter.*` | `SceneNaviMap` 中 RoadStructure 部分 |
| `adapter/perception_camera/src/convertors/planning/cross_info_convertor.cpp` | `transport/rscl/converters/scene_navi_map_converter.*` | `SceneNaviMap` 中 CrossInfo 部分 |
| `adapter/perception_camera/src/convertors/planning/navi_topo_convertor.cpp` | `transport/rscl/converters/scene_navi_map_converter.*` | `SceneNaviMap` 中 NaviTopo 部分 |
| `adapter/perception_camera/src/convertors/planning/sdmap_convertor.cpp` | `transport/rscl/converters/scene_navi_map_converter.*` | `SceneNaviMap` 中 SDMap 部分 |
| `adapter/perception_camera/src/convertors/planning/amap_navi_info_convertor.cpp` | `transport/rscl/converters/amap_navi_info_converter.*` | 高德导航输入转换 |
| `adapter/perception_camera/src/convertors/planning/prediction_convertor.cpp` | `transport/rscl/converters/prediction_converter.*` | planning prediction 输出转换 |
| `adapter/perception_camera/src/convertors/planning/planning_debug_convertor.cpp` | `transport/rscl/converters/planning_debug_converter.*` | planning debug 输出转换 |
| 0434 `resource/config/perception_camera/x86/vd_mapqr_dlp` | `resource/config/perception_camera/x86/vd_mapqr_dlp` | x86 DLP SDK pipeline 配置 |
| 0434 `resource/config/perception_camera/nvidia/vd_dlp` | `resource/config/perception_camera/nvidia/vd_dlp` | thor/nvidia DLP SDK pipeline 配置 |

## 3. 顶层目录职责

| 路径 | 作用 |
| --- | --- |
| `app/` | 可执行程序入口，只做进程级组装和生命周期控制。允许依赖具体通信框架。 |
| `common/` | 中立的时间和日志抽象，不依赖 ROS/RSCL/SWCFL。 |
| `config/` | RSCL 版配置模型和 JSON 文件加载器，把原 DAG/SWCFL workshop 配置转换为 `PlanningConfig` 与 `RsclPortConfig`。 |
| `core/` | planning 编排层。只处理 engine/transport 接口、车辆状态机、诊断状态和中立配置。 |
| `sap/` | `sap_camera` SDK 适配层和 `sap*` 内存所有权封装。只处理 SDK DTO、SDK 生命周期和 `sapPush*`、`sapGet*`。 |
| `sdk/` | 本地复制的 `sap_camera`、`sap_common` 头文件布局，以及预期放置 SDK 动态库的位置。不是本工程业务代码。 |
| `transport/` | 通信抽象和具体通信实现。`transport/rscl` 负责 SenseAuto RSCL 输入输出；`transport/ros2` 是后续 ROS2 边界占位。 |
| `resource/` | 运行配置、SDK pipeline prototxt、license、模型和车辆标定占位。 |
| `build*/` | CMake 生成目录，不属于源码。 |

## 4. 逐文件说明

本节的 `0434 对应部分` 均相对 0434 源工程根目录：

```text
/home/lenovo/project/l29_sap_node-refs_tags_SenseAuto_thor_mp_v0.4.34
```

`本工程新增` 表示该文件是移植时新增的边界、文档或占位，没有 0434 源树中的
一对一文件；这类文件的来源会写出它承接的 0434 配置、workshop 或 convertor。
路径中的 `{x86,thor}` 表示 x86 和 thor 两个平台目录，`*` 表示同目录下同类文件。

### 4.1 构建与入口

| 文件 | 作用 | 0434 对应部分 |
| --- | --- | --- |
| `CMakeLists.txt` | 定义 `core`、`sap`、SenseAuto converters、RSCL transport、RSCL app 等可选 target；配置 SDK、ADMsgIDL、RSCL、JsonCpp 依赖；提供 `USHARING_DLP_NODE_HAS_SAP_PUSH_LIDAR_GOP_FRAME` 开关。 | `CMakeLists.txt`、`adapter/perception_camera/CMakeLists.txt`、`.saphub/cmake/*` 的构建和安装逻辑。 |
| `build.sh` | 本地构建脚本。默认构建 RSCL 静态库和 converter；如果本地存在 `libsap_camera.so`、`libsap_common.so`，自动打开 RSCL 可执行程序构建。 | 本工程新增；承接 0434 `Makefile`、`setup_env.sh` 和 `.saphub/saphub` 的本地构建入口职责。 |
| `README.md` | 工程入口说明，指向本文档。 | 本工程新增；无 0434 源码对应。 |
| `PROJECT_STRUCTURE.md` | 本文档，说明目录职责、源代码关系和文件作用。 | 本工程新增；依据 0434 `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml`、`adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml`、`adapter/perception_camera/src/convertors/*` 和 `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp` 整理。 |
| `app/main_rscl.cpp` | RSCL 可执行程序入口。加载配置，初始化 `CRsclAccess`，组装 `PlanningService + SapPlanningEngine + RsclPlanningTransport`，处理进程退出信号。 | 本工程新增入口；对应 0434 `adapter/perception_camera/resource/{x86,thor}/dag/pilot_dlp*.dag`、`adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 和 `adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml` 中的组件启动编排。 |

### 4.2 common

| 文件 | 作用 | 0434 对应部分 |
| --- | --- | --- |
| `common/clock.hpp` | 中立时钟接口 `IClock` 和系统时钟实现。替代旧代码中散落的时间工具，避免 core/sap 直接依赖 RSCL 或 ROS 时间。 | `adapter/perception_camera/include/perception_camera_helper.hpp`、`adapter/perception_camera/include/utils/sys_time.hpp`、`adapter/perception_camera/src/utils/sys_time.cpp` 中的时间工具职责。 |
| `common/logger.hpp` | 中立日志接口 `ILogger` 和 stderr fallback 实现。`SapPlanningEngine` 通过它记录 SDK 生命周期和错误。 | `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp` 中的 `AD_SWCFL_*` 日志使用点；本工程改为中立接口。 |

### 4.3 config

| 文件 | 作用 | 0434 对应部分 |
| --- | --- | --- |
| `config/rscl_config.hpp` | 定义 `TopicConfig`、`RsclPortConfig`、`JsonConfigLoader`。这是 RSCL 版本配置边界。 | 0434 无同名 JSON 配置类；对应 `adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml` 的 topic/SDK 配置结构和 `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp` 的 `PerceptionCameraSdkWorkshopOption`。 |
| `config/rscl_config.cpp` | 读取 `resource/config/dlp_node/<platform>/{sys_cfg,planning_cfg,topics_cfg}`，解析 SDK 初始化参数和 RSCL topic，解析后把相对资源路径转换到模块根目录。 | 0434 `adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml`、`adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 和 `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp::PerceptionCameraSdkWorkshopOption::OnInit()` 的解析内容。 |

### 4.4 core

| 文件 | 作用 | 0434 对应部分 |
| --- | --- | --- |
| `core/config.hpp` | 定义中立 `PlanningConfig` 和 `PlanningPipelineConfig`，对应旧 `PerceptionCameraSdk` workshop 配置块。 | `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp` 的 `PerceptionCameraSdkWorkshopOption`、`PipelineInfo`，以及 `adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml` 的 `PerceptionCameraSdk` 配置块。 |
| `core/diagnostics.hpp` | 定义 `FaultCodes`、`VehicleInfo`、`PerceptionPlanningInfo` 等 core 共享状态结构。 | `adapter/perception_camera/include/perception_camera_helper.hpp`、`adapter/perception_camera/include/perception_camera_dtc.hpp`、`adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp` 中的车辆状态和故障状态字段。 |
| `core/planning_engine.hpp` | 定义 `IPlanningEngine`，输入统一是 `sap*` 或 core DTO，输出通过回调交给 service。 | `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp` 中 `PerceptionCameraSdkWorkshop` 对 `sap_camera` 的输入/输出边界。 |
| `core/planning_service.hpp` | 定义 `PlanningService`，负责组合 engine 与 transport。 | `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 的 `RunOnceSynchronizer -> PerceptionCameraSdk -> Prediction/DebugConvertor` 链路。 |
| `core/planning_service.cpp` | 绑定 transport 输入回调到 `IPlanningEngine::PushXxx()`，绑定 engine 输出回调到 transport 发布函数；替代旧 workshop 图中的数据分发。 | `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp` 的 `OnDataArrived()` 分发逻辑，以及 `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 的 successor 关系。 |
| `core/vehicle_state_machine.hpp` | 声明车辆状态机，把 `VehicleInfo`、`sapChassisState` 转换/缓存为 planning 所需的 `sapManualSignal` 和 debug 状态。 | `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp` 的 `OnVehicleInfo()` 和 `sapManualSignal` 生成相关成员状态。 |
| `core/vehicle_state_machine.cpp` | 实现旧 `OnVehicleInfo()` 中的自动驾驶状态、接管状态、灯光/拨杆状态、期望速度等跨帧逻辑。 | `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp::OnVehicleInfo()`。 |

### 4.5 sap

| 文件 | 作用 | 0434 对应部分 |
| --- | --- | --- |
| `sap/sap_types.hpp` | 集中定义 `SapObjectFramePtr`、`SapRoadFramePtr` 等 `sap*` shared_ptr 别名和工厂/释放函数声明。所有 transport/converter 都以这里的类型向 core 交付数据。 | `adapter/perception_camera/include/perception_camera_proxy_traits.hpp` 中的 proxy 类型约定，以及 `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp` 中实际消费的 `sap*` DTO。 |
| `sap/sap_raii.cpp` | 实现 `sap*` 输入和 SDK 输出的 RAII 释放规则，包括 ObjectFrame、LidarGOP、OCC、PlanningFrame、PlanningDebugInfo 的内存释放。 | `adapter/perception_camera/src/convertors/{perception,planning}/*_convertor.cpp` 的 `new/delete` 释放逻辑，以及 `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp` 中 SDK 输出释放职责。 |
| `sap/sap_planning_engine.hpp` | 声明 `SapPlanningEngine`，是 `IPlanningEngine` 的 `sap_camera` SDK 实现。 | `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp` 的 `PerceptionCameraSdkWorkshop` 类和 `sap_camera` API 边界。 |
| `sap/sap_planning_engine.cpp` | 管理 `sap_camera` 生命周期，填充 `sapCameraParam`，注册 SDK 回调，调用 `sapPush*` 输入 SDK，并通过 `sapGetPlanningFrame`、`sapGetPlanningDebugInfo` 拉取输出。 | `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp` 中 `OnInit()`、SDK callback 注册、`OnDataArrived()`、`sapPush*`、`sapGetPlanningFrame()` 和 `sapGetPlanningDebugInfo()` 调用。 |

注意：当前本地 `sap_camera.h` 没有声明 `sapPushLidarGopFrame`，代码默认保留
Lidar GOP 输入链路但不强行链接该符号。使用匹配 0434 的 SDK 动态库和头文件时，
可通过 `USHARING_DLP_NODE_HAS_SAP_PUSH_LIDAR_GOP_FRAME=ON` 打开真实推送。

### 4.6 transport 公共接口

| 文件 | 作用 | 0434 对应部分 |
| --- | --- | --- |
| `transport/transport.hpp` | 定义 `IPlanningTransport`。transport 通过 `on_object`、`on_road`、`on_lidar_gop` 等回调把转换后的内部 DTO 交给 `PlanningService`，并提供 prediction/debug/diag 发布接口。 | 本工程新增抽象；对应 `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 中 receiver、convertor、sender 的通信边界。 |

### 4.7 transport/rscl

| 文件 | 作用 | 0434 对应部分 |
| --- | --- | --- |
| `transport/rscl/rscl_access.hpp` | 声明 `CRsclAccess` 单例封装，集中暴露 RSCL init、subscribe、advertise、publish、message 创建。 | 本工程新增 RSCL 封装；对应 0434 `adapter/perception_camera/src/workshops/perception_rsclmsg_workshop_register.cpp` 中由 SWCFL 宏生成的 RSCL receiver/sender 能力。 |
| `transport/rscl/rscl_access.cpp` | 实现 RSCL runtime 管理。把 RSCL 生命周期限制在 transport/app 边界。 | 本工程新增 RSCL 封装；对应 0434 `adapter/perception_camera/src/workshops/perception_rsclmsg_workshop_register.cpp` 和 `adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml` 的 topic 绑定。 |
| `transport/rscl/rscl_planning_transport.hpp` | 声明 RSCL transport 实现，持有各输入 subscriber 和输出序列号状态。 | 本工程新增 transport；对应 `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 中 receiver/convertor/sender 的整体链路。 |
| `transport/rscl/rscl_planning_transport.cpp` | 按 `RsclPortConfig` 初始化订阅和发布；订阅回调转发给 `IPlanningTransport` 的 on_xxx；输出侧把 SDK sap 结果转换为 RSCL message 并发布。空 topic 会跳过订阅，用于表达 0434 pipeline 中关闭的链路。 | `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml`、`adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml` 和 `adapter/perception_camera/src/workshops/perception_rsclmsg_workshop_register.cpp` 中的 receiver/sender 组合。 |

### 4.8 transport/rscl/converters

| 文件 | 作用 | 0434 对应部分 |
| --- | --- | --- |
| `amap_navi_info_converter.hpp` | 声明 `AmapNaviInfo -> sapAmapNaviInfo` 转换接口。 | `adapter/perception_camera/src/convertors/planning/amap_navi_info_convertor.cpp`。 |
| `amap_navi_info_converter.cpp` | 实现高德导航信息、车道、相机、交通设施、红绿灯等动态数组字段转换和释放规则。 | `adapter/perception_camera/src/convertors/planning/amap_navi_info_convertor.cpp`。 |
| `converter_common.hpp` | converter 内共享的内存分配、Cap'n Proto text 拷贝、点坐标拷贝、时间戳辅助函数。 | 从多个 0434 convertor 抽出的公共逻辑，主要对应 `adapter/perception_camera/include/perception_camera_convert_common.hpp` 和 `adapter/perception_camera/src/convertors/{perception,planning}/*_convertor.cpp` 中重复的数组/字符串/点拷贝代码。 |
| `diag_converter.hpp` | 声明 fault code 到 `DiagMsg` 的输出转换接口。 | 0434 默认未启用 DLP diag sender；对应 `adapter/perception_camera/resource/{x86,thor}/config/diag/pilot_dlp_diag*.yaml` 和 `adapter/perception_camera/src/workshops/perception_rsclmsg_workshop_register.cpp` 中注释掉的 `DiagMsg` sender 定义。 |
| `diag_converter.cpp` | 实现诊断消息 header、序列号和故障码数组写入。当前 0434 默认配置未发布 diag。 | 0434 默认未启用 DLP diag sender；对应 `adapter/perception_camera/resource/{x86,thor}/config/diag/pilot_dlp_diag*.yaml` 和 `adapter/perception_camera/src/workshops/perception_rsclmsg_workshop_register.cpp` 中注释掉的 `DiagMsg` sender 定义。 |
| `fusion_object_frame_converter.hpp` | 声明 fusion `ObjectFrame -> sapPerceptionFusionObjectFrame` 转换接口。 | `adapter/perception_camera/src/convertors/perception/fusion_object_convertor.cpp`，在 `pilot_dlp_pipeline.yaml` 中默认注释关闭。 |
| `fusion_object_frame_converter.cpp` | 实现 fusion object base、kinematics、polygon、source sensor 等字段转换。当前 0434 默认配置关闭该输入。 | `adapter/perception_camera/src/convertors/perception/fusion_object_convertor.cpp`。 |
| `lidar_gop_frame_converter.hpp` | 声明 0434 `/perception/lidar_gop/object` 的 `ObjectFrame -> sapPerceptionLidarGopFrame` 转换接口。 | `adapter/perception_camera/src/convertors/perception/lidar_gop_convertor.cpp`。 |
| `lidar_gop_frame_converter.cpp` | 实现 Lidar GOP obstacle 的 id、type、confidence、size、center、yaw、2D box 等基础字段转换。GOP 扩展字段受当前 SDK 头文件限制未写入。 | `adapter/perception_camera/src/convertors/perception/lidar_gop_convertor.cpp`。 |
| `local_localization_converter.hpp` | 声明 `OdomStateInfo -> sapLocalLocalizationInfo` 转换接口。 | `adapter/perception_camera/src/convertors/perception/local_localization_convertor.cpp`。 |
| `local_localization_converter.cpp` | 实现局部定位位姿、速度、角速度、四元数转换；用标准数学替代旧 Eigen 表达式。 | `adapter/perception_camera/src/convertors/perception/local_localization_convertor.cpp`。 |
| `location_info_converter.hpp` | 声明 `Ins -> sapLocationInfo` 转换接口。 | `adapter/perception_camera/src/convertors/planning/global_localization_convertor.cpp`，在 `pilot_dlp_pipeline.yaml` 中默认注释关闭。 |
| `location_info_converter.cpp` | 实现 INS/global localization 的最小字段转换。当前 0434 默认配置关闭该输入。 | `adapter/perception_camera/src/convertors/planning/global_localization_convertor.cpp`。 |
| `object_common.hpp` | 声明 ObjectFrame 和 FusionObjectFrame 共用的 `sapBaseObject` 字段转换。 | 从 `adapter/perception_camera/src/convertors/perception/object_convertor.cpp` 和 `adapter/perception_camera/src/convertors/perception/fusion_object_convertor.cpp` 抽出的公共目标字段逻辑。 |
| `object_common.cpp` | 实现 label/subtype 数值映射、尺寸、polygon、方向、运动、历史信息等基础目标字段转换。 | `adapter/perception_camera/src/convertors/perception/object_convertor.cpp`、`adapter/perception_camera/src/convertors/perception/fusion_object_convertor.cpp`，并参考 `adapter/perception_camera/include/message_helper/message_object_label.hpp` 的 label 映射。 |
| `object_frame_converter.hpp` | 声明相机 `ObjectFrame -> sapPerceptionObjectFrame` 转换接口。 | `adapter/perception_camera/src/convertors/perception/object_convertor.cpp`。 |
| `object_frame_converter.cpp` | 实现 vehicle、non-vehicle、pedestrian 分类、动态内存布局、landmark、lane assignment、depth point、车灯和 2D bbox 字段转换。 | `adapter/perception_camera/src/convertors/perception/object_convertor.cpp`。 |
| `occupancy_info_converter.hpp` | 声明 `OccupancyInfo -> sapPerceptionOccFrame` 转换接口。 | `adapter/perception_camera/src/convertors/perception/occ_convertor.cpp`。 |
| `occupancy_info_converter.cpp` | 实现 OCC partition、grid、range 等字段转换，并与 `sap_raii` 中的释放规则配对。 | `adapter/perception_camera/src/convertors/perception/occ_convertor.cpp`。 |
| `output_sequence_state.hpp` | 定义 prediction/diag 输出序列号状态，由 transport 持有，避免 converter 内部静态状态。 | `adapter/perception_camera/src/convertors/planning/prediction_convertor.cpp` 中输出序列号状态；diag 部分为本工程补充。 |
| `planning_debug_converter.hpp` | 声明 `sapPerceptionPlanningDebugInfo -> UniADPlanningDebug` 转换接口。 | `adapter/perception_camera/src/convertors/planning/planning_debug_convertor.cpp`。 |
| `planning_debug_converter.cpp` | 组合 SDK debug 字符串和车辆状态机状态，写入 `textboxString`。 | `adapter/perception_camera/src/convertors/planning/planning_debug_convertor.cpp`。 |
| `prediction_converter.hpp` | 声明 `sapPerceptionPlanningFrame -> Prediction` 输出转换接口。 | `adapter/perception_camera/src/convertors/planning/prediction_convertor.cpp`。 |
| `prediction_converter.cpp` | 实现 prediction header、object、trajectory point、polygon contour 等输出字段转换。 | `adapter/perception_camera/src/convertors/planning/prediction_convertor.cpp`。 |
| `road_geometry_frame_converter.hpp` | 声明 `RoadGeometryFrame -> sapPerceptionRoadFrame` 转换接口。 | `adapter/perception_camera/src/convertors/perception/road_geometry_convertor.cpp`。 |
| `road_geometry_frame_converter.cpp` | 实现车道线、道路边界、道路边缘、holistic path 等道路几何字段转换。 | `adapter/perception_camera/src/convertors/perception/road_geometry_convertor.cpp`，并参考 `adapter/perception_camera/include/message_helper/message_road_geometry_label.hpp`。 |
| `scene_navi_map_converter.hpp` | 声明 `SceneNaviMap -> SDMap/RoadStructure/CrossInfo/NaviTopo` 一次转换多路 sap DTO 的接口。 | `adapter/perception_camera/src/convertors/perception/road_struct_convertor.cpp`、`adapter/perception_camera/src/convertors/planning/cross_info_convertor.cpp`、`adapter/perception_camera/src/convertors/planning/navi_topo_convertor.cpp`、`adapter/perception_camera/src/convertors/planning/sdmap_convertor.cpp`。 |
| `scene_navi_map_converter.cpp` | 实现 0434 `SceneNaviMapReceiver` 的四路 fan-out，替代旧 pipeline 中多个 convertor 重复消费同一 topic。 | `adapter/perception_camera/src/convertors/perception/road_struct_convertor.cpp`、`adapter/perception_camera/src/convertors/planning/cross_info_convertor.cpp`、`adapter/perception_camera/src/convertors/planning/navi_topo_convertor.cpp`、`adapter/perception_camera/src/convertors/planning/sdmap_convertor.cpp`，以及 `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 的 `SceneNaviMapReceiver` successor 配置。 |
| `vehicle_processing_converter.hpp` | 声明 `Vehicleprocessing -> sapChassisState` 转换接口。 | `adapter/perception_camera/src/convertors/planning/vehicle_processing_convertor.cpp`。 |
| `vehicle_processing_converter.cpp` | 实现 `sapChassisState` 中转向灯、set speed、no_impl_turn_signal 等当前旧链路实际写入字段。 | `adapter/perception_camera/src/convertors/planning/vehicle_processing_convertor.cpp`。 |
| `vehicle_report_converter.hpp` | 声明 `VehicleReport -> VehicleInfo` 转换接口和跨帧计数状态。 | `adapter/perception_camera/src/convertors/planning/vehicle_report_convertor.cpp`。 |
| `vehicle_report_converter.cpp` | 实现车速、加速度、档位、转角、接管判断、横向控制状态等转换。`sapManualSignal` 生成不在此处，而在 `VehicleStateMachine`。 | `adapter/perception_camera/src/convertors/planning/vehicle_report_convertor.cpp`；`sapManualSignal` 生成对应 `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp::OnVehicleInfo()`。 |

### 4.9 transport/rscl/subscribers

每个 subscriber 只做三件事：保存 topic 配置，注册 RSCL subscribe，收到消息后调用对应 converter 并触发内部回调。它们不直接调用 SDK。

| 文件 | 作用 | 0434 对应部分 |
| --- | --- | --- |
| `amap_navi_info/amap_navi_info_subscriber.hpp` | 声明高德导航输入订阅器。 | `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 的 `AmapNaviInfoReceiver -> AmapNaviInfoConvertor`，以及 `adapter/perception_camera/src/convertors/planning/amap_navi_info_convertor.cpp`。 |
| `amap_navi_info/amap_navi_info_subscriber.cpp` | 订阅 `/amap/nav_realtime/info`，输出 `sapAmapNaviInfo` 和 timestamp。 | `adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml` 的 `AmapNaviInfoReceiver` topic，`adapter/perception_camera/src/workshops/perception_rsclmsg_workshop_register.cpp` 的 AmapNaviInfo receiver 宏，以及 `adapter/perception_camera/src/convertors/planning/amap_navi_info_convertor.cpp`。 |
| `fusion_object_frame/fusion_object_frame_subscriber.hpp` | 声明 fusion object 兼容订阅器。 | `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 中注释关闭的 `FusionObjectFrameReceiver -> FusionObjectFrameConvertor`，以及 `adapter/perception_camera/src/convertors/perception/fusion_object_convertor.cpp`。 |
| `fusion_object_frame/fusion_object_frame_subscriber.cpp` | 订阅 ObjectFrame schema 的 fusion topic，输出 `sapPerceptionFusionObjectFrame`。默认配置未启用。 | `adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml` 的 `FusionObjectFrameReceiver` topic，`adapter/perception_camera/src/workshops/perception_rsclmsg_workshop_register.cpp` 的 ObjectFrame receiver 宏，以及 `adapter/perception_camera/src/convertors/perception/fusion_object_convertor.cpp`。 |
| `lidar_gop_frame/lidar_gop_frame_subscriber.hpp` | 声明 0434 Lidar GOP 订阅器。 | `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 的 `LidarGopFrameReceiver -> LidarGopFrameConvertor`，以及 `adapter/perception_camera/src/convertors/perception/lidar_gop_convertor.cpp`。 |
| `lidar_gop_frame/lidar_gop_frame_subscriber.cpp` | 订阅 `/perception/lidar_gop/object`，输出 `sapPerceptionLidarGopFrame` 和 timestamp。 | `adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml` 的 `LidarGopFrameReceiver` topic，`adapter/perception_camera/src/workshops/perception_rsclmsg_workshop_register.cpp` 的 ObjectFrame receiver 宏，以及 `adapter/perception_camera/src/convertors/perception/lidar_gop_convertor.cpp`。 |
| `local_localization/local_localization_subscriber.hpp` | 声明局部定位订阅器。 | `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 的 `LocalLocalizationReceiver -> LocalLocalizationConvertor`，以及 `adapter/perception_camera/src/convertors/perception/local_localization_convertor.cpp`。 |
| `local_localization/local_localization_subscriber.cpp` | 订阅 `/localization/odomstate_info`，输出 `sapLocalLocalizationInfo`。 | `adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml` 的 `LocalLocalizationReceiver` topic，`adapter/perception_camera/src/workshops/perception_rsclmsg_workshop_register.cpp` 的 OdomStateInfo receiver 宏，以及 `adapter/perception_camera/src/convertors/perception/local_localization_convertor.cpp`。 |
| `location_info/location_info_subscriber.hpp` | 声明 INS/global localization 兼容订阅器。 | `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 中注释关闭的 `InsReceiver -> InsConvertor`，以及 `adapter/perception_camera/src/convertors/planning/global_localization_convertor.cpp`。 |
| `location_info/location_info_subscriber.cpp` | 订阅 `/sensor/ins`，输出 `sapLocationInfo` 和 timestamp。默认配置未启用。 | `adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml` 的 `InsReceiver` topic，`adapter/perception_camera/src/workshops/perception_rsclmsg_workshop_register.cpp` 的 Ins receiver 宏，以及 `adapter/perception_camera/src/convertors/planning/global_localization_convertor.cpp`。 |
| `object_frame/object_frame_subscriber.hpp` | 声明相机目标订阅器。 | `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 的 `ObjectFrameReceiver -> ObjectFrameConvertor`，以及 `adapter/perception_camera/src/convertors/perception/object_convertor.cpp`。 |
| `object_frame/object_frame_subscriber.cpp` | 订阅 `/perception/nv_cameras/object`，输出 `sapPerceptionObjectFrame`。 | `adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml` 的 `ObjectFrameReceiver` topic，`adapter/perception_camera/src/workshops/perception_rsclmsg_workshop_register.cpp` 的 ObjectFrame receiver 宏，以及 `adapter/perception_camera/src/convertors/perception/object_convertor.cpp`。 |
| `occupancy_info/occupancy_info_subscriber.hpp` | 声明 OCC 订阅器。 | `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 的 `OccupancyInfoReceiver -> OccFrameConvertor`，以及 `adapter/perception_camera/src/convertors/perception/occ_convertor.cpp`。 |
| `occupancy_info/occupancy_info_subscriber.cpp` | 订阅 `/perception/nv_cameras/occ_info`，输出 `sapPerceptionOccFrame`。 | `adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml` 的 `OccupancyInfoReceiver` topic，`adapter/perception_camera/src/workshops/perception_rsclmsg_workshop_register.cpp` 的 OccupancyInfo receiver 宏，以及 `adapter/perception_camera/src/convertors/perception/occ_convertor.cpp`。 |
| `road_geometry_frame/road_geometry_frame_subscriber.hpp` | 声明道路几何订阅器。 | `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 的 `RoadGeometryFrameReceiver -> RoadGeometryFrameConvertor`，以及 `adapter/perception_camera/src/convertors/perception/road_geometry_convertor.cpp`。 |
| `road_geometry_frame/road_geometry_frame_subscriber.cpp` | 订阅 `/perception/nv_cameras/road_geometry`，输出 `sapPerceptionRoadFrame`。 | `adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml` 的 `RoadGeometryFrameReceiver` topic，`adapter/perception_camera/src/workshops/perception_rsclmsg_workshop_register.cpp` 的 RoadGeometryFrame receiver 宏，以及 `adapter/perception_camera/src/convertors/perception/road_geometry_convertor.cpp`。 |
| `scene_navi_map/scene_navi_map_subscriber.hpp` | 声明 SceneNaviMap 订阅器和四路转换结果结构。 | `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 的 `SceneNaviMapReceiver -> RoadStruct/CrossInfo/NaviTopo/SDMapConvertor` fan-out。 |
| `scene_navi_map/scene_navi_map_subscriber.cpp` | 订阅 `/maplesslm/scene_navi_map`，一次转换出 SDMap、RoadStructure、CrossInfo、NaviTopo。 | `adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml` 的 `SceneNaviMapReceiver` topic，`adapter/perception_camera/src/workshops/perception_rsclmsg_workshop_register.cpp` 的 SceneNaviMap receiver 宏，以及 `adapter/perception_camera/src/convertors/perception/road_struct_convertor.cpp`、`adapter/perception_camera/src/convertors/planning/cross_info_convertor.cpp`、`adapter/perception_camera/src/convertors/planning/navi_topo_convertor.cpp`、`adapter/perception_camera/src/convertors/planning/sdmap_convertor.cpp`。 |
| `vehicle_processing/vehicle_processing_subscriber.hpp` | 声明 Vehicleprocessing 订阅器。 | `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 的 `VehicleprocessingReceiver -> VehicleProcessingConvertor`，以及 `adapter/perception_camera/src/convertors/planning/vehicle_processing_convertor.cpp`。 |
| `vehicle_processing/vehicle_processing_subscriber.cpp` | 订阅 `/mcu/vehicle_processing`，输出 `sapChassisState` 和 timestamp。 | `adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml` 的 `VehicleprocessingReceiver` topic，`adapter/perception_camera/src/workshops/perception_rsclmsg_workshop_register.cpp` 的 Vehicleprocessing receiver 宏，以及 `adapter/perception_camera/src/convertors/planning/vehicle_processing_convertor.cpp`。 |
| `vehicle_report/vehicle_report_subscriber.hpp` | 声明 VehicleReport 订阅器，内部持有 takeover 计数状态。 | `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 的 `VehicleReportReceiver -> VehicleReportConvertor`，以及 `adapter/perception_camera/src/convertors/planning/vehicle_report_convertor.cpp`。 |
| `vehicle_report/vehicle_report_subscriber.cpp` | 订阅 `/canbus/vehicle_report`，输出中立 `VehicleInfo` 和 timestamp。 | `adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml` 的 `VehicleReportReceiver` topic，`adapter/perception_camera/src/workshops/perception_rsclmsg_workshop_register.cpp` 的 VehicleReport receiver 宏，以及 `adapter/perception_camera/src/convertors/planning/vehicle_report_convertor.cpp`。 |

### 4.10 transport/ros2

| 文件 | 作用 | 0434 对应部分 |
| --- | --- | --- |
| `transport/ros2/ros2_transport.hpp` | ROS2 transport 占位声明。当前 0434 迁移主要验证 RSCL 边界。 | 本工程新增占位；未来替换 0434 `adapter/perception_camera/resource/{x86,thor}/config/pipeline/pilot_dlp_pipeline.yaml` 和 `adapter/perception_camera/resource/{x86,thor}/config/workshop/pilot_dlp_workshop.yaml` 中的 RSCL receiver/sender 边界。 |
| `transport/ros2/ros2_transport.cpp` | ROS2 transport 占位实现，保留未来替换通信边界的位置。 | 本工程新增占位；未来承接 0434 pipeline/workshop 的通信边界，不对应 0434 源树中的单独 ROS2 文件。 |

### 4.11 resource

| 文件/目录 | 作用 | 0434 对应部分 |
| --- | --- | --- |
| `resource/README.md` | 运行资源布局说明。 | 本工程新增；无 0434 源码对应。 |
| `resource/config/dlp_node/x86/sys_cfg` | x86 默认进程名、车型和日志配置。 | 本工程新增 JSON；对应 0434 `adapter/perception_camera/resource/x86/dag/pilot_dlp_x86.dag` 的进程/组件入口和 `adapter/perception_camera/resource/x86/config/workshop/pilot_dlp_workshop.yaml` 的节点配置。 |
| `resource/config/dlp_node/x86/planning_cfg` | x86 SDK 初始化配置，指向 `config/perception_camera/x86/vd_mapqr_dlp/camera_bev_v9_7_ep12_dlp.prototxt`。 | 本工程新增 JSON；对应 `adapter/perception_camera/resource/x86/config/workshop/pilot_dlp_workshop.yaml` 的 `PerceptionCameraSdk` 配置块和 `resource/config/perception_camera/x86/vd_mapqr_dlp/camera_bev_v9_7_ep12_dlp.prototxt`。 |
| `resource/config/dlp_node/x86/topics_cfg` | x86 RSCL 输入输出 topic 配置。 | 本工程新增 JSON；对应 `adapter/perception_camera/resource/x86/config/workshop/pilot_dlp_workshop.yaml` 的 receiver/sender topic 和 `adapter/perception_camera/resource/x86/config/pipeline/pilot_dlp_pipeline.yaml` 的启用链路。 |
| `resource/config/dlp_node/thor/sys_cfg` | thor 默认进程名、车型和日志配置。 | 本工程新增 JSON；对应 0434 `adapter/perception_camera/resource/thor/dag/pilot_dlp.dag` 的进程/组件入口和 `adapter/perception_camera/resource/thor/config/workshop/pilot_dlp_workshop.yaml` 的节点配置。 |
| `resource/config/dlp_node/thor/planning_cfg` | thor SDK 初始化配置，指向 `config/perception_camera/nvidia/vd_dlp/camera_bev_v9_7_ep12_dlp.prototxt`。 | 本工程新增 JSON；对应 `adapter/perception_camera/resource/thor/config/workshop/pilot_dlp_workshop.yaml` 的 `PerceptionCameraSdk` 配置块和 `resource/config/perception_camera/nvidia/vd_dlp/camera_bev_v9_7_ep12_dlp.prototxt`。 |
| `resource/config/dlp_node/thor/topics_cfg` | thor RSCL 输入输出 topic 配置。 | 本工程新增 JSON；对应 `adapter/perception_camera/resource/thor/config/workshop/pilot_dlp_workshop.yaml` 的 receiver/sender topic 和 `adapter/perception_camera/resource/thor/config/pipeline/pilot_dlp_pipeline.yaml` 的启用链路。 |
| `resource/config/perception_camera/x86/vd_mapqr_dlp/*.prototxt` | 从 0434 x86 复制的 DLP SDK pipeline 子配置。 | `resource/config/perception_camera/x86/vd_mapqr_dlp/*.prototxt`。 |
| `resource/config/perception_camera/nvidia/vd_dlp/*.prototxt` | 从 0434 thor/nvidia 复制的 DLP SDK pipeline 子配置。 | `resource/config/perception_camera/nvidia/vd_dlp/*.prototxt`。 |
| `resource/config/x86/vd_mapqr_dlp/*.prototxt` | 早期迁移 scaffold 保留的 x86 DLP 配置副本，当前默认 `planning_cfg` 不再指向它。 | 早期副本；源自 0434 `resource/config/perception_camera/x86/vd_mapqr_dlp/*.prototxt`。 |
| `resource/licenses/perception_camera/SENSEAD.lic` | `sap_camera` SDK license。 | `resource/licenses/perception_camera/SENSEAD.lic`。 |
| `resource/licenses/perception_aggregator/GAC_SENSEADAS.lic` | perception aggregator license，随资源包保留。 | `resource/licenses/perception_aggregator/GAC_SENSEADAS.lic`。 |
| `resource/models/x86/vd_dlp/dlp_v1_009a9_rl_issue_aug_new/*` | DLP x86 模型和 embedding/normalization 配套文件。 | 当前 0434 源树未展开 `resource/models`；属于部署/模型资源包，`resource/config/perception_camera/{x86,nvidia}/vd_*/*.prototxt` 中引用。 |
| `resource/vehicle/default/README.md` | 车辆标定目录占位说明；真实车端需要放入目标车辆标定。 | 本工程新增占位；0434 车辆配置在 `resource/vehicle_node_config/*.yaml`，真实传感器标定由部署环境提供。 |

常用 prototxt 作用：

| 文件 | 作用 | 0434 对应部分 |
| --- | --- | --- |
| `camera_bev_v9_7_ep12_dlp.prototxt` | DLP planning 主 pipeline，`planning_cfg` 直接引用它。 | `resource/config/perception_camera/x86/vd_mapqr_dlp/camera_bev_v9_7_ep12_dlp.prototxt`、`resource/config/perception_camera/nvidia/vd_dlp/camera_bev_v9_7_ep12_dlp.prototxt`。 |
| `camera_bev_v9_7_ep12_backbone.prototxt` | BEV backbone 子 pipeline 配置。 | `resource/config/perception_camera/x86/vd_mapqr_dlp/camera_bev_v9_7_ep12_backbone.prototxt`、`resource/config/perception_camera/nvidia/vd_dlp/camera_bev_v9_7_ep12_backbone.prototxt`。 |
| `camera_bev_v9_7_ep12_pvb.prototxt` | PVB/object 相关子配置。 | `resource/config/perception_camera/x86/vd_mapqr_dlp/camera_bev_v9_7_ep12_pvb.prototxt`、`resource/config/perception_camera/nvidia/vd_dlp/camera_bev_v9_7_ep12_pvb.prototxt`。 |
| `camera_bev_v9_7_ep12_road.prototxt` | road 相关子配置。 | `resource/config/perception_camera/x86/vd_mapqr_dlp/camera_bev_v9_7_ep12_road.prototxt`、`resource/config/perception_camera/nvidia/vd_dlp/camera_bev_v9_7_ep12_road.prototxt`。 |
| `camera_bev_v9_7_ep12_road_mapqr_post.prototxt` | road mapqr 后处理配置。 | `resource/config/perception_camera/x86/vd_mapqr_dlp/camera_bev_v9_7_ep12_road_mapqr_post.prototxt`、`resource/config/perception_camera/nvidia/vd_dlp/camera_bev_v9_7_ep12_road_mapqr_post.prototxt`。 |
| `camera_bev_v9_7_ep12_road_mapseg_post.prototxt` | road mapseg 后处理配置。 | `resource/config/perception_camera/x86/vd_mapqr_dlp/camera_bev_v9_7_ep12_road_mapseg_post.prototxt`、`resource/config/perception_camera/nvidia/vd_dlp/camera_bev_v9_7_ep12_road_mapseg_post.prototxt`。 |
| `camera_bev_v9_7_ep12_occ_4in1.prototxt` | OCC 子配置。 | `resource/config/perception_camera/x86/vd_mapqr_dlp/camera_bev_v9_7_ep12_occ_4in1.prototxt`、`resource/config/perception_camera/nvidia/vd_dlp/camera_bev_v9_7_ep12_occ_4in1.prototxt`。 |
| `camera_bev_v9_7_ep12_gop.prototxt` | GOP 子配置。 | `resource/config/perception_camera/x86/vd_mapqr_dlp/camera_bev_v9_7_ep12_gop.prototxt`、`resource/config/perception_camera/nvidia/vd_dlp/camera_bev_v9_7_ep12_gop.prototxt`。 |
| `fusion_lane.prototxt` | lane fusion 子配置。 | `resource/config/perception_camera/x86/vd_mapqr_dlp/fusion_lane.prototxt`、`resource/config/perception_camera/nvidia/vd_dlp/fusion_lane.prototxt`。 |
| `lidar_bev_backbone.prototxt` | lidar BEV backbone 子配置。 | `resource/config/perception_camera/x86/vd_mapqr_dlp/lidar_bev_backbone.prototxt`、`resource/config/perception_camera/nvidia/vd_dlp/lidar_bev_backbone.prototxt`。 |
| `lidar_camera_bev_fusion.prototxt` | lidar-camera BEV fusion 子配置。 | `resource/config/perception_camera/x86/vd_mapqr_dlp/lidar_camera_bev_fusion.prototxt`、`resource/config/perception_camera/nvidia/vd_dlp/lidar_camera_bev_fusion.prototxt`。 |
| `camera_bev_v9_7_ep12_aggregator.prototxt` | nvidia 资源中的 aggregator 配置。 | `resource/config/perception_camera/nvidia/vd_dlp/camera_bev_v9_7_ep12_aggregator.prototxt`。 |
| `camera_bev_v9_7_ep12_dlp_only_pre.prototxt` | nvidia 资源中的仅前处理 DLP 配置。 | `resource/config/perception_camera/nvidia/vd_dlp/camera_bev_v9_7_ep12_dlp_only_pre.prototxt`；x86 0434 另有 `adapter/perception_camera/resource/x86/config/pipeline/pilot_dlp_only_pre_pipeline.yaml` 编排。 |
| `camera_bev_v9_7_ep12_dlp_produce_fillback.prototxt` | nvidia 资源中的 fillback 相关 DLP 配置。 | `resource/config/perception_camera/nvidia/vd_dlp/camera_bev_v9_7_ep12_dlp_produce_fillback.prototxt`；x86 0434 另有 `adapter/perception_camera/resource/x86/config/pipeline/pilot_dlp_produce_fillback_pipeline.yaml` 编排。 |

### 4.12 sdk

`sdk/` 是本工程本地化依赖目录，目的是让编译不再到兄弟工程或下载目录找
`sap_camera`、`sap_common` 头文件。它不是业务迁移代码。

当前 0434 源树没有展开 `module/perception_camera` 或 `/opt/senseauto/...` 下的 SDK
头文件；它们在 0434 中由 `adapter/perception_camera/CMakeLists.txt` 通过
`PERCEPTION_CAMERA_SDK_INCLUDE_DIRS`、`Perception_Common_INCLUDE_DIRS` 引入，并链接
`sap_camera`、`sap_common`。因此下表的 `0434 对应部分` 写的是 0434 对该外部 SDK 的
引用位置和主要使用点。

| 文件/目录 | 作用 | 0434 对应部分 |
| --- | --- | --- |
| `sdk/README.md` | SDK 目录布局和动态库放置说明。 | 本工程新增；说明 0434 `adapter/perception_camera/CMakeLists.txt` 外部依赖在本工程中的本地化布局。 |
| `sdk/sap_camera/include/perception_camera/sap_camera.h` | `sap_camera` C API 入口，声明 `sapCreate/init/start/stop`、`sapPush*`、`sapGet*` 等。 | 外部 SDK 头；0434 由 `adapter/perception_camera/CMakeLists.txt` 引用，主要使用点是 `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp`。 |
| `sdk/sap_camera/include/perception_camera/sap_camera_define.h` | `sap_camera` 基础宏、handle、callback、参数结构。 | 外部 SDK 头；0434 由 `adapter/perception_camera/CMakeLists.txt` 引用，随 `sap_camera.h` 被 `perception_camera_sdk_workshop.cpp` 使用。 |
| `sdk/sap_camera/include/perception_camera/sap_macros.h` | SDK 宏定义。 | 外部 SDK 头；0434 由 `adapter/perception_camera/CMakeLists.txt` 引用，随 `sap_camera` SDK 头链引入。 |
| `sdk/sap_camera/include/perception_camera/sap_image_info.h` | 图像输入相关结构。 | 外部 SDK 头；0434 `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp` 和 perception pipeline 通过 `sap_camera` SDK 间接使用。 |
| `sdk/sap_camera/include/perception_camera/sap_lidar_define.h` | lidar 输入基础结构。 | 外部 SDK 头；0434 Lidar 相关使用点包括 `adapter/perception_camera/src/convertors/perception/lidar_gop_convertor.cpp` 和 `perception_camera_sdk_workshop.cpp`。 |
| `sdk/sap_camera/include/perception_camera/sap_radar_define.h` | radar 输入基础结构。 | 外部 SDK 头；0434 radar/fusion 相关 convertor 通过 `sap_camera`/`sap_common` SDK 间接依赖，DLP 当前链路不直接启用 radar 输入。 |
| `sdk/sap_camera/include/perception_camera/interface/mdc_dvpp_dev.hpp` | mdc dvpp 设备接口声明。 | 外部 SDK 头；0434 由 `adapter/perception_camera/CMakeLists.txt` 引用，属于平台 SDK interface。 |
| `sdk/sap_common/include/perception_common/...` | `sap_camera` 头文件依赖的 perception_common 第三方头文件集合。按子目录提供基础类型、几何、目标、frame、状态估计、工具函数等依赖。 | 外部 `sap_common`/`perception_common` 头；0434 由 `adapter/perception_camera/CMakeLists.txt` 的 `Perception_Common_INCLUDE_DIRS` 引入，并被 `adapter/perception_camera/src/convertors/*` 与 `perception_camera_sdk_workshop.cpp` 使用。 |

`sap_camera` 数据结构头文件按用途划分如下。

| 文件 | 作用 | 0434 对应部分 |
| --- | --- | --- |
| `data_type/base_type.h` | SDK 基础类型定义。 | 外部 SDK 头；0434 通过 `adapter/perception_camera/CMakeLists.txt` 引入，供所有 `sap*` DTO 使用。 |
| `data_type/localization_info.h` | localization 基础 DTO。 | 外部 SDK 头；主要被 0434 `adapter/perception_camera/src/convertors/perception/local_localization_convertor.cpp` 和 `adapter/perception_camera/src/convertors/planning/global_localization_convertor.cpp` 使用。 |
| `data_type/frame/perception_object_frame.h` | `sapPerceptionObjectFrame`，相机目标输入 frame。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/perception/object_convertor.cpp` 和 `perception_camera_sdk_workshop.cpp` 的 object push 路径。 |
| `data_type/frame/perception_fusion_object_frame.h` | `sapPerceptionFusionObjectFrame`，fusion 目标输入 frame。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/perception/fusion_object_convertor.cpp`，默认 pipeline 未启用。 |
| `data_type/frame/perception_lidar_frame.h` | `sapPerceptionLidarGopFrame`，Lidar GOP 输入/输出 frame。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/perception/lidar_gop_convertor.cpp`。 |
| `data_type/frame/perception_road_geometry_frame.h` | `sapPerceptionRoadFrame`，道路几何输入 frame。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/perception/road_geometry_convertor.cpp`。 |
| `data_type/frame/perception_occ_frame.h` | `sapPerceptionOccFrame`，OCC 输入 frame。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/perception/occ_convertor.cpp`。 |
| `data_type/frame/perception_planning_frame.h` | `sapPerceptionPlanningFrame` 和 debug 输出结构。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/planning/prediction_convertor.cpp`、`adapter/perception_camera/src/convertors/planning/planning_debug_convertor.cpp` 和 `perception_camera_sdk_workshop.cpp` 的 SDK 输出路径。 |
| `data_type/frame/perception_gop_frame.h` | camera GOP frame 结构。 | 外部 SDK 头；0434 perception camera GOP 相关输入/输出结构，DLP 主要使用 Lidar GOP 路径。 |
| `data_type/frame/perception_blob_frame.h` | blob frame 结构。 | 外部 SDK 头；0434 perception camera 通用 frame 类型，DLP planning 主链路未直接迁移。 |
| `data_type/frame/perception_failsafe_frame.h` | failsafe frame 结构。 | 外部 SDK 头；0434 `perception_camera_sdk_workshop.cpp` 中 failsafe 输出/诊断相关路径使用。 |
| `data_type/frame/perception_fsline_frame.h` | freespace line frame 结构。 | 外部 SDK 头；0434 freespace/parking 相关路径使用，DLP planning 主链路未直接迁移。 |
| `data_type/frame/perception_road_semantics.h` | road semantics frame 结构。 | 外部 SDK 头；0434 road semantics 相关路径使用，DLP planning 主链路主要迁移 road geometry。 |
| `data_type/frame/perception_scene_frame.h` | scene frame 结构。 | 外部 SDK 头；0434 scene 相关 SDK DTO，DLP planning 主链路未直接迁移。 |
| `data_type/object/base_object.h` | `sapBaseObject` 和目标基础字段。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/perception/object_convertor.cpp`、`fusion_object_convertor.cpp`、`lidar_gop_convertor.cpp` 的基础目标字段。 |
| `data_type/object/base_camera_attribute.h` | camera landmark、方向等 attribute 结构。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/perception/object_convertor.cpp` 中 camera attribute/landmark 字段。 |
| `data_type/object/base_data_struct.h` | bbox、point、vector 等基础结构。 | 外部 SDK 头；0434 各 perception/planning convertor 中点、框、向量字段共用。 |
| `data_type/object/automobile.h` | `sapAutomobile`，车辆目标结构。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/perception/object_convertor.cpp` 的 vehicle 分支。 |
| `data_type/object/non_automobile_vehicle.h` | `sapNonAutomobileVehicle`，非机动车目标结构。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/perception/object_convertor.cpp` 的 non-vehicle 分支。 |
| `data_type/object/pedestrian.h` | `sapPedestrian`，行人目标结构。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/perception/object_convertor.cpp` 的 pedestrian 分支。 |
| `data_type/object/fusion_object.h` | `sapFusionObject`，fusion 目标结构。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/perception/fusion_object_convertor.cpp`。 |
| `data_type/object/lidar_object.h` | `sapLidarObject`、`sapLidarObstacle`，lidar/GOP 目标结构。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/perception/lidar_gop_convertor.cpp`。 |
| `data_type/object/board_pole.h` | 标牌杆类目标结构。 | 外部 SDK 头；0434 object/road marker 相关字段依赖，DLP planning 主链路未单独拆出。 |
| `data_type/object/road_marker_object.h` | road marker 目标结构。 | 外部 SDK 头；0434 object convertor 中 road marker/landmark 相关字段依赖。 |
| `data_type/object/static_obstacle.h` | 静态障碍物结构。 | 外部 SDK 头；0434 perception object 相关 DTO 依赖，DLP planning 主链路未单独拆出。 |
| `data_type/object/traffic_light.h` | 交通灯目标结构。 | 外部 SDK 头；0434 perception/planning traffic light 相关 DTO 依赖，DLP 默认 map traffic light 链路未启用。 |
| `data_type/object/traffic_sign.h` | 交通标识目标结构。 | 外部 SDK 头；0434 perception/planning traffic sign 相关 DTO 依赖。 |
| `data_type/planning/chassis_state.h` | `sapChassisState`，车辆处理状态输入。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/planning/vehicle_processing_convertor.cpp` 和 `perception_camera_sdk_workshop.cpp` 的 chassis push 路径。 |
| `data_type/planning/manual_signal.h` | `sapManualSignal`，由车辆状态机生成的人工/驾驶控制输入。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/workshops/perception/perception_camera_sdk_workshop.cpp::OnVehicleInfo()`。 |
| `data_type/planning/location_info.h` | `sapLocationInfo`，全局定位输入。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/planning/global_localization_convertor.cpp`，默认 pipeline 未启用。 |
| `data_type/planning/local_map.h` | road structure/local map 相关结构。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/perception/road_struct_convertor.cpp` 和 `adapter/perception_camera/src/convertors/planning/cross_info_convertor.cpp`。 |
| `data_type/planning/cross_info.h` | `sapCrossInfo`，路口信息输入。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/planning/cross_info_convertor.cpp`。 |
| `data_type/planning/navi_topo.h` | `sapNaviTopo`，导航拓扑输入。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/planning/navi_topo_convertor.cpp`。 |
| `data_type/planning/navigation_info.h` | `sapAmapNaviInfo`，高德导航输入。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/planning/amap_navi_info_convertor.cpp`。 |
| `data_type/planning/planning_info.h` | planning 输出中 prediction object/trajectory 结构。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/planning/prediction_convertor.cpp`。 |
| `data_type/planning/pred_map.h` | pred map 结构。当前 0434 pipeline 中 PredMapConvertor 未启用。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/planning/predmap_convertor.cpp`，默认 pipeline 注释关闭。 |
| `data_type/planning/route_base_info.h` | 路线基础信息结构。 | 外部 SDK 头；0434 腾讯地图/route 相关 convertor 依赖，DLP 默认未启用。 |
| `data_type/planning/route_guidance.h` | route guidance 结构。当前 0434 pipeline 默认未启用腾讯地图链路。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/planning/route_guidance_convertor.cpp`，默认 pipeline 注释关闭。 |
| `data_type/planning/route_lane_info.h` | route lane info 结构。当前 0434 pipeline 默认未启用腾讯地图链路。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/planning/lane_info_convertor.cpp`，默认 pipeline 注释关闭。 |
| `data_type/planning/route_light.h` | route traffic light 结构。当前 0434 pipeline 默认未启用腾讯地图链路。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/planning/traffic_light_convertor.cpp`，默认 pipeline 注释关闭。 |
| `data_type/road/fusion_slot.h` | fusion slot 结构。 | 外部 SDK 头；0434 parking/fusion slot 相关 convertor 使用，DLP planning 主链路未直接迁移。 |
| `data_type/road/occupancy_info.h` | OCC 内部栅格结构。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/perception/occ_convertor.cpp`。 |
| `data_type/sdmap/sdmap.h` | `sapSDMap`，SDMap 输入结构。 | 外部 SDK 头；0434 对应 `adapter/perception_camera/src/convertors/planning/sdmap_convertor.cpp`。 |
| `data_type/sdmap/route_loc.h` | route localization 结构。 | 外部 SDK 头；0434 SDMap/route localization 相关 DTO 依赖。 |

`sdk/sap_common` 文件数量较多，属于第三方基础库。移植代码不直接修改这些文件，
只通过 include 依赖它们。排查编译问题时通常只需要确认该目录完整，以及最终链接
阶段有匹配的 `libsap_common.so`。

## 5. 运行和构建注意事项

默认静态库构建：

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
```

RSCL 边界构建：

```bash
cmake -S . -B build_rscl -DUSHARING_DLP_NODE_BUILD_RSCL=ON
cmake --build build_rscl -j$(nproc)
```

可执行程序构建需要：

```text
sdk/sap_camera/lib/libsap_camera.so
sdk/sap_common/lib/libsap_common.so
```

当前目录只有头文件和资源，缺少上述两个动态库时，
`USHARING_DLP_NODE_BUILD_RSCL_APP=ON` 会在 CMake 配置阶段失败。

`build/`、`build_rscl_check/`、`build_converter_check/` 等目录是本地构建产物，
不应作为源代码或移植文档依据。
