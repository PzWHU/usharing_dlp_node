# RSCL Bag Smoke Test Report

## 1. 测试目标

验证 `rsclbag0714.000.rsclbag` 回放时，本程序的 RSCL 输入链路可以正常完成：

```text
RSCL message
  -> transport/rscl/subscribers/*
  -> transport/rscl/converters/*
  -> sap* DTO
  -> PlanningService on_xxx callback
  -> IPlanningEngine::PushXxx
```

本次测试不加载真实 `sap_camera` runtime，不调用真实 SDK 算法库，只验证“消息收发、转换、传入 SDK 接口”是否正常。

## 2. 测试环境

| 项目 | 内容 |
| --- | --- |
| 测试日期 | 2026-07-15 |
| 工程目录 | `/home/lenovo/project/usharing_dlp_node` |
| 数据包 | `/home/lenovo/下载/rsclbag0714.000.rsclbag` |
| RSCL 文档 | `/home/lenovo/下载/商汤RSCL中间件安装及开发指导文档.docx` |
| 配置目录 | `resource/config/dlp_node/x86` |
| RSCL domain | `CYBER_DOMAIN_ID: 123` |
| 数据包时间 | 2026-07-14 18:15:11 至 2026-07-14 18:15:18，约 7.05 秒 |

本机同时存在 ROS Humble 的 FastDDS 库。运行 RSCL 程序时需要优先加载 SenseAuto 的 FastDDS，否则会出现 `libcyber.so` 符号不匹配。

```bash
SENSELOTU=/opt/senseauto/senselotu_em/1.20250918-001042-098cd.pilotmdc-rscl-x86.default.4664d8.pilotrscl-thor-x86-dev
export LD_LIBRARY_PATH="$SENSELOTU/lib:$LD_LIBRARY_PATH"
```

## 3. 测试程序

新增 smoke 测试可执行程序：

| 文件 | 作用 |
| --- | --- |
| `app/main_rscl_smoke.cpp` | 创建 fake `IPlanningEngine`，统计所有 `Push*` 和 `SetPlanningInfo` 调用次数。 |
| `app/rscl_smoke_sap_stubs.cpp` | 提供测试专用 sap DTO 初始化/释放 stub，避免链接真实 `libsap_camera.so`、`libsap_common.so`。 |
| `CMakeLists.txt` | 新增 `USHARING_DLP_NODE_BUILD_RSCL_SMOKE_APP` 选项和 `usharing_dlp_node_rscl_smoke` target。 |

同时修复了 `CRsclAccess::release()` 的 RSCL runtime 所有权问题：`senseAD::rscl::GetCurRuntime()` 返回 RSCL 内部单例，本工程只应调用 `Shutdown()`，不能手动 `delete`。

## 4. 构建步骤

```bash
cmake -S . -B build_rscl_smoke \
  -DUSHARING_DLP_NODE_BUILD_RSCL=ON \
  -DUSHARING_DLP_NODE_BUILD_RSCL_APP=OFF \
  -DUSHARING_DLP_NODE_BUILD_RSCL_SMOKE_APP=ON \
  -DUSHARING_DLP_NODE_BUILD_SENSEAUTO_CONVERTERS=ON

cmake --build build_rscl_smoke -j"$(nproc)" \
  --target usharing_dlp_node_rscl_smoke
```

构建结果：通过。

备注：构建过程中 SDK 头文件存在已知 warning：`MAX_OP_TIME_COST_NUM` 重复定义，不影响本次 smoke 验证。

## 5. 执行步骤

### 5.1 空跑验证

先验证 smoke 程序在不回放数据时能正常启动和退出。

```bash
SENSELOTU=/opt/senseauto/senselotu_em/1.20250918-001042-098cd.pilotmdc-rscl-x86.default.4664d8.pilotrscl-thor-x86-dev
LD_LIBRARY_PATH="$SENSELOTU/lib:$LD_LIBRARY_PATH" \
  ./build_rscl_smoke/usharing_dlp_node_rscl_smoke \
  --config_dir resource/config/dlp_node/x86 \
  --run_seconds 2
```

结果：退出码 `0`，所有计数为 `0`，说明 RSCL runtime 初始化和退出路径正常。

### 5.2 启动 smoke 接收端

```bash
SENSELOTU=/opt/senseauto/senselotu_em/1.20250918-001042-098cd.pilotmdc-rscl-x86.default.4664d8.pilotrscl-thor-x86-dev
LD_LIBRARY_PATH="$SENSELOTU/lib:$LD_LIBRARY_PATH" \
  ./build_rscl_smoke/usharing_dlp_node_rscl_smoke \
  --config_dir resource/config/dlp_node/x86 \
  --run_seconds 30
```

启动后等待数秒，确保订阅已注册。

### 5.3 回放 rsclbag

```bash
SENSELOTU=/opt/senseauto/senselotu_em/1.20250918-001042-098cd.pilotmdc-rscl-x86.default.4664d8.pilotrscl-thor-x86-dev
LD_LIBRARY_PATH="$SENSELOTU/lib:$LD_LIBRARY_PATH" \
  rsclbag simple_play \
  --included \
    /perception/nv_cameras/object \
    /perception/nv_cameras/road_geometry \
    /perception/lidar_gop/object \
    /perception/nv_cameras/occ_info \
    /localization/odomstate_info \
    /canbus/vehicle_report \
    /mcu/vehicle_processing \
    /maplesslm/scene_navi_map \
    /amap/nav_realtime/info \
  --play_rate 1 \
  /home/lenovo/下载/rsclbag0714.000.rsclbag
```

回放日志显示：

```text
Play Begin Time(ns): 1784024111094197430 (2026-07-14 18:15:11 GMT+0800)
Play End Time(ns):   1784024118146970690 (2026-07-14 18:15:18 GMT+0800)
Initial Play Rate:   1.000000
------ Play End -------
```

备注：`rsclbag simple_play` 命令在本机返回码为 `1`，但日志显示完整播放到 `Play End`。本次通过 smoke 接收端退出码和计数判断链路结果。

## 6. 测试结果

smoke 接收端最终退出码：`0`。

fake `IPlanningEngine` 收到的调用计数：

| 接口计数 | 次数 | 说明 |
| --- | ---: | --- |
| `push_object` | 54 | ObjectFrame 已收到并转换为 `sapPerceptionObjectFrame`。 |
| `push_road` | 56 | RoadGeometry 已收到并转换为 `sapPerceptionRoadFrame`。 |
| `push_fusion_object` | 0 | 本次回放未验证 fusion object 链路。 |
| `push_lidar_gop` | 56 | Lidar GOP 已收到并转换为 `sapPerceptionLidarGopFrame`。 |
| `push_occ` | 56 | Occupancy 已收到并转换为 `sapPerceptionOccFrame`。 |
| `push_local_localization` | 677 | 定位输入已收到并转换为 `sapLocalLocalizationInfo`。 |
| `push_location_info` | 0 | 当前回放/配置未触发 location info 链路。 |
| `push_sdmap` | 118 | SceneNaviMap 已拆分并传入 SDMap。 |
| `push_cross_info` | 118 | SceneNaviMap 已拆分并传入 CrossInfo。 |
| `push_road_structure` | 118 | SceneNaviMap 已拆分并传入 RoadStructure。 |
| `push_navi_topo` | 118 | SceneNaviMap 已拆分并传入 NaviTopo。 |
| `push_manual_signal` | 688 | VehicleReport 经 `VehicleStateMachine` 生成 manual signal 后传入 engine。 |
| `push_chassis_state` | 0 | 配置订阅 `/mcu/vehicle_processing`，该 topic 在本数据包中未出现。 |
| `push_amap_navi_info` | 7 | AMap 导航信息已收到并转换为 `sapAmapNaviInfo`。 |
| `set_planning_info` | 688 | Vehicle 状态机更新 planning info。 |

关键结果与数据包 topic 数量对齐：

| 数据链路 | 数据包消息数 | smoke 传入次数 | 结论 |
| --- | ---: | ---: | --- |
| `/perception/nv_cameras/object` | 54 | 54 | 通过 |
| `/perception/nv_cameras/road_geometry` | 56 | 56 | 通过 |
| `/perception/lidar_gop/object` | 56 | 56 | 通过 |
| `/perception/nv_cameras/occ_info` | 56 | 56 | 通过 |
| `/maplesslm/scene_navi_map` | 118 | 118 | 通过 |
| `/amap/nav_realtime/info` | 7 | 7 | 通过 |

## 7. 结论

本次 smoke 测试通过。

已验证当前代码在不加载真实 SDK 算法库的情况下，可以从 RSCL bag 收到主要输入消息，完成 SenseAuto RSCL message 到 `sap*` DTO 的转换，并通过 `PlanningService` 正确传入 `IPlanningEngine::Push*` 接口。

本次未验证内容：

- 真实 `SapPlanningEngine` 调用 `sap_camera` SDK。
- SDK 输出 prediction/debug/diag 发布。
- `/mcu/vehicle_processing` 对应的 chassis state 链路，因为测试包未包含该 topic。
- fusion object 和 location info 链路，因为本次回放未触发对应输入。

后续如果要验证到真实 SDK，需要在运行环境中提供匹配版本的 `libsap_camera.so`、`libsap_common.so` 和完整 DLP 资源，并改用正式 `usharing_dlp_node_main_rscl` 入口。
