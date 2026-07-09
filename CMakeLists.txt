cmake_minimum_required(VERSION 3.16)

project(usharing_dlp_node LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SAP_CAMERA_ROOT
    "${CMAKE_CURRENT_LIST_DIR}/sdk/sap_camera"
    CACHE PATH "Path to sap_camera SDK package")
set(SAP_COMMON_ROOT
    "${CMAKE_CURRENT_LIST_DIR}/sdk/sap_common"
    CACHE PATH "Path to sap_common/perception_common package")
set(SENSEAUTO_MSG_ROOT
    "/opt/senseauto/senseauto-msgs/active"
    CACHE PATH "Path to SenseAuto generated Cap'n Proto message package")
set(JSONCPP_ROOT
    "/usr"
    CACHE PATH "Path to JsonCpp package")
set(SENSEAUTO_3RDPARTY_ROOT
    "/opt/senseauto/senseauto-3rdparty/active/3rdparty"
    CACHE PATH "Path to SenseAuto 3rdparty active package")
set(USHARING_DLP_NODE_CONFIG_PLATFORM
    "x86"
    CACHE STRING "Default runtime config platform under resource/config/dlp_node")
set_property(CACHE USHARING_DLP_NODE_CONFIG_PLATFORM PROPERTY STRINGS x86 thor)
set(USHARING_DLP_NODE_DEFAULT_CONFIG_SUBDIR
    "config/dlp_node/${USHARING_DLP_NODE_CONFIG_PLATFORM}"
    CACHE STRING "Default runtime config subdirectory under resource")
option(USHARING_DLP_NODE_BUILD_ROS2 "Build ROS2 transport boundary" OFF)
option(USHARING_DLP_NODE_BUILD_RSCL
       "Build planning_bus-style RSCL transport boundary" OFF)
option(USHARING_DLP_NODE_BUILD_RSCL_APP
       "Build standalone RSCL executable; requires libsap_camera.so and libsap_common.so"
       OFF)
option(USHARING_DLP_NODE_BUILD_SENSEAUTO_CONVERTERS
       "Build SenseAuto Cap'n Proto converters used by the RSCL transport" OFF)
option(USHARING_DLP_NODE_HAS_SAP_PUSH_LIDAR_GOP_FRAME
       "Enable sapPushLidarGopFrame calls when using a 0434 sap_camera SDK that declares/exports it"
       OFF)

# Backward compatibility for old local commands while the migrated project is
# being renamed from planning_port to usharing_dlp_node.
if(DEFINED PLANNING_PORT_BUILD_ROS2)
  set(USHARING_DLP_NODE_BUILD_ROS2 "${PLANNING_PORT_BUILD_ROS2}"
      CACHE BOOL "Build ROS2 transport boundary" FORCE)
endif()
if(DEFINED PLANNING_PORT_BUILD_RSCL)
  set(USHARING_DLP_NODE_BUILD_RSCL "${PLANNING_PORT_BUILD_RSCL}"
      CACHE BOOL "Build planning_bus-style RSCL transport boundary" FORCE)
endif()
if(DEFINED PLANNING_PORT_BUILD_RSCL_APP)
  set(USHARING_DLP_NODE_BUILD_RSCL_APP "${PLANNING_PORT_BUILD_RSCL_APP}"
      CACHE BOOL "Build standalone RSCL executable" FORCE)
endif()
if(DEFINED PLANNING_PORT_BUILD_SENSEAUTO_CONVERTERS)
  set(USHARING_DLP_NODE_BUILD_SENSEAUTO_CONVERTERS
      "${PLANNING_PORT_BUILD_SENSEAUTO_CONVERTERS}"
      CACHE BOOL "Build SenseAuto Cap'n Proto converters" FORCE)
endif()

set(USHARING_DLP_NODE_COMMON_INCLUDE_DIRS
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/sap
  ${SAP_CAMERA_ROOT}/include
  ${SAP_COMMON_ROOT}/include
  ${SAP_COMMON_ROOT}/include/perception_common
  ${SAP_COMMON_ROOT}/include/perception_common/proto
)

set(USHARING_DLP_NODE_SENSEAUTO_CONVERTER_SOURCES
  transport/rscl/converters/amap_navi_info_converter.cpp
  transport/rscl/converters/diag_converter.cpp
  transport/rscl/converters/fusion_object_frame_converter.cpp
  transport/rscl/converters/lidar_gop_frame_converter.cpp
  transport/rscl/converters/local_localization_converter.cpp
  transport/rscl/converters/location_info_converter.cpp
  transport/rscl/converters/object_common.cpp
  transport/rscl/converters/object_frame_converter.cpp
  transport/rscl/converters/occupancy_info_converter.cpp
  transport/rscl/converters/planning_debug_converter.cpp
  transport/rscl/converters/prediction_converter.cpp
  transport/rscl/converters/road_geometry_frame_converter.cpp
  transport/rscl/converters/scene_navi_map_converter.cpp
  transport/rscl/converters/vehicle_processing_converter.cpp
  transport/rscl/converters/vehicle_report_converter.cpp
)

add_library(usharing_dlp_node_core STATIC
  core/planning_service.cpp
  core/vehicle_state_machine.cpp
)

target_include_directories(usharing_dlp_node_core
  PUBLIC
    ${USHARING_DLP_NODE_COMMON_INCLUDE_DIRS}
)

add_library(usharing_dlp_node_sap STATIC
  sap/sap_planning_engine.cpp
  sap/sap_raii.cpp
)

target_include_directories(usharing_dlp_node_sap
  PUBLIC
    ${USHARING_DLP_NODE_COMMON_INCLUDE_DIRS}
)

target_link_directories(usharing_dlp_node_sap
  PUBLIC
    ${SAP_CAMERA_ROOT}/lib
    ${SAP_COMMON_ROOT}/lib
)

target_link_libraries(usharing_dlp_node_sap
  PUBLIC
    usharing_dlp_node_core
    sap_camera
    sap_common
)
if(USHARING_DLP_NODE_HAS_SAP_PUSH_LIDAR_GOP_FRAME)
  target_compile_definitions(usharing_dlp_node_sap
    PUBLIC
      USHARING_DLP_NODE_HAS_SAP_PUSH_LIDAR_GOP_FRAME=1
  )
endif()

if(USHARING_DLP_NODE_BUILD_RSCL OR
   USHARING_DLP_NODE_BUILD_SENSEAUTO_CONVERTERS)
  # Source mapping:
  #   rscl_install-V1.0/example/4_raw_pubsub/CMakeLists.txt
  # Difference from source:
  #   This module uses the same SenseAuto package discovery, but keeps the
  #   RSCL target optional so usharing_dlp_node_core remains middleware-neutral.
  set(BUILDTOOLS_TARGET_PLATFORM "x86_64" CACHE STRING
      "SenseAuto buildtools target platform")
  list(APPEND CMAKE_PREFIX_PATH
    /opt/senseauto/senseauto-buildtools/active
    /opt/senseauto/senseauto-framework-sdk/active/cmake
    /opt/senseauto/senseauto-msgs/active/cmake/ADMsgIDL
  )
  find_package(ADBuildTools REQUIRED
    PATHS /opt/senseauto/senseauto-buildtools/active)
  find_senseauto_package(senseauto-msgs ADMsgIDL REQUIRED)
  if(USHARING_DLP_NODE_BUILD_RSCL)
    find_senseauto_package(senseauto-framework-sdk ADFramework REQUIRED
      COMPONENTS ad_log ad_common ad_scm ad_time ad_coordutils ad_utils
                 ad_rscl ad_msg_bridge)
  endif()
endif()

if(USHARING_DLP_NODE_BUILD_SENSEAUTO_CONVERTERS)
  add_library(usharing_dlp_node_senseauto_converters STATIC
    ${USHARING_DLP_NODE_SENSEAUTO_CONVERTER_SOURCES}
  )
  target_include_directories(usharing_dlp_node_senseauto_converters
    PUBLIC
      ${USHARING_DLP_NODE_COMMON_INCLUDE_DIRS}
      ${ADMsgIDL_INCLUDE_DIRS}
      ${SENSEAUTO_MSG_ROOT}/include
      ${SENSEAUTO_3RDPARTY_ROOT}/include
      ${SENSEAUTO_3RDPARTY_ROOT}/capnp/include
      ${SENSEAUTO_3RDPARTY_ROOT}/kj/include
  )
  target_link_directories(usharing_dlp_node_senseauto_converters
    PUBLIC
      ${ADMsgIDL_LIBRARY_DIRS}
      ${SENSEAUTO_MSG_ROOT}/lib
      ${SENSEAUTO_3RDPARTY_ROOT}/lib
      ${SENSEAUTO_3RDPARTY_ROOT}/capnp/lib
      ${SENSEAUTO_3RDPARTY_ROOT}/kj/lib
  )
  target_link_libraries(usharing_dlp_node_senseauto_converters
    PUBLIC
      usharing_dlp_node_core
      ad_prediction_capnp
      perception_capnp
      ad_framework_sdk_capnp
      ad_navi_capnp
      l2plus_maplesslm_capnp
      l2plus_sdmap_capnp
      L2Plus_localmap_msgs_capnp
      ad_vehicle_capnp
      ad_planning_capnp
      ad_localization_capnp
      ad_sensor_capnp
      std_msgs_capnp
      capnp
      kj
  )
endif()

if(USHARING_DLP_NODE_BUILD_RSCL)
  find_path(JSONCPP_INCLUDE_DIR
    NAMES json/json.h
    PATHS
      ${JSONCPP_ROOT}/include
      ${JSONCPP_ROOT}/include/jsoncpp
      /usr/include/jsoncpp
      /opt/libs-x64/jsoncpp/jsoncpp-1.9.5/include
      ${SENSEAUTO_3RDPARTY_ROOT}/jsoncpp/include
  )
  find_library(JSONCPP_LIBRARY
    NAMES jsoncpp
    PATHS
      ${JSONCPP_ROOT}/lib
      ${JSONCPP_ROOT}/lib/x86_64-linux-gnu
      /usr/lib/x86_64-linux-gnu
      /opt/libs-x64/jsoncpp/jsoncpp-1.9.5/lib
      ${SENSEAUTO_3RDPARTY_ROOT}/jsoncpp/lib
  )
  if(NOT JSONCPP_INCLUDE_DIR OR NOT JSONCPP_LIBRARY)
    message(FATAL_ERROR
      "USHARING_DLP_NODE_BUILD_RSCL=ON requires JsonCpp headers/libs. "
      "Set JSONCPP_ROOT to a JsonCpp installation.")
  endif()

  add_library(usharing_dlp_node_rscl STATIC
    config/rscl_config.cpp
    ${USHARING_DLP_NODE_SENSEAUTO_CONVERTER_SOURCES}
    transport/rscl/subscribers/amap_navi_info/amap_navi_info_subscriber.cpp
    transport/rscl/subscribers/fusion_object_frame/fusion_object_frame_subscriber.cpp
    transport/rscl/subscribers/lidar_gop_frame/lidar_gop_frame_subscriber.cpp
    transport/rscl/subscribers/local_localization/local_localization_subscriber.cpp
    transport/rscl/subscribers/location_info/location_info_subscriber.cpp
    transport/rscl/subscribers/object_frame/object_frame_subscriber.cpp
    transport/rscl/subscribers/occupancy_info/occupancy_info_subscriber.cpp
    transport/rscl/subscribers/road_geometry_frame/road_geometry_frame_subscriber.cpp
    transport/rscl/subscribers/scene_navi_map/scene_navi_map_subscriber.cpp
    transport/rscl/subscribers/vehicle_processing/vehicle_processing_subscriber.cpp
    transport/rscl/subscribers/vehicle_report/vehicle_report_subscriber.cpp
    transport/rscl/rscl_access.cpp
    transport/rscl/rscl_planning_transport.cpp
  )
  target_include_directories(usharing_dlp_node_rscl
    PUBLIC
      ${USHARING_DLP_NODE_COMMON_INCLUDE_DIRS}
      ${CMAKE_CURRENT_LIST_DIR}/transport/rscl
      ${ADFramework_INCLUDE_DIRS}
      ${ADMsgIDL_INCLUDE_DIRS}
      ${SENSEAUTO_MSG_ROOT}/include
      ${SENSEAUTO_3RDPARTY_ROOT}/include
      ${SENSEAUTO_3RDPARTY_ROOT}/capnp/include
      ${SENSEAUTO_3RDPARTY_ROOT}/kj/include
      ${SENSEAUTO_3RDPARTY_ROOT}/protobuf/include
      ${JSONCPP_INCLUDE_DIR}
  )
  target_compile_definitions(usharing_dlp_node_rscl
    PUBLIC
      USHARING_DLP_NODE_DEFAULT_CONFIG_SUBDIR="${USHARING_DLP_NODE_DEFAULT_CONFIG_SUBDIR}"
  )
  target_link_directories(usharing_dlp_node_rscl
    PUBLIC
      ${ADFramework_LIBRARY_DIRS}
      ${ADMsgIDL_LIBRARY_DIRS}
      ${SENSEAUTO_MSG_ROOT}/lib
      ${SENSEAUTO_3RDPARTY_ROOT}/lib
      ${SENSEAUTO_3RDPARTY_ROOT}/capnp/lib
      ${SENSEAUTO_3RDPARTY_ROOT}/kj/lib
      ${SENSEAUTO_3RDPARTY_ROOT}/protobuf/lib
  )
  target_link_libraries(usharing_dlp_node_rscl
    PUBLIC
      usharing_dlp_node_core
      ${JSONCPP_LIBRARY}
      ${ADFramework_LIBRARIES}
      ad_prediction_capnp
      perception_capnp
      ad_framework_sdk_capnp
      ad_navi_capnp
      l2plus_maplesslm_capnp
      l2plus_sdmap_capnp
      L2Plus_localmap_msgs_capnp
      ad_vehicle_capnp
      ad_planning_capnp
      ad_localization_capnp
      ad_sensor_capnp
      std_msgs_capnp
      capnp
      kj
  )

  if(USHARING_DLP_NODE_BUILD_RSCL_APP)
    find_library(SAP_CAMERA_LIBRARY
      NAMES sap_camera
      PATHS
        ${SAP_CAMERA_ROOT}/lib
        ${SAP_CAMERA_ROOT}/build/lib
      NO_DEFAULT_PATH
    )
    find_library(SAP_COMMON_LIBRARY
      NAMES sap_common
      PATHS
        ${SAP_COMMON_ROOT}/lib
        ${SAP_COMMON_ROOT}/build/lib
      NO_DEFAULT_PATH
    )
    if(NOT SAP_CAMERA_LIBRARY OR NOT SAP_COMMON_LIBRARY)
      message(FATAL_ERROR
        "USHARING_DLP_NODE_BUILD_RSCL_APP=ON requires libsap_camera.so and "
        "libsap_common.so. Copy them to "
        "usharing_dlp_node/sdk/sap_camera/lib and "
        "usharing_dlp_node/sdk/sap_common/lib, or set "
        "SAP_CAMERA_ROOT/SAP_COMMON_ROOT explicitly.")
    endif()

    add_executable(usharing_dlp_node_main_rscl
      app/main_rscl.cpp
    )
    target_include_directories(usharing_dlp_node_main_rscl
      PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/transport/rscl
    )
    target_link_libraries(usharing_dlp_node_main_rscl
      PRIVATE
        usharing_dlp_node_rscl
        usharing_dlp_node_sap
        ${SAP_CAMERA_LIBRARY}
        ${SAP_COMMON_LIBRARY}
    )
  endif()
endif()

if(USHARING_DLP_NODE_BUILD_ROS2)
  find_package(rclcpp REQUIRED)
  add_library(usharing_dlp_node_ros2 STATIC
    transport/ros2/ros2_transport.cpp
  )
  target_include_directories(usharing_dlp_node_ros2
    PUBLIC
      ${USHARING_DLP_NODE_COMMON_INCLUDE_DIRS}
  )
  target_link_libraries(usharing_dlp_node_ros2
    PUBLIC
      usharing_dlp_node_core
      rclcpp::rclcpp
  )
endif()
