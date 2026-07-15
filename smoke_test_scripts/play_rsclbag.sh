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