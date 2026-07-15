SENSELOTU=/opt/senseauto/senselotu_em/1.20250918-001042-098cd.pilotmdc-rscl-x86.default.4664d8.pilotrscl-thor-x86-dev
LD_LIBRARY_PATH="$SENSELOTU/lib:$LD_LIBRARY_PATH" \
  ../bin/usharing_dlp_node_rscl_smoke \
  --config_dir ../resource/config/dlp_node/x86 \
  --run_seconds 30