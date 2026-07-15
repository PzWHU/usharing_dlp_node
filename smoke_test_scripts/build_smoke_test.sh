cmake -S .. -B ../bin \
  -DUSHARING_DLP_NODE_BUILD_RSCL=ON \
  -DUSHARING_DLP_NODE_BUILD_RSCL_APP=OFF \
  -DUSHARING_DLP_NODE_BUILD_RSCL_SMOKE_APP=ON \
  -DUSHARING_DLP_NODE_BUILD_SENSEAUTO_CONVERTERS=ON

cmake --build ../bin -j"$(nproc)" \
  --target usharing_dlp_node_rscl_smoke