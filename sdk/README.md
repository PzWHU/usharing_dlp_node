# SDK Layout

`usharing_dlp_node` is intended to build from this directory without reading
headers from sibling projects or download folders.

Required local layout:

```text
sdk/
  sap_camera/
    include/perception_camera/...
    lib/libsap_camera.so
  sap_common/
    include/perception_common/...
    lib/libsap_common.so
```

The `include` trees are copied into this module. The two `.so` files are not
present in the current workspace; copy them to the `lib` directories above
before building the standalone executable with `BUILD_RSCL_APP=ON`.

System-installed SenseAuto RSCL, ADMsgIDL, Cap'n Proto, and JsonCpp are still
resolved through `/opt/senseauto` or the system package paths because they are
the selected RSCL communication boundary for this migration stage.
