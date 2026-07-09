# Runtime Resource Layout

This directory contains the DLP planning SDK resources needed by
`PlanningConfig::resource_root_path` when it is set to `./resource/`.

Included:

```text
config/dlp_node/x86/{sys_cfg,planning_cfg,topics_cfg}
config/dlp_node/thor/{sys_cfg,planning_cfg,topics_cfg}
config/x86/vd_mapqr_dlp/
config/perception_camera/x86/vd_mapqr_dlp/
licenses/perception_camera/SENSEAD.lic
licenses/perception_aggregator/GAC_SENSEADAS.lic
models/x86/vd_dlp/dlp_v1_009a9_rl_issue_aug_new/
```

`models/x86/vd_dlp/dlp_v1_009a9_rl_issue_aug_new/DLP.bin` is a relative
symlink to the copied x86_4090 DLP model binary because the source prototxt
uses the generic filename `DLP.bin`.

Vehicle calibration is expected under:

```text
resource/vehicle/default/
```

The current workspace did not contain a matching SenseAuto vehicle calibration
package. Put the target vehicle's sensor calibration files in that directory,
or update `resource/config/dlp_node/<platform>/planning_cfg` to another path
inside this module.
