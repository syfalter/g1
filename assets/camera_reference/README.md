# Nominal Camera Axis Inspection

Official source downloaded on 2026-09-08:
https://raw.githubusercontent.com/realsenseai/realsense-ros/ros2-master/realsense2_description/urdf/_d435.urdf.xacro

The source explicitly marks the internal offsets as approximate and recommends
device-calibrated extrinsics. Actual installed model and coordinate semantics
of the task DDS service remain to be confirmed.

The preview assumes Unitree `d435_link` equals RealSense `camera_link`, NOT
`bottom_screw_frame`. This identification has not been established by Unitree's
URDF. Do not use preview markers as a calibrated task transform.

- Blue: nominal depth optical +Z, which is body +X.
- Orange: nominal RGB optical +Z, parallel and displaced +0.015 m on body Y.
- Purple: original Unitree d435_link +Z, not the hypothesized optical axis.
- Transparent housing: official nominal 0.02505 x 0.090 x 0.025 m dimensions.
- Depth and infra1 origins coincide in the official nominal model.
- Optical rpy relative to body is [-pi/2, 0, -pi/2].
- Unitree torso -> d435: xyz [0.0576235, 0.01753, 0.42987] m;
  rpy [0, 0.8307767239493009, 0] rad.
- Under the stated assumption, optical Z in torso is
  [0.67430239, 0, -0.73845534], i.e. forward and downward.

During startup eth0 was DOWN without an IP. Preview therefore runs in isolated
localhost ROS domain 91, using the saved 15:02 pre-return joint snapshot. It is
not live robot telemetry. The model includes the updated bilateral RH56DFX and
4 cm length / 6 cm diameter connectors. No arm controller runs in domain 91.

Verified: torso->d435 TF values, nine markers, read-only RViz subscriptions to
both robot_description and MarkerArray, and zero /arm_sdk publishers in domain
91. Qt window screenshot was all black under WSL, so it is NOT visual proof.
Original control-domain processes were not restarted or changed.

Files: camera_axes.rviz, official_d435.urdf.xacro, offline_robot_params.yaml.
Publisher implementation: ../../scripts/camera_axis_preview.py.

## Vision task seed preview

`run_vision_task_preview.sh` validates one `vision_service` capture, uses the
saved D435I color intrinsics, applies the nominal Unitree camera mount plus the
device-reported depth-to-color extrinsic, and shows the resulting torso-frame
task seed in isolated localhost ROS domain 92. It never starts a camera owner,
hand service, arm gateway, planner, or trajectory executor.

The vision service returns 3-D points in `head_color_camera_optical_frame`
because depth has already been aligned and deprojected using the color
intrinsics. The conversion therefore uses
`T_torso_color = T_torso_depth * inverse(T_color_depth)`. RealSense serializes
the 3x3 extrinsic rotation column-major. Both details have non-symmetric
fixture coverage in `test_vision_task_geometry.py`.

Preview markers distinguish the raw operation point from its projection onto
an assumed horizontal shaft-top plane. A correction larger than 30 mm or a
radius outside 5 to 250 mm invalidates the planning seed, while retaining both
points for diagnosis. The service does not currently provide the shaft axis
orientation, so torso +Z is an explicit temporary assumption.

The bilateral index TCP estimates come from the articulated community RH56DFX
URDF and include the 40 mm wrist extension. They are model estimates, not
measured fingertip contact centers, and are not installed as calibrated real
controller tool profiles.

Synthetic transform/UI check:

```bash
./scripts/run_vision_task_preview.sh \
  assets/camera_reference/vision_task_synthetic.json
```

Real capture flow, after the independently deployed `vision_service` responds:

```bash
./scripts/run_task_vision_capture.sh eth0 logs/vision_task_capture_NEW.json
./scripts/run_vision_task_preview.sh logs/vision_task_capture_NEW.json
```

Close the RViz window to stop the preview. If it was launched as the transient
user service used during integration, stop it with
`systemctl --user stop g1-vision-task-preview.service`.

As of 2026-09-09 the preview also computes bilateral geometric 6-D grasp
candidates and displays the selected RH56DFX hand mesh plus wrist/TCP axes.
These are not IK solutions and have not passed collision checking. Candidate
orientation assumes a 20 degree tilt from the downward shaft normal.
The fourth positional launcher argument selects `left` (default), `right`,
or `both`; arguments two and three select intrinsics and the robot snapshot.
Low confidence and raw plane-height mismatch are retained as review reasons.
The first real capture is recorded under `logs/vision_live_20260909_104045`.

## RGB-D surface archive (2026-09-09)

Acquire a paired frame through the existing camera owner and download its saved
files using an already authorized SSH connection:

```bash
/usr/bin/python3 scripts/vision_rgbd.py --network eth0 --ssh unitree@192.168.123.164
```

Output defaults to `data/vision_captures/YYYYMMDD_HHMMSS_microseconds/`:
`capture.json`, `server_result.json`, `rgb.png`, `depth_aligned.png`,
`annotated.jpg`, `intrinsics.json`, `cloud.npz`, and `manifest.json`.
The manifest is published only after transfer, identity/encoding checks,
deprojection and checksums succeed. A failed download leaves the original DDS
response, not a pretend completed archive. Existing outputs are not overwritten.

Resume a saved request after SSH access is configured:

```bash
/usr/bin/python3 scripts/vision_rgbd.py \
  --capture data/vision_captures/20260909_rgbd_001/capture.json \
  --ssh unitree@192.168.123.164
```

Alternatively use `--server-record /path/to/head_rgbd_pedal_..._result.json`
with all three original server images in that JSON's local directory. Only
the paths returned by the service are fetched; the client does not start a
camera pipeline or modify the remote service. Passwords are not embedded.

`run_vision_task_preview.sh` automatically loads a completed archive beside
the capture JSON, or accepts its directory as positional argument five.
RViz provides a `Measured RGB-D surface - snapshot` layer. At the user's
request the old CAD cabinet/handle are not displayed; only the robot,
vision-derived targets and measured points belong in this preview. The colored
point cloud uses measured depth, not a flattened or fitted CAD surface. Display subsampling is three
pixels with a 0.1 to 2.0 m camera-depth range; the original full-resolution
uint16 depth remains unchanged. Zero-depth pixels are omitted. Nonzero lens
distortion is rejected rather than silently using an incorrect pinhole model.

Depth is **aligned to color**; use the matching color intrinsics and depth
scale, then the same nominal `T_torso_color` as the target markers. Frame
identity, dimensions, calibration, timestamps and cloud checksum are checked.
The camera mount remains nominal, not physically calibrated. The cloud is a
static observation, not a live collision map or a planner input. Removing CAD
from this visualization does not disable collisions in the separate historical
offline planner. Its fixed-CAD results are not valid for a differently arranged
real scene; measured collision geometry is still needed before execution.

The 2026-09-09 11:04 capture succeeded (sequence 15011) and requested images on
the server. Initial key-only SSH authentication was denied. After the user
provided authorized password access, SSH confirmed `192.168.123.164` is an
`NVIDIA Orin NX Developer Kit`; no additional network configuration was needed.
All four server files were downloaded into `server_originals/`, then validated
and imported into `data/vision_captures/20260909_rgbd_001/`. The archive now
contains all three images, metadata, manifest and 93,613 colored depth points.
The temporary authenticated SSH connection was closed; no password was saved.
The server record is
`/home/unitree/rh56dfx/workspace/captures/head_rgbd_pedal_20260909_110447_942_result.json`.

Live ROS verification is in `rviz_cloud_verification.json`: frame `torso_link`,
93,613 finite points, RViz subscribed, no CAD publisher or motion command
topics. This is the saved 11:04 snapshot, not a continuously updated camera
feed, and is not a calibrated/validated collision map. The downloaded
annotation image was visually inspected; RViz rendering itself was verified
through message data/subscriptions, not a successful GUI screenshot.

Pair checking keeps sequence, both frame numbers and camera identity exact.
Double serialization differences are accepted only within 0.001 ms absolute
timestamp error (no epoch-relative tolerance) and 1e-10 absolute calibration
numeric error. Regression tests reject genuine frame/calibration mismatches.

## Horizontal ray-plane entry preview (2026-09-09)

With the user's approval, schema version 2 defaults to
`operation_diagonal_intersection`, before the upstream recognizer's 1.35x
pixel-space outward offset. The service is unchanged. Its original XYZ and
pixels remain in the archive; local selection bypasses the outward offset.

The selected camera ray intersects a horizontal plane through the shaft-center
candidate, with torso +Z as its normal. The ray is obtained from the calibrated
camera origin and the service's transformed XYZ; no second depth scaling or
pixel deprojection is applied. Near-parallel, backward, degenerate and
out-of-range intersections are rejected. A local in-plane radial adjustment
defaults to zero, with a +/-3 cm configuration bound; positive is outward.

`task_preview.insertion_reference_m` is the selected target for BOTH hand
candidates, preinsert references and the circular sweep. The old
`operation_point_projected_m` field is retained strictly as the legacy Z-only
comparison. `ray_plane_intersections_m` retains both the pre-offset and
upstream-offset intersections. The 30 mm raw-depth/plane discrepancy gate now
applies to the selected source; the upstream point's discrepancy and low
confidence still appear separately as review reasons. Neither this gate nor a
valid geometric seed means that IK, collisions or hardware execution passed.

Configuration examples (preview only):

```bash
G1_VISION_ENTRY_KEY=operation_diagonal_intersection \
G1_VISION_RADIAL_OFFSET_M=0 ./scripts/run_vision_task_preview.sh CAPTURE_JSON
```

For an upstream-offset comparison, set `G1_VISION_ENTRY_KEY=operation_point`;
that still uses ray-plane intersection, never the old Z-only projection.
The geometry CLI exposes the equivalent `--entry-key` and
`--radial-offset-m` arguments. No upstream service or real arm parameter is
modified by either option.

RViz cyan is the selected entry/hand/sweep, gray the legacy Z-only reference,
and orange the upstream point comparisons. Original RGB-D stays unchanged.
For sequence 102835, the new target is [0.426933298, 0.081514427, -0.024890133]
m in torso, radius 55.995 mm (previous Z-only radius 102.386 mm). The source
frame is deliberately reused for comparison, not presented as a new capture.
Evidence is `ray_plane_preview.json` and `ray_plane_rviz_verification.json`
beside that capture: 29 markers, 93828 depth points, matching displayed
fingertip/arc/entry, no CAD or motion command topics. 61 regression tests pass;
no IK/collision/real motion verification was performed for this new target.
