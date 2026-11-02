# Offline Workbench Preview

This scene is isolated on ROS domain 77 with localhost-only communication.
It has no hardware gateway, robot-state bridge, or trajectory execution.

`workbench.stl` retains the STEP dimensions in metres. The preview uses pitch
-90 degrees and yaw 180 degrees, with the cabinet bottom aligned to the static
robot's foot meshes. The main work surface is approximately 0.710 m above the
cabinet bottom; the 0.830 m CAD top-face reference is obsolete after rotation.
With `align_bottom_to_feet=true`, bottom alignment overrides `table_height`.

`initial_pose.json` records the historical right-arm measurement and explicitly
marks the left arm as a mirrored estimate. Neither is a verified R2+A pose.
The current default is `task_start_pose.json`: a separate synthetic candidate
with both shoulder-pitch joints increased by 0.20 rad, keeping the historical
record unchanged. It has static cabinet clearance at the 0.06 m robot retreat;
the entrance motion and full MoveIt self-collision validity are not yet certified.

`payload_preview.json` now describes matching 50 mm long, 60 mm diameter left/right
connector, replacing the old box. It starts at the vendor wrist mounting face;
the hand and tool0 move outward by 50 mm. Both hands now use the community
RH56DFX mesh in a baked open posture, replacing the vendor rubber hands and
old hand collision boxes. Both hand mounts and tool0 frames extend by 50 mm,
with mirrored original lateral offsets. Installation rotation/translation
is approximate and must be checked against the real hands. The camera is not
separately modeled. No hardware URDF, dynamics or tool profiles change.
Both the preview and offline checker use `scripts/preview_payload.py`.

Run `scripts/prepare_rh56dfx.py` in the CAD environment with the ROS environment
sourced to regenerate the hand meshes. Source provenance, mount assumptions
and the static finger pose are in `rh56dfx/provenance.json`. Upstream source
and its license are preserved in `assets/rh56dfx_description`; this is not an
official manufacturer release. The upstream additional wrist mechanism is
excluded to avoid duplicating G1 wrist joints. No hand controller is started.

`robot_retreat` moves the entire preview robot in world -X without moving the
cabinet. The actual pelvis-front to table-edge gap is `table_gap + robot_retreat`.
The pelvis front reference is the maximum X of the pelvis visual meshes, not
a physical calibration of the real robot's housing.

`scripts/check_workbench_retreat.py` checks the fixed default scene and initial
pose in 5 mm increments, seeking at least 20 mm model clearance. It writes
`retreat_check.json`. It must be rerun if the pose or geometry changes. The
check includes all available visual/collision geometries but does not certify
real-world safety, startup motion, self-collision, walking or swept motion.
