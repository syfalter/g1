# Mirrored Preparation Release (2026-09-28)

This update changes trajectories, not ELN task-to-action mappings. No physical
initialization, recovery, control acquisition or motion was requested during
deployment. The exported plans remain `planned`, not physically verified.

## Selected Actions

| Action | Sequence | Pinned preview seconds |
| --- | --- | ---: |
| `dring_pair` | Left once, right once, return | 74.605 |
| `push` | Prepare hand, approach, push, withdraw, return, open | 31.105 |
| `pull` | Open, approach, grip, pull, open, withdraw, return | 39.024 |
| `gear_roundtrip_lower_upper` | Enter LOWER, LOWER-UPPER-LOWER-UPPER, exit | 58.707 |
| `gear_roundtrip_upper_lower` | Enter UPPER, UPPER-LOWER-UPPER-LOWER, exit | 73.449 |

Preview time excludes pre-task alignment from the actual measured position and
controller completion waits. Gear execution also has an initial 1-second hand
opening step. Motion completion does not verify WOW or gear-lock signals.

## Preparation And Geometry

- Saved right above-table now uses the existing `mirror_left_ready` convention.
- All four right-arm task starts and returns match Jetson's initialization
  target, including waist yaw. Export rejects mismatched task starts.
- The previous non-mirrored right pose is backed up at
  `data/rail_deployment/approved_v5_20260928/ready_poses_before_mirror.json`.
- Rail no longer travels from the non-mirrored pose back to the mirrored pose.
- Gear entry/exit were replanned from the mirrored pose; `lu`, `ul`, and both
  repeated-cycle units are byte-for-byte unchanged from the approved V5 preview.
- Local low-motion compression retains endpoints and analytically checks
  position, velocity, acceleration, jerk and spatial-deviation constraints.
  Not every low-speed region can be removed within these constraints.
- The repeated final OPEN command in the pull cycle was removed. Required hand
  transitions remain, with the same one-second duration.

## Preview

The offline editor loads the pinned result directly in the optimization tab.
Select the full push/pull entries, not the individual `push`/`pull` recordings.
No optimization-generation click is needed. ROS uses localhost domain 89 only.

```bash
bash /home/syf/g1_upper_body_ws/scripts/task_editor_offline.sh start
```

```bash
bash /home/syf/g1_upper_body_ws/scripts/task_editor_offline.sh stop
```

## Unchanged Interfaces

- Execution: `eln_executor.service`, loopback port 8795.
- LAN relay: `eln_interface.service`, port 8090.
- Read-only observation: `eln_monitor.service`, port 8092.
- Existing independent dispatch: `POST /api/v1/actions/{action}/tasks`.
- Initialization remains explicit. R2+A alone does not initialize or acquire.
- The new proposed `GEAR_OPERATION` mapping was withdrawn before deployment.
  Its correspondence with rod positions and cycle counts awaits user agreement.
- No protocol parser, callback behavior or cloud mapping was changed by this
  mirrored-preparation release.

Artifacts, parity tests and activation receipt are under
`data/rail_deployment/approved_mirror_20260928`.
