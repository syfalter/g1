# Offline trajectory optimizer V3

## Scope

The offline editor has a separate **优化对比** tab. It supports selected single
D-ring, landing-gear and rail recordings, D-ring left/right repetitions, and
landing-gear round trips. Existing selections, production caches, gateway
settings and Jetson deployment are not replaced by this tab.

Start: `bash /home/syf/g1_upper_body_ws/scripts/task_editor_offline.sh start`

Stop: `bash /home/syf/g1_upper_body_ws/scripts/task_editor_offline.sh stop`

## Comparison

1. Select a single recording in its original editor tab, then choose that slot
   in **优化对比**. For D-ring combinations, select the two explicit trimmed
   versions in this tab. The current single-recording selections are unchanged.
2. Start with jerk **5**, contact tolerance **1 mm**, transit tolerance **5 mm**,
   and speed **1.0**. Generate, then switch between the original path at jerk 5,
   original path at the selected jerk, and fitted-path best candidate at that jerk.
3. Inspect fingertip speed, joint speed, acceleration and jerk plots. Play uses
   the same one-shot offline RViz trajectory playback as composition previews;
   the slider seeks independently. Plot samples are not continuous extrema tests.
4. Try jerk 10 or 20 only as an offline sensitivity experiment. Compare original
   and fitted paths **at the same jerk** to separate geometry improvements from
   relaxing the policy. These values are not a hardware release authorization.
5. Speed 0.5 doubles arm motion duration. Mandatory hand waits are not shortened
   or stretched by the arm speed control. Repetition and speed changes reuse
   optimized units; stitching and plot generation still take time.
6. The save icon exports an offline composition candidate under
   `data/composition_candidates/`. It does not push, select, execute or authorize
   a trajectory on Jetson. All artifacts have `hardware_executable=false` and
   `physical_verified=false`.

For landing gear, the count means **complete round trips**. A differing final
rod position adds one final stroke. D-ring count means left-then-right pairs;
only the final right stroke uses the verified full parent's original exit tail.
Fitting can adjust that route within the specified operation tolerance.

## Geometry And Timing

- Local C2 fits operate in bounded sections of the source path. They preserve
  endpoint position, velocity and acceleration, including moving internal cuts.
- Local quintic replacements remove eligible low-speed valleys. Continuous
  Bernstein bounds certify deviation under a monotone path correspondence.
- Local fits consume 65% of the deviation budget, subsequent replacements 35%.
  Windows crossing a role boundary use the tighter budget. Previously available
  global smoothing and seam/valley strategies also compete where applicable.
- Retiming and continuous polynomial V/A/J and joint-position checks follow
  geometry edits. The shortest verified proposal competes against the unchanged
  path at the same limits; slower proposals remain in reports but are not selected.
- Defaults map 1 mm to 3 mrad joint and 0.5 degree orientation bounds, and 5 mm
  to 15 mrad and 2.5 degrees. TCP bounds use conservative kinematic chain radii,
  not just sampled point distances. Bounds are against the **fitted source**, not
  a guarantee against raw sensor error or object contact/collision clearance.
- Recorded D-ring strokes and unknown contact phases default to operation.
  Known gear/rail approach and retreat slots use transit. An optional explicit
  source-time transit interval is available for a single recording; annotate it
  only when it is actually free of contact.
- Rail previews in this tab are **arm joint-path experiments**. They do not apply
  the production rail contact TCP-speed policy or constitute a full rail task.
  Synchronized finger recordings are rejected, not silently discarded. Existing
  landing-gear hand waits and grip/release boundaries remain in combinations.
- Minimal units are cached by source trajectory, spatial roles, seams, policy,
  model, planning settings and optimizer code. Repeat count is not a cache key.
  Search is local and heuristic, not a proof of global time optimality or
  suitability for the physical robot.

## Jerk Provenance Audit

The repository's **5 rad/s^3 is a software policy**. The audited paths are:

- `scripts/task_trajectory.py`, `task_jerk_limits`: returns 5.0 per joint.
- `src/g1_upper_body_moveit_config/config/joint_limits.yaml`: arm `max_jerk: 5.0`.
- `src/g1_upper_body_moveit_config/src/single_arm_planner_server.cpp`: populates
  `jerk[name] = 5.0` before smoothing.
- `src/g1_unitree_control/src/arm_sdk_hardware_gateway.cpp`: has output,
  tracking and motion protections, but this audit did not establish that the
  planning value 5 derives from a motor/drive jerk specification.

No manufacturer specification or calibration evidence establishing 5 as the
motor's physical jerk limit was found in these paths. Neither 10 nor 20 is an
established motor rating. Absence of a gateway jerk check does not establish
that the firmware or physical system is unrestricted. The application velocity
and acceleration caps remain unchanged; this experiment does not establish
the maximum available actuator performance.

The optional `jerk_limits` argument in `task_fast_timing.local_candidate` is
explicit and local to the new offline caller. Existing callers retain 5.0.
No production YAML, C++ planning constant or hardware gateway protection was
relaxed. The new tab is available only in the disconnected offline editor.

## Reproducible Evidence

`scripts/check_task_offline_optimizer.py` generates offline candidate artifacts
and evidence with per-policy times and continuous derivative peaks. It needs
the generated offline URDF and the usual sourced ROS environment. Pass
`--workflow landing_gear` or `--workflow dring_demo` for combinations, and
`--slot all` for selected standalone recordings. `--slot right_4 --version 9`
selects that version only in memory. Rail requires an explicit `--source` path.

Artifacts use `*-v3-evidence.json` and matching `*-original.json`,
`*-baseline.json`, `*-candidate.json`. Their `unitEvidence` contains geometry
bounds, rejected proposals, selected method, continuous peaks and source hashes.
Generated artifact names identify slot/policy; source hashes, not filenames,
identify the precise version used in a comparison.

## Offline Results: 2026-09-27

Same application velocity/acceleration caps and jerk=5, speed=1.0:

| Case | Original path retimed (s) | Best bounded path (s) |
| --- | ---: | ---: |
| Full right stroke, right_4 version 9 | 39.574 | 38.113 |
| Gear recorded above_to_lower segment | 7.786 | 6.659 |
| Gear recorded upper_to_above segment | 11.347 | 9.335 |
| Gear default combination | 65.627 | 62.872 |
| D-ring one left/right pair with entry/exit | 78.851 | 77.197 |
| Rail approach_B joint-path experiment | 29.218 | 25.981 |

The complete gear entry_lower unit includes connectors and retains the older
valley strategy at approximately 8.862 s, essentially the prior best result.
Do not confuse that complete unit with the 6.659 s recorded segment alone.
The default gear combination is one complete round trip starting lower, a final
stroke to upper, and exit, including the two hand waits.

For the **same complete right version 9**, experimental jerk10 gives original
path 33.788 s versus fitted candidate 31.498 s; jerk20 gives 26.174 s versus
25.467 s. These are software-policy experiments, not a motor capability claim.

Ten repetitions assembled from cached units in approximately 0.56 s for D-ring
and 0.34 s for gear in this run. UI generation also performs loading, fitting
source representations, serialization and plotting; the measured warm gear
comparison took about 6 s, not just the stitching time.

After restarting the isolated offline service, a ten-second playback observation
received 300 joint-state messages, 301 transform messages, and 299 changing joint
frames; maximum observed joint-message gap was 34.18 ms. This measures the ROS
preview stream, **not RViz GPU rendering FPS**. No hardware commands were sent.
