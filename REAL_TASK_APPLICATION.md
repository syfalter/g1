# Real task application commissioning

## Rail Capture Alignment (2026-09-20)

The recorded-path default is now `cycle`, a three-recording cycle:

If the global quintic spline exceeds the recorded-path fitting bounds, rail
replay automatically retries with the existing stop-to-stop quintic waypoint
interpolator through the selected measured positions. Each leg is checked using
its own progress against the raw recording; the 0.5 mm / 0.5 degree / 0.01 rad
bounds are not relaxed. This fallback slows/stops at retained waypoints, retains
the original endpoints and undergoes the same joint, speed, jerk and waist
checks. The plan reports the fallback count and caches successful results. It is
separate from automatic connections between stages. A fallback that still fails
validation does not execute.

- Push: current posture -> right above-table -> push hand -> `approach_A` ->
  `push` -> `retreat_B` -> right above-table -> open hand.
- Pull: current posture -> right above-table -> open hand -> `approach_B` ->
  pull hand at the final frame of `approach_B` -> alignment to the recorded pull start -> `pull` -> open hand at A
  -> `retreat_A` -> right above-table (hand stays open).

Recorded rail curves use the shared `scale_arm_segments` retimer at 0.7
velocity/acceleration scales. `task_planning.yaml` section `rail_replay` selects
`rail_replay_multiplier` (0.1 through 10, live file requests 2), or
`rail_replay_fastest: true` for the fastest locally verified result. This is
not a global optimum or a mechanical-limit claim. Faster requests use the
path-preserving local policy; legacy 1x snapshots retain the recorded policy.
The geometry-validated curve is normalized to captured motion duration before
local limit enforcement. Hand switching and connection timing are unchanged.
`rail_trim_idle_ends` removes leading/trailing near-stationary time padding and
tries omitting redundant knots within 0.3 mm, 0.3 degrees and 0.003 rad of the
respective endpoint. Original data and exact first/last endpoint positions are
preserved. Omitted knots are restored if they prevent the original fit checks
from passing. `rail_replay_smooth` tries adaptive-knot continuous quintics before
stop-to-stop fallback, retaining the 0.5 mm / 0.5 degree / 0.01 rad checks against
the raw path. All options enter the cache identity; changed settings require
replanning. Existing plans keep frozen settings. New configuration fields require
reopening an older panel; this does not tune or restart the hardware controller.
The recorded rail panel exposes a 0.1x-10x speed input, a fastest-verified checkbox
and a leading/trailing idle-trim checkbox, initialized from that file. The trim
result shows removed original-recording padding and fit-knot counts, with
per-stage details in the tooltip; padding is not a promised execution-time saving.
Panel choices override only this plan's rail multiplier/fastest/trim fields,
not the YAML, original recording or controller limits. Changing any option
invalidates the old rail plan and cancels pending cache execution. Cache reuse
includes these choices. Source-file and controller-limit changes still invalidate
the frozen configuration; endpoint planning does not use these controls.
The extra 5 mm/s cap is removed from recorded replay and its small joins.
Curvature/jerk limits or stop-to-stop waypoint fallback can still lengthen a
recorded segment. Already-matching cycle hand targets are skipped.

Finger health and fresh measured starts remain required, but recorded-path
planning no longer requires the current hand to match a saved open/grip target.
The panel fixes recorded playback to complete three-stage cycles. Older
hold/release modes remain available with the Cartesian endpoint planner.
Cycle connections are bounded joint moves with operator clearance, not collision
avoidance. Traces can have different waist angles; the endpoint planner's fixed
waist constraint is not used for these recorded cycles. Controller/model angle
limits, interpolation checks, velocity/torque faults and STOP are not removed.

Validated recorded curves persist under `data/task_panel/rail_replay_cache`.
After a successful plan, the reuse-and-execute buttons reuse those curves across
panel restarts, freshly constructing only current-position connections and finger
transitions. The cache key covers the selected recordings, model, motion settings
and implementation; the successful-plan key additionally covers hand targets,
above-table target and mode. Changing unrelated opposite-direction recordings
does not invalidate the selected direction's cache. Missing, changed or corrupt
cache entries stop a reuse request without motion or implicit recomputation.
Recorded planning and cache reuse do not require manual rail/grasp state or
preplanning checkbox confirmation. Execution clearance, controller feedback and
STOP checks remain; cached data never restores execution authorization.
Recorded planning reads current hand positions once for model and finger
transitions, without the two-sample finger-stationarity check. Fresh feedback
and execution checks remain; legacy endpoint planning is unchanged.
Recorded rail geometry/timing runs in a separate process without ROS nodes or
command clients. Stage progress is reported to the panel; cancellation or a
120-second deadline terminates and reaps the worker. Stop-to-stop fallback
quintics retain only their exact leg boundaries before timing optimization,
not the redundant 20 subdivisions per leg. Fit and final motion bounds remain.
Reuse execution is queued for one matching generation
only, and STOP, selection changes or a failed preparation cancel it.

At startup the rail panel restores the most recently modified session
`rail_autosave/config.json`, including recordings and taught hand targets.
Loading is read-only and does not restore mechanical state or execution authority.
A latest save with no trajectories remains empty (an intentional clear is not
undone). An invalid/incompatible latest save is reported with its path, without
silently falling back to older recordings. Manual loading remains available.

Rail teaching uses the existing dedicated measured-joint feedback executor and
the same read-only, full-rate LowState recorder as the D-ring demonstration.
Every received distinct tick is archived, including joint velocity, estimated
torque, motor status and DDS timestamps. The bundle retains measured positions
and timing; telemetry stays in the recording directory. Finish/save immediately
sends the click's monotonic cutoff, validates the preceding stationary window,
and excludes later samples from the saved playback bundle.

Accepted rail arm actions now rely on the gateway's LowState watchdog and action
result, as demonstrated by D-ring playback. STOP/epoch/configuration checks,
fresh stationary admission and segment-boundary checks remain. No hardware
watchdog, Cartesian speed limit or rail
path-fitting bound was changed. Full-rate capture is not full-rate command output:
rail replay keeps its existing geometric selection and bounded interpolation.

Rail and D-ring demonstration guards no longer reuse the console handoff's
single-sample 0.05 rad/s stationary-velocity gate. Measured position windows at
admission/boundaries, held-position checks during finger transitions, controller
health, feedback freshness and STOP remain. Console handoff, screen workflows,
landing-gear teaching and hardware velocity/fault protection are unchanged.

The rail hand table now separates open, push grip (`grip`) and pull grip
(`pull_grip`). Record the actual pull posture explicitly; no new finger target is
invented. Both hold and release workflows switch to the saved pull grip before
pulling. Old two-posture
bundles remain usable; older hand-bound recordings retain their stored evidence
checks. New captures now use `hand_policy=not_recorded`: all six stages record
only the right arm and waist, without hand RPCs, hand preset requirements or a
required preparation/previous-stage pose. Hand targets are needed only for task
replay, not capture or standalone arm moves. Changing a hand preset does not
invalidate a new arm-only recording.
Recording or applying a hand posture still invalidates execution authorization
and, for legacy endpoint planning only, requires reconfirming the actual rail state.

Recorded endpoints are derived directly from the final captured frame, not from
an independently edited point or the opposite direction's trajectory:

| Endpoint | Source Stage's Last Frame |
| --- | --- |
| Push A | `approach_A` |
| Push B | `push` |
| Pull B | `approach_B` |
| Pull A | `pull` |

The four endpoints update with replacement/import and disappear with deletion
of their source recording. The old endpoint-planning A/B remain separate.
Buttons can move from the current measured posture to right above-table, any
of these four points, the selected stage's existing first frame, or its preceding
endpoint. Push uses Push A, pull uses Pull B, and retreats use their corresponding
direction's final point. Approach stages use above-table as the optional start.
These are explicitly confirmed right-arm/waist-only moves in ARMED_HOLD; they
never read, command or switch fingers, and never infer the rail's mechanical state.
The UI executes only the matching completed plan generation; STOP cancels it.

Recorded replay requires only the stages used by the selected operation. It no
longer enforces equality of the two directions' A/B or a shared ready/start/end
pose. Current-to-start, inter-stage and final above-table connections use the
existing bounded joint transition planner, appear in RViz, and require operator
clearance (no collision checking). Recorded-path fitting bounds remain;
all rail arm/waist planning uses 0.7 velocity and 0.7 acceleration scaling with
the existing joint limits. This includes above-table, endpoint/start moves,
connections, recorded replay, legacy Cartesian plans and hold recovery. The
additional 5 mm/s cap is retained only by the legacy Cartesian endpoint planner.
Jerk limits and fitting bounds remain unchanged; 0.7 is a planning scale, not
an absolute end-effector speed. Executed finger ramps retain their original timing.

Right-arm base control now copies the configured left-arm values. In shoulder
pitch/roll/yaw, elbow, wrist roll/pitch/yaw order, Kp is
`[75, 70, 75, 75, 40, 55, 55]` and Kd is
`[4.2, 4, 4.2, 4.2, 3.5, 3, 3]`. Both arms use the same configured integral
gains and limits. Both right-arm and passive-arm integral switches are enabled:
the launcher fixes `active_arm=left`, so the right arm needs the passive switch.

Recorded rail segments use the same replay profile as recorded D-ring segments:
Kp `[650, 550, 550, 550, 220, 300, 300]`, proximal Kd `6`, base wrist Kd,
and waist Kp/Kd `300/5`. Standalone moves and connections use normal gains;
stationary boundaries switch profiles and completion/STOP restores normal.
Recorded replay does not apply the legacy rotate/withdraw speed factors.

Rail endpoint and subsequent segment-boundary checks now use the same frozen
`execution.endpoint_tolerance_deg` as D-ring playback (currently 1.432394 degrees,
0.025 rad), without the former rail-only 0.5-degree cap. This includes standalone
above-table and A/B moves. The configured 5-second settling deadline, initial
0.025-rad start check, feedback freshness and STOP guards are unchanged. The
separate waist-recovery operation retains its 0.5-degree cap.

Verification uses offline FK, fake hand/action transport, in-memory LowState and
offscreen Qt. It does not certify physical contact or execute the robot. Restart
both the controller and task panel through the normal safe shutdown/start procedure
to load the base gains and integral settings, then regenerate plans. Restarting
only the panel is insufficient. No running robot process is restarted by this change.

## Recorded D-Ring Demonstration (2026-09-18)

Recorded playback now reuses the previous `withdraw` stage gains when
`stage_control_enabled` is true: selected-arm shoulder pitch/roll/yaw and elbow
Kp=550, Kd=6; waist yaw Kp=120, Kd=5 in the current configuration. Wrists and the
other arm keep their base gains. Preparation, start alignment and return use
`normal`; the final stage also restores `normal`. Gains are changed through the
existing stationary-boundary atomic stage service, never mid-trajectory. STOP
retains the gateway's normal-profile restoration. Configuration is frozen in each
new replay plan; the panel shows its replay gains. The recorded trajectory duration
is not multiplied by the legacy rotate/withdraw velocity factors.

Execution watchdog correction: after an arm goal has been accepted, demonstration
execution relies on the gateway's own LowState watchdog and action result, without
an additional GUI diagnostic freshness/service-health/watchdog-configuration gate.
Delayed diagnostics or a 150 ms GUI JointState gap do not independently abort an
accepted action. STOP/epoch/configuration guards remain.
Fresh measured state is still required before motion and at every segment boundary;
endpoint tolerances are unchanged. Independent feedback reception alone did not
eliminate GUI gaps in the 19:39 session, so it was not a complete fix by itself.
The initial added 500 ms diagnostic deadline was erroneous: the gateway's 100 ms
timer deliberately throttles unchanged diagnostics to at least 500 ms between
publications (approximately 500-600 ms with scheduling). The demonstration-specific
500 ms check and the other added in-flight diagnostic polling gates have been
removed, not the gateway's separate 100 ms LowState watchdog. Existing preflight
and segment-boundary hold/fresh-position checks remain unchanged.

The default replay mode is now **Full cycle** for the selected arm and recording:
current pose -> that arm's existing above-table preparation -> taught ready point
-> recorded start (if different) -> recorded trajectory -> above-table preparation.
Preparation/return targets reuse `node.prepared`, the same existing preparation
poses shown elsewhere in the panel. They do not overwrite the taught ready point.
No collision checker or obstacle-avoidance planner is called for any cycle stage.
Joint, speed, acceleration, jerk, start-state and STOP checks remain. Record the
release/extraction before the recording ends if necessary; the automatic return is
a joint-space transition, not a contact-aware disengagement maneuver.

**Move to taught ready** and **Return to above-table preparation** each ask for
explicit clearance, then generate and execute that one movement automatically.
There is no need to click Execute again for these two buttons. STOP or a changed
generation cancels a queued move. Full cycles still require reviewing RViz and
the normal Execute action. Single-recording playback remains an alternate mode.

The arm selector switches persistent bundles without clearing the other arm:
`data/task_panel/dring_demonstration_left.json` and `_right.json`. The original
`dring_demonstration.json` remains the active bundle and is migrated on first
switch/save. Import/export applies to the selected bundle. A right-arm recording
is never synthesized by mirroring a left-arm recording; teach it separately.

Latest correction: the panel now records and replays **arm/waist only**. Imported
finger evidence remains in the original bundle but is not executed. Use the
separate **Index and middle extended** button, while holding, to extend the selected
hand's index and middle fingers together and fold the other fingers. It requires an explicit confirmation;
it does not move the arm. The older optional finger-timeline implementation is
retained internally, not exposed in this demonstration workflow.

Finish/save now sends the capture process a monotonic cutoff immediately on the
click, before any hand RPC or save work. Saved samples exclude later arrivals;
the required 0.4-second stationary window is checked before the cutoff, not added
after it. The panel immediately reports capture ended while serialization finishes.

Measured joints now have a dedicated read-only ROS node and single-threaded
executor, separate from action/service callbacks. The 150 ms freshness limit is
unchanged. In a six-second read-only test with the main probe executor deliberately
blocked for 200 ms periodically, the independent feed had a 9.11 ms maximum gap
and 3.64 ms maximum message age. This verifies callback isolation, not an actual
robot execution. Failure reports now include callback gap, source age and lock wait.

RViz uses recorded trajectory time (`1x`), interrupts an old animation when a new
one arrives and plays once. **Replay RViz** republishes the generated preview
without fitting or commanding the robot, including when RViz subscribed after
generation (its subscriber is volatile). Reopen RViz to load the revised display
settings; changing this file does not reset an already-running or paused display.

The new default demonstration tab records measured motion, without RGB-D or IK.
Left/right sweep are two recordings on the selected execution arm, not two arms.
Existing vision, history and rail workflows remain available.

1. Select the execution arm (use New when changing arms). Enter arm or arm+waist
   admittance explicitly, move to the ready pose, stop and record the ready pose.
2. Select left sweep, start recording, wait for the recording status, then drag
   the arm through the desired motion. Stop for at least 0.4 seconds and finish.
   Return to the ready pose and record right sweep in the same way.
3. Set the desired index-and-middle posture using the separate button before entering
   admittance. Do not force powered fingers manually. Arm feedback is captured by
   a read-only LowState process; the current demo workflow does not record fingers.
4. Leave admittance and hold. Select a recorded segment and multiplier, confirm
   the fixed workcell and path clearance, then generate playback / RViz. Review
   the complete preview, confirm clearance in the existing execution controls
   and press Execute. Preview alone never sends robot commands.

Both RViz and hardware use the same current-to-recorded-start transition and
recorded trajectory. The transition has **no collision checking**, and its speed
is independent of the recording multiplier. Joint, velocity, acceleration, jerk,
fresh-feedback and STOP guards remain active. Playback is not obstacle avoidance;
operator clearance applies to the complete transition and recording. If the robot
moves after generation, regenerate from its new position before Execute.

Recorded timing (including pauses) is retained, with up to 0.005 rad joint-space
smoothing deviation. This is unrelated to the deferred Cartesian 5 mm test.
Arm playback uses uniform speed scaling capped by the existing motion limits.
The panel reports requested, applied and maximum multipliers. Current hand posture
is retained during replay; recorded fingers do not restrict the requested speed.

Single-recording mode only replays recorded motion. Full-cycle mode adds the
preparation, taught-ready and final return movements described above.
The active bundle autosaves to `data/task_panel/dring_demonstration.json` and can
be imported/exported. Raw captures and dated snapshots remain in the task session.
Cancellation or unsuccessful capture preserves the previous valid recording.
Re-recording the ready pose clears the active two slots, with old raw files retained.

Reopen the panel after active motion ends to load this code; no controller restart
or automatic hardware movement is performed by this update. Hardware reproduction
must still be commissioned on the actual fixed workcell.

## Online Continuity And Speed Capacity (2026-09-18)

New complete online D-ring plans use a 0.04 rad maximum adjacent IK joint step
(previously 0.12 rad), a previous-slope predictor and a slope-change ranking term.
Oversized steps trigger local Cartesian subdivision, bounded at six levels and
240 additional points per stage, followed by the existing branch repair. No
intermediate stops are inserted. Geometry/orientation and final interpolated-path
validation remain unchanged; this does not enable the deferred 5 mm tolerance test.
The policy is recorded in solver/search diagnostics and frozen with new archives.
Existing archived trajectories are not rewritten.

Guarded speed changes now use **independent stage limits and continuous local
timing**. A SciPy SLSQP solve varies the rate along the original interpolated
path. Adjacent rates are joined continuously with zero rate derivative at their
boundaries, preserving joint velocity/acceleration continuity and existing stops.
Quintic gateway resampling has a whole-interval Bernstein error certificate:
target joint-path difference 1e-8 rad, total including timestamp rounding at most
1e-7 rad. Exact polynomial velocity/acceleration/jerk peaks and the existing waist
enclosure are checked after resampling, with the existing 0.5% timing margin.
Unverified or slower proposals fall back to that stage's verified uniform timing.

Motion setup shows the fastest verified request setting, arm-duration equivalent
multiplier, and total trajectory duration. The request setting is the largest
individual stage multiplier needed to reach all verified stage minima; stages
already at their limits do not throttle other stages. These are bounded numerical
optimization results, not proven global optima or hardware maxima. A smaller
requested multiplier retains the optimized local profile but increases each stage
duration to respect the requested stage-duration ratio. Hand stages retain their
duration; endpoint waits and feedback latency are not included. Repeated retiming
restores the exact stored baseline, never divides a locally timed path by an
aggregate multiplier. The same function is used for live plans, saved execution,
and guarded RViz, including partial-range selection.

The 2026-09-18 16:54 session's `plan_0013`, retimed offline at requested 24x with
the same 0.7 velocity/acceleration settings, changed from 189.753 to 58.539 s:
left rotation 124.366 to 23.397 s; return 22.022 to 10.641 s. This is a trajectory
timing result, not a completed hardware run. The immutable original was retained.

Use the fast-forward icon beside **Apply to planned path** to fill the maximum,
then **Apply to planned path** to retime without another IK solve. Review RViz and
use the existing Execute confirmations. Selecting the maximum never commands
motion. New acceleration/jerk values are not introduced. A running panel must be
reopened through the existing guarded attachment workflow after motion is complete;
this change does not restart a controller or modify an active execution.

## Saved-Start Alignment: Operator Clearance (2026-09-18)

At the operator's explicit request, new saved-start alignment prefixes use the
existing quintic joint-line generator without software environment/self-collision
checks or obstacle-avoidance planning. This applies only to the added alignment,
not the archived stages. Prefixes retain joint margins, velocity/acceleration/jerk
bounds, fresh start feedback, STOP handling and per-segment endpoint checks.
The requested archive playback multiplier does not accelerate the prefix.
Reports and the panel mark alignment collision as **NOT CHECKED**; the operator
must review the entire swept path and confirm clearance before Execute.
Existing running panels do not hot-reload this change. Reload the selected range
in the updated panel; an old alignment authorization is not reusable.

## RGB-D Planning, History And Custom RViz Speed (2026-09-18)

The existing task console now includes the updated workflow; the normal
`scripts/start_task_panel.sh` launcher still selects it. This delivery does not
restart a running controller or console automatically. Follow the existing
guarded panel attachment/startup procedure; do not restart an armed controller
merely to load UI changes.

1. In **Plan / session**, select the arm/direction and plan from a new measured
   RGB-D capture. Complete D-ring planning uses bounded IK, OMPL return routing
   and diverse preinsert candidates, with unchanged geometry/collision gates.
2. Successful full plans are automatically saved to
   `data/offline_trajectory_library/live/`. The archive contains the validated
   baseline trajectory, RGB-D, capture state, model/meshes and integrity evidence.
   Publication is atomic and checked by reloading. History refreshes automatically.
3. In **History / RViz**, select a library entry or open an archive directory.
   The play icon opens an isolated offline RViz. **Current RGB-D** requests a new
   capture; **Archived RGB-D** uses the saved scene without requiring a camera.
   The current overlay requires matching model/calibration and uses capture-time
   pelvis coordinates. It does not transform the old path or certify current-scene
   collision safety. Scene drift is recorded in `scene_overlay`.
4. **Requested speed (x)** accepts a custom positive finite multiplier (e.g. 0.5, 4,
   24, 73.125). Existing live paths use **Apply to planned path**, without another
   IK search. The separate replay window also has a trajectory-speed field and
   apply icon/Enter action. Changes restart the preview from its baseline, not
   from an already multiplied copy. Arm stages have independent local timing;
   hand opening/closing retains its existing timing.
5. Panel RViz and hardware range loading now use the same guarded time scaling,
   with a snapshot of current controller-bounded motion parameters. Velocity and
   acceleration sliders constrain limits; they do not multiply the requested
   speed a second time. Existing acceleration/jerk caps remain in force.
   Reports distinguish requested/applied speed and duration. Nonfinite
   input, timestamp collapse/overflow or unrepresentable curve accuracy is
   rejected. In this mode the extra viewing-speed dropdown is locked at 1x.
   Selected hardware stages inherit the full archive's identical stage timing.
   The history table's **Original duration** remains the immutable baseline.
6. **Remember / reuse verified collision checks** is opt-in. Check it before
   validating to save a successful result, or after a successful validation to
   remember that result without rerunning it. On subsequent loads, check it to
   reuse certificates from `data/trajectory_collision_checks/`, across restarts.
   A certificate covers only the exact checked stages and trajectory timing,
   archived scene/model/meshes and checker implementation. Changes invalidate it.
   No certificate is created on failure; a missing/corrupt certificate triggers
   normal validation. Uncached dense collision checks use up to four isolated
   processes with bounded batches, keeping the existing sample resolution.
   Cache hits skip scoped collision computation only: motion limits, archive
   integrity, actual start, current feedback and explicit scene/contact
   confirmations remain mandatory. This is not a full-environment collision
   certificate or an execution authorization.

Legacy unrestricted previews remain `preview_only_timing` and cannot be
authorized for hardware execution. Standalone experimental gallery CLI preview
mode remains available, but the panel launches guarded mode. A separately opened
gallery does not change an already validated main-panel plan: use matching
settings and reload/review the hardware range after edits.
No ARM, controller restart, hand command or real trajectory execution is part
of this delivery. A fresh camera round trip still requires the connected robot
session and was not exercised during offline verification. Position tolerance
remains 3 mm; the proposed 5 mm experiment was deferred.

Historical unrestricted-preview fixture verification: 24x left/right sweeps last
29.893/26.687 seconds (both use the left arm), including withdrawal and return.
These are not guarded hardware timings. Earlier evidence, panel images and report:
`data/offline_archive_previews/20260918_panel_delivery/`.

## Panel Opens Without Healthy Hold (2026-09-11 12:44)

Fixed `start_task_panel.sh` rejecting UI startup because the retained controller
was `LATCHED_SILENT`, unarmed, with no command publisher. GUI-only attachment now
uses `attachment_mode=panel`: it restores local home/ready records and displays
the current diagnostic phase/STOP epoch without requiring healthy hold or fresh
stationary joint feedback. It does not ARM, reset, resume, authorize or execute.
Execution and the legacy guarded hold/stopped migration checks are unchanged.

A real startup also exposed a roughly two-second backwards wall-clock adjustment:
the just-created manifest was treated as future-dated and rejected as expired.
Panel-only records are not motion handoffs and now omit this wall-clock TTL;
controller process identity, record shape and single-console locking still apply.
Legacy hold/stopped manifests retain their 120-second TTL. Startup acknowledgement
checks the exact restored manifest path, not a whitelist of healthy phase names.

Successful live GUI PID41718, unit `g1-task-console-20260911-124450-184019.service`,
session `data/task_sessions/20260911_124450_953314`. Gateway7888 and RViz7896 remain.
Final GUI feedback age 0.001933s, idle, no plan; gateway LATCHED_SILENT, armed=false,
command_publisher_exists=false, epoch4. Trajectory counts remain4/4/0 and command
count146827. The first failed GUI startup's existing systemd exit hook issued STOP
(epoch3->4); the successful startup did not change these counters or protection.
284 task tests passed before the clock-specific fix; 56 focused panel/handoff/Qt
tests passed after it, including unhealthy/unarmed attachment, absent joint state,
backwards clock, preserved STOP, replaced-controller rejection and no control RPCs.

## Planning Configuration and Panel Close (2026-09-11)

Edit `config/task_planning.yaml`: every parameter has an inline Chinese comment,
units and supported range. Each plan worker reads and freezes one configuration
snapshot before capture. Successful `plan.json` records the source path, SHA256
and effective values. Changes apply on the next plan, never midway through an
existing plan or execution. The straight offline replay uses the same file;
explicit offline search CLI arguments override their corresponding file values.

The file includes radial/height offsets, orientation tilt/twist/neighbor step,
arc yaw gain, initial tilt search, angular margin, arc/line density, initial search
budget and batches, complete candidate count, fast-path selection, and local
repair count/backtracking depth. Hardware gains/watchdogs and collision rules
are not changed by this file. Defaults retain the behavior documented below.
Set `orientation.arc_yaw_gain: 0` for a fixed insertion nominal reference along
the arc; the shipped value remains 0.15. Tolerances are relative to the selected
nominal/scheduled reference, not an independently enforced bound from the actual
measured orientation at the first insertion point.

The operator reported powering off the robot. GUI PID7881 was closed and verified
absent; RViz7896, controller7888 and supervisor7777 were retained. No successful
DDS STOP acknowledgement or current physical support is claimed. No ARM or motion
was sent. Local attachment context is `data/task_panel/retained_session.json`.
`bash scripts/start_task_panel.sh` reopens only the existing controller session's
GUI, with fresh diagnostics and a newly generated short-lived attachment record;
it does not restart a controller, ARM, resume or execute. It requires the retained
supported controller and network, so it is NOT a full reboot/recovery script.
For a new controller session the original entry remains
`scripts/run_real_task_application.sh eth0 --hardware --dring-contact-test`;
its existing exclusive-controller lock prevents simultaneous owners.

347 offline tests passed (281 task, 26 vision, 40 workbench), including seven new
config/default/reload/type/range/custom-sampling/fixed-reference/search/repair
tests. Panel close was exercised locally; reopening on hardware was not tested
because the operator powered the robot off and requested that the GUI stay closed.

## Target and Arc Reference Fix: Offline Only (2026-09-11)

The latest source patch is NOT loaded into the existing live console. It changes
the local target offset from +15 to +10 mm radially (5 mm inward relative to the
current target), and raises the operation plane 10 mm. Measured shaft center and
base collision geometry are unchanged. Online capture and offline straight replay
share these settings; archived targets are rebuilt once from original measurements.
Existing captures/plans are not rewritten, and old plans do not acquire new targets.

The arc already uses 120 new IK waypoints per 120-degree leg with local orientation
search, joint continuity and retimed interpolation checks. Fixed insertion and
withdrawal references are now separated from the arc's slowly rotating reference;
the old code incorrectly evaluated arc yaw at slanted insertion points, rejecting
three recorded initial poses at 5.60/6.32/5.93 degrees. Their actual tilt from the
selected insertion reference was less than 0.001 degree. Arc height validation now
uses the operation plane, not the physical shaft-top plane.

340 offline tests passed (274 task, 26 vision, 40 workbench), plus Python and shell
syntax checks. This includes recorded-pose rejection reproduction, not full-path
IK replay or measured planning success-rate improvement. No runtime process or
hardware command was touched, and no pause, confirmation or execution step added.
Details: `logs/dring_target_update_20260911/REPORT.md`.

## Live Session Restored (2026-09-11)

The feedback/execution fixes below are now loaded by the fresh live console
PID 7881, unit `g1-real-task-control-20260911.service`. Gateway 7888, validator
7892, planner 7894 and RViz 7896 are running. The launcher now forwards the explicit
`--dring-contact-test` option. Startup itself did not ARM or execute; the operator
subsequently completed one preparation action and started D-ring planning.

Hand and vision DDS passed live read-only checks, SSH artifact access was restored
through an operator-entered login, and the console has fresh source-stamped joint
feedback. Reports and exact source hashes: `logs/dds_restore_20260911/final_live.json`
and the adjacent hand/vision/LowState reports. No controller parameter was changed:
the measured gains are 60/4.5/20 with a 1.2 Nm integral limit, not the previous
session's temporary 1.7 Nm. This supersedes the deployment status in the historical
section below, not its limits on physical test evidence.

## Feedback and Execution Fixes: Offline Only (2026-09-10)

This update is on disk only. No live console/controller was restarted, no ROS
test node was initialized and no hardware command was sent. Historical deployment
entries below do not mean that a running console has loaded these changes.
Verification: `logs/feedback_execution_fix_20260910/REPORT.md`.

The intended workflow remains one explicit execution of the complete ten-stage
plan. There is no new pause/resume button, confirmation, or hand pause-and-restart
state machine. The hand pause/resume implementation explored during this update
was removed following the operator's clarification. Existing stationary trajectory
endpoints and hand-settling requirements remain; this is not velocity blending
across contact stages.

- Authorization file parsing/fingerprints and Qt rendering no longer hold the
  joint-feedback lock. Failed preflight preserves the original preview; STOP
  cancels the pending commit. Motion settings are disabled during preflight.
- Joint callbacks use separate callback groups, a depth-one subscription, source
  timestamps and receipt times. Replayed/delayed data cannot refresh the measured
  state. Identical positions with advancing timestamps remain valid observations.
- Arm starts/endpoints reuse a bounded automatic feedback/settling check. Healthy
  already-settled boundaries return immediately. A brief gap no longer falls
  directly into task STOP, and a successful action can await its lagging hold
  diagnostic. Real start displacement, faults, STOP and changed epochs still fail.
- During a hand ramp, the arm gateway owns LowState/hold monitoring; the console
  checks fresh healthy hold diagnostics, STOP/epoch and observed arm displacement.
  The GUI's 150 ms arm-sample timer is no longer a second watchdog on that hand
  ramp. Hand DDS readback, advancing timestamps, faults and settling remain checked.
  A GUI-only short gap does not send hold or restart the ramp. Gateway diagnostics
  retain their existing 1.5 s freshness deadline; this is not equivalent to a
  150 ms end-to-end hand-stop guarantee. No controller/hand-service watchdog changed.
- Removed the redundant bilateral read immediately after a completed hand stage;
  the next stage already reads and checks both hands before output. Full completion
  now reports the complete D-ring task, not an instruction to review another range.
- Recovery hold acknowledgement reconciles the stopped flag independently of a
  simultaneous GUI feedback gap, without clearing a newer STOP. Hand STOP can
  interrupt the local pending reply stream; discarded targets are never replayed.
  Local adapter interruption is not proof of an acknowledged physical hand hold.
- Initial IK search offers up to two ranked diverse full-path candidates every
  32 trials, stops at complete success, and continues only on retryable candidate
  failure. The existing 220-attempt/30 s active initial-search budget remains;
  suspended full-path validation is accounted separately. A 0.1 degree planning
  margin avoids selecting poses on the final angular boundary; final 5/10 degree,
  2 mm, collision and trajectory-density checks are not relaxed.
- Execution logs now include progressive stage/RPC checkpoints and detailed failure
  evidence, rather than only the final stale message.

Offline regression: 272 task tests plus 40 workbench tests passed. Timed execution
simulations use the production Python executor and hand ramp with in-memory
action/DDS transports; they do not simulate robot dynamics. Normal ten-stage and
mid-hand GUI-gap cases need no added boundary sleeps. Deterministic IK tests prove
32-trial early exit and continued search on failure, not a measured real MoveIt
speedup. Physical tracking, network reliability and full D-ring contact success
still require a separately authorized live test after deployment.

## Explicit Full Live D-ring Tests (2026-09-10 21:18)

The operator explicitly requested full ten-stage hardware testing. Launching the
console with `--dring-contact-test` now permits the current, completely validated
live D-ring plan to enter the existing staged commissioning executor. The ordinary
preview remains `hardware_executable=false`; only explicit path/scene and contact
alignment confirmations create a separate in-memory `staged_test` authorization.
The button is `Execute full D-ring test (1-10)`. Without this launch option the
normal preview gate remains. Historical statements below about unconditional
blocking of fresh D-ring tests are superseded only for this explicit test mode.

Authorization checks the current ticket generation, gateway stop epoch, robot
model SHA, saved plan SHA, exact ten-stage labels/ownership/trajectory content,
rest boundaries and the existing scoped collision profile. It does not run IK
again or change the validated curve. Authorization is recorded beside the source
plan without changing its preview metadata. The same staged executor performs
eight arm actions and two DDS hand transitions, validates measured starts and
hand feedback per stage, waits for settling, and requests STOP on failure without
continuing later stages. This is a supervised commissioning test, not production
certification: the scene checks are still selected hand/base and selected
arm/body checks, not full environment/self-collision validation. Contact remains
position-controlled, not force-compliant. Physical alignment and STOP access
must be confirmed at the robot.

Feedback freshness for execution remains 150 ms. Feedback gaps, including gaps
over 1.5 s, do not destroy a completed frozen preview or saved test. Completion
retains the validated trajectory immediately; the UI blocks execution until fresh
feedback and 0.4 s of continuous stationary observations return, without replanning.
Measured movement, STOP and changed selection still invalidate review. Execution
rechecks current arm/waist start, hand DDS, controller and operator confirmations.
A historical gap alone is not proof of movement. External scene/base changes
still require recapture. Active motion and hand-transition protections are unchanged.

94 focused tests passed, including the complete mock ten-stage execution,
mid-task rejection/STOP, missing confirmations, changed model/source/trajectory,
feedback recovery, sustained loss and cancellation. No full real D-ring motion
was sent by this update. Deployment: console PID315291, unit
`g1-task-console-20260910-213508-612586.service`; gateway286890 and RViz286898
retained. The latest completed plan0005 is frozen and preselected in Saved
trajectory test as stages1-10 at
`data/task_commissioning/20260910_212255_left_full`; loading still requires the
existing scene/hand/contact confirmations and range validation, not a new IK solve.

## Live Prepare Batch-Validation Fix (2026-09-10)

The steep-straight planner's batch trajectory hook was accidentally selected for
prepare/recover trajectories merely because the subclass exposed the hook. Those
flows intentionally install only the measured MoveIt surface, not the task-only
local base/body checker, so post-retiming validation incorrectly raised
`SCENE_NOT_READY: Fresh scoped target required` after MoveIt had already planned.
`TaskPlanner.retime()` now enables that batch hook only for `ticket.kind == task`;
prepare/recover retain their original scalar MoveIt/FCL validation. A small-step
prepare regression test prevents the optimized batch path from being selected.

Task and workbench Python regression is `232/232` passing. The live console was
replaced through the guarded handoff without restarting gateway PID 6504 or RViz
PID 6512. Report: `data/console_handoffs/20260910_111849_874654/report.json`.
No motion trajectory was received, accepted or rejected during the replacement.

## Runtime Waist Tuning and Replay Performance (2026-09-09)

The command console now keeps a compact live status summary above two tabs:
`Plan / session` and `Saved trajectory test`. Session state, gateway phase,
LowState age, waist gains, tracking error and motor temperature remain visible
while switching workflows. Execute eligibility and its blocking reason stay
next to the fixed STOP/recovery controls. Planning exposes elapsed time and
sample-validation progress. Both 720x860 offscreen layouts pass text and window
bounds checks; reference renders are
`logs/task_application_gateway_test/console_widget_plan.png` and
`console_widget_saved.png`.

The gateway default waist gains are now Kp 40 and Kd 2. An atomic runtime update
callback accepts only Kp `[20,80]`, Kd `[0.5,5]`, with maximum one-step changes
of 20 and 1.5. Updates are rejected during a trajectory, release, velocity
recovery or task stop. While armed they additionally require `ARMED_HOLD`, fresh
LowState, waist speed at most 0.03 rad/s and hold error at most 0.015 rad. Current
values, revision, last accepted change and last rejection are published in the
gateway diagnostics. The console requires a fresh safe phase plus an explicit
stationary/unloaded confirmation before enabling Apply.

`scripts/check_waist_gain_runtime.py` exercises the actual installed gateway in
localhost-only domain 88 with hardware output disabled. Startup 40/2, atomic
success, invalid startup/range/type/step rejection and parameter/diagnostic synchronization pass;
the report is `logs/waist_gain_runtime_test/report.json`. No `/arm_sdk` publisher
is created. This verifies software guards only. The robot was powered off, so Kp
40/Kd 2 still requires a fresh-session, low-load 1-3 degree hardware tuning run
before any saved stage is retried.

The same archived `plan_0003` replay used two pre-change baselines of 74.49 s and
84.013838 s, mean 79.251919 s. Four stable final runs completed in
45.1356/42.8341/45.9020/45.5972 s, mean 44.8673 s. This is 1.766x throughput, or
76.6% faster under the requested throughput definition; wall time is 43.4% lower.
Every stable run preserved ten stages, 6633 IK calls, 75076 dense samples, stage
point counts, TCP extrema and zero hardware commands. Reports are under
`data/offline_task_tests/20260910_{001449_857719,001639_436968,001808_045492,003419_895213}/`.

The speedup comes from precompiled limits, shared and batched FK, conservative
bounds before exact FCL, and batched retimed TCP/line/arc checks. The 5 ms sample
density, 0.005 rad interpolation boundary, exact FCL narrow phase and all existing
joint/TCP thresholds are unchanged. Tests cover scalar/batch FK equivalence,
first failure index, collision-before-geometry ordering at the same sample, TCP
extrema and scalar fallback for a larger adjacent step. Full regression is 231
Python tests plus a successful Release build of both ROS packages and one passing
`colcon` C++ test.

## Saved-Range Commissioning Console (2026-09-09)

`Saved trajectory test` is integrated in the existing command console. Select an
immutable archive and an inclusive From/Through range (1-10). Equal endpoints
select one stage; a larger range runs those stages sequentially, preserving DDS
hand transitions, arm feedback checks and deliberate holds at stage boundaries.
It does not promise nonstop velocity across separate actions.

The operator confirms unchanged archived RGB-D scene/base placement and actual
hand-model agreement. Ranges intersecting insertion through withdrawal (3-8)
add an explicit physical alignment/contact-test acknowledgement. These are
operator-directed commissioning tests, not production hardware certification.
The normal freshly planned `task` execution gate is unchanged. Only the separately
validated `staged_test` range is eligible for explicit execution.

Any selected range may be previewed while holding, even if the real robot is not
at that range's start. Execution requires both arms, waist and both hand postures
to match the selected first stage; there is no automatic repositioning. Review
is invalidated by measured movement, STOP or changed selection/scaling. Stale
feedback blocks execution temporarily without discarding the reviewed trajectory.
The archive's original capture time is retained; stable operator-confirmed saved
tests do not expire solely due to review wall-clock time. Actual table/base motion
is not detected and requires a new capture/plan. Reboot is not a continuation of
an earlier live authorization.

Velocity/acceleration controls apply to the selected arm trajectories using
uniform time scaling of the original Hermite curves. Positions and normalized
curves are retained; derivative/waist bounds, joint margins and existing scoped
base/body collisions are checked again. Hand transition time remains two seconds.
The resulting per-stage durations are displayed before execution. Scaled copies,
validation provenance and execution results are saved in the current session's
`staged_test_NNNN` directory; original archived files are never changed.

The stored left plan `20260909_plan0004_left` has a 35.620 s first stage at original
0.15/0.15. The same curve is 25.187 s at 0.30/0.30 and 19.510 s at 0.50/0.50;
the acceleration envelope prevents speed-only linear shortening. A complete
offline 0.30/0.30 range validation passed without hardware commands.

STOP remains the existing immediate stop/hold-to-admittance path, not a newly
implemented or safety-rated emergency stop. Failure stops the task and records
completed stages; it does not skip ahead, reopen a caught finger or auto-recover.
After a selected range completes, review/select the next range explicitly.

On a fresh hardware startup the archive may be preselected with
`scripts/run_real_task_application.sh eth0 --hardware --saved-test-archive PATH`.
This does not ARM, load or execute automatically. Do not start another supervisor
while the current controller owns the robot; GUI-only replacement uses the
existing guarded handoff procedure and keeps the controller/RViz alive.

## Launch

Read-only application, live measured robot state, no arm command publisher:

```bash
/home/syf/g1_upper_body_ws/scripts/run_real_task_application.sh eth0
```

Add `--connect-vision` from a terminal to open an authorized SSH ControlMaster to
`unitree@192.168.123.164`. SSH prompts interactively when a key is unavailable.
No password is saved. Without it, image transfer requires existing key access or
`G1_VISION_SSH_CONTROL_PATH`. Vision and hand services are never started here.
The SSH master now permits 8 hours of idle persistence, with 30-second keepalive
checks and three unanswered probes before disconnecting. A disconnected or
expired master still requires authorized login; no password is stored for
automatic reconnection. Restoring the same ControlPath does not require a GUI
or controller restart; request a fresh plan afterward.
If the network interface is down, the read-only launcher opens a disconnected
UI in localhost domain 88, with no simulated startup state. Restart after
connecting the cable to use domain 0 live feedback. Hardware startup refuses a
down interface.

`--hardware` enables manual ARM and preparation/recovery execution. It does not
ARM automatically. The integrated bilateral admittance is a new commissioning
implementation, not a hardware-validated equivalent of the old SDK teach loop.
Do not use it for loaded/contact tasks before testing near a supported pose.

## Workflow

1. Enter R2+A yourself. Confirm and capture the stationary startup pose before ARM.
   It remains immutable for this session; the 14 arm joints and observed waist
   yaw are saved, not the hand actuator startup state.
2. Record each above-table preparation pose separately. The historical left pose
   is loaded, but its right-arm values are deliberately not treated as a right
   above-table target. Use `--ready-poses PATH` to reload a recorded pair.
3. Select left/right/both and click a plan button. Each click requests one fresh
   vision result with saved RGB-D. Failed requests never reuse an older target.
4. Review the target and complete time-parameterized path. Preparation/recovery
   require a separate execution confirmation in hardware mode.
5. STOP TASK invalidates all work immediately in the application and requests a
   measured-pose hold in the gateway. The gateway settles for 0.3 s, then blends
   into gravity-supported bilateral admittance over 0.5 s. Actual transition
   depends on measured speed and gateway health; read the gateway phase rather
   than assuming a service acknowledgement means the transition has completed.
6. Manually disengage the finger and return near startup. Confirm this, click
   Recovery to leave admittance, then Plan recovery and Confirm and execute.
   Recovery uses actual current state and the immutable session startup target.

The stop button is **not** power removal or a certified hardware emergency stop.
Native L2+B, stale-state, mode, velocity, joint limit, overload and temperature
behavior remain governed by the guarded gateway. No protection reset is called.
The new stop/resume epoch prevents a delayed old recovery request clearing a
newer stop. The stop callback has an independent executor callback group.

## State and scene

- The task RViz display listens only to `/g1/task/validated_path`, published by
  this console after full validation. MoveIt's intermediate output on the shared
  `/display_planned_path` topic is deliberately not shown as an executable plan.
- Planning shows elapsed time and sample-validation progress. ROS replies wake
  the worker directly rather than adding a 20 ms sleep per checked state. The
  collision samples, interpolation checks and hardware gates are unchanged.
- Execute has an explicit blocking reason. Expired plans leave `PLAN_READY` and
  clear the preview; a fresh plan requires a fresh Path clear confirmation. A
  plan whose observation expires during validation is never published as ready.
  `/g1/task/console_status` exposes these read-only UI diagnostics as JSON.
- These Python/RViz changes require a new application session. Do not kill or
  restart an armed controller to reload them; unload the arms and follow the
  deliberate robot shutdown/session restart procedure first.

### Checked Console Attachment

The September 9 legacy session was migrated without restarting its hardware
gateway. The old supervisor issued its normal task stop on GUI exit; the
migration checked the exact stop epoch, stationary state, unchanged trajectory
counters and controller identity before requesting the existing guarded hold
service. This briefly passed through admittance/hold blending. It was not a
zero-transition hot reload and did not send ARM or motion trajectories.

`real_task_app.py --attach-handoff PATH` accepts a short-lived checked manifest
for the same gateway PID/start identity. It inherits only the original confirmed
startup pose and recorded preparation poses, never plans or execution approval.
It does not ARM, clear a stop, or move the robot. A separate console lock prevents
duplicate new command panels. Attachment starts at `HOLD_FOR_REPLAN`.

Current units are `g1-task-console-20260909-154314-346142.service` and
`g1-task-rviz-20260909-154314-346142.service`. The original hardware supervisor
remains alive. The console has an `ExecStopPost` hook to request task stop on
exit/crash; RViz has no control-side shutdown hook. Do not blindly restart the
console unit: its one-time handoff manifest expires after 120 seconds. The
legacy migration script is specifically for the old coupled launcher, not a
generic restart command to rerun with stale PIDs.

Verification: `data/console_handoffs/20260909_154314_346142/completion_verification.json`.
The initial migration report records delayed RViz exit; a subsequent visualization
restart completed the migration. 48 task regression tests passed. This observed
low-load transition is not certification of stopping behavior under contact/load.

### Live Topics

- Arm/waist `/joint_states` come from the real LowState bridge. The hand adapter
  additionally publishes approximate phalanx angles from queried DDS actuator
  feedback, cached between reads and updated during hand transitions. These
  are not independently measured finger-link angles. No planned-path playback
  is published as actual robot feedback.
- One complete RGB-D capture is saved under `data/task_sessions/TIME/plan_NNNN/`.
  Cloud and visual targets publish once with transient-local durability.
- Measured points are frozen in the pelvis frame using the capture-time nominal
  torso transform. No old cabinet CAD is loaded. Moving the base is unsupported.
- A conservative 15 mm occupied-surface voxel mesh is applied to MoveIt. Missing
  depth is not observed free space. Robot self-filtering, multi-view coverage,
  calibrated camera mount and dynamic obstacle monitoring are not implemented.
  A self-observed hand can therefore cause conservative planning rejection.
- The selected insertion reference retains horizontal ray/plane intersection of
  `operation_diagonal_intersection` with a **+20 mm radial offset**. Camera mount,
  horizontal plane, and community-model fingertip TCP remain nominal assumptions.
- Hardware arm segments contain seven joints; preparation/recovery can sequence
  both arms with one command owner. Hand stages use a separate HandClient path.
  Optional waist-assisted eight-joint segments require a freshly confirmed
  waist-capable gateway. The nonmoving arm has dynamic gravity feedforward in
  task mode; waist yaw still changes its Cartesian position.
- MoveIt TOTG uses 0.60 rad/s and 1.50 rad/s^2 absolute limits multiplied by the
  two scaling values. Timing is uniformly stretched if the gateway's Hermite
  interpolation has larger interior peaks; the same interpolation is sampled
  for collision checks. This is not
  a claim of measured jerk or physical contact smoothness.

## Explicit commissioning limits

**D-ring hardware execution is blocked in this version**, including through the
UI callback. It can request observations and attempt complete planning of
approach, insertion, +120/0/-120/0, withdrawal, and return. No partial plan is
accepted. Small orientation freedom is bounded by the existing 5/10 degree
tilt/roll policy. Planning failure is a rejection, not a hardware-ready result.

The legacy task model assumes index-extended/other-fingers-folded static hands.
New hand-enabled sessions instead use the articulated model and explicit DDS
stages described below. Both keep the 40 mm connector, 60 mm diameter, symmetric
configuration and existing index TCP. Non-task endpoint mode remains unchanged.
The task panel still requires confirmation that actual fingers agree with the
approximate model. Measured finger/contact calibration and full real-scene
D-ring execution commissioning remain outstanding. Do not interpret a displayed
candidate or an offline successful path as proof of safe loaded operation.

## Offline RGB-D Replay (2026-09-09)

```bash
bash /home/syf/g1_upper_body_ws/scripts/run_task_offline_replay.sh --rviz --keep-open
```

Optional `--arm right` chooses the other hand; `--contract-tests` runs synthetic
contact checks before the archived task. This launcher uses domain 93, a
loopback-only FastDDS transport, and an exclusive replay lock. It starts NO
hardware gateway, LowState bridge, shadow executor, or trajectory action client.
Ctrl+C stops only this replay. It does not replace the existing hardware console.

Default capture: `data/task_sessions/20260909_154321_451801/plan_0008/`.
RGB, aligned depth, and cloud checksums are verified before use. That older
capture lacks saved joint feedback, so the initial state is reconstructed from
the previously completed `plan_0004` endpoint and is explicitly marked as such,
not measured capture-time state. New captures also save `capture_state.json`.
RViz labels archived state/data as OFFLINE. Reports, ROS logs, exported URDF, and
the reconstructed state are under `data/offline_task_tests/TIMESTAMP/`.

Preparation/recovery no longer publish whole candidate hands or D-ring target
decorations. A single-hand D-ring request shows target coordinates and the arc,
without an extra translucent hand mesh. Selecting both hands for a D-ring task
is rejected before capturing images. The validated animation still represents
the planned robot, not the measured robot.

`/g1/task/validate_state` applies contact rules to a per-request ACM copy. Only
the selected `index_2`, `index_rubber_2`, `index_pad`, and `index_tip` links may
contact the separate ring object during insert/rotate/withdraw. Background,
palm, wrist, opposite-hand and self-collision checks remain enabled. A full
scene snapshot replaces the validator's cache; geometry changes invalidate it.
Global ACM entries are not relaxed by task requests.

The ring/background split is PREVIEW ONLY: bbox plus elevated occupied cells
above a fitted local table plane, NOT a semantic mask. Ambiguous measured cells
stay in the background. The target is conservatively voxelized over a +/-120
degree sweep. Inadequate table/target support raises `TARGET_NOT_SEPARATED`.
This does not certify unseen space or loaded contact, and hardware D-ring
execution remains blocked. The approved horizontal ray-plane intersection and
+20 mm radial target offset are unchanged.

Initial IK explores bounded orientations and seeds, but accepts a solution only
after scoped collision checking. Kinematic failure, body-pair/depth collision,
TCP residual, and continuity rejection have separate diagnostic records.
Preinsert height candidates are 10/8/6 cm along the existing approach direction;
the operator-approved insertion point and circle center are not moved.
Event-based RPC waits, full Hermite interpolation checks, guarded joint margins,
velocity/acceleration scaling, and hardware protections are retained.

Offline result `20260909_164728_293559`: 25 MoveIt/FCL contact-contract checks
passed, including both hands and unchanged global ACM. The archived left task
did NOT produce a complete trajectory: 15 initial kinematic failures and 6
colliding solutions (shoulder vs torso/head), reported in 5.30 s of planning.
This is a failed task replay with passing contact-rule tests, not PLAN READY or
hardware commissioning. New code/model have not been loaded into the running
domain-0 hardware session; do not restart its expired attachment manifest.

Expanded left search `20260909_165114_021336` tried 63 candidates over the three
heights: 33 kinematic failures and 30 colliding solutions, no complete path.
Right replay `20260909_164836_388985` also failed IK at the original height.
RViz launched and loaded the model in that run; a screenshot-based visual
inspection was not completed. Geometry, orientation/reachability, and physical
calibration still need work before claiming end-to-end task readiness.

## Scored IK Search (2026-09-09)

The task planner now restores the original workbench objective:
`sum(dq^2) + 0.5*sum(wrist_dq^2) + 2*sum(max(0, 0.15-margin)^2)
+ 0.2*orientation_change_rad^2`.
All candidates must still pass scoped collision checks, TCP/orientation checks,
and the existing 0.02 rad joint margin. Cartesian segments keep the stricter
existing 0.12 rad maximum joint step and 2 degree orientation step. The original
fast path remains: the first continuation can be used when every joint step is
under 0.04 rad and the joint-limit penalty is zero. Otherwise valid candidates
are compared by score instead of accepting the first valid one.

Preinsert searches 91 orientations: all seven previous orientations plus
diagonal swing/twist combinations inside the SAME 5 degree tilt / +/-10 degree
twist bounds. It retains 10/8/6 cm heights and uses five deterministic selected-
arm seeds, without changing the other arm. The complete grid has 1365 jobs;
default search is capped at 600 calls or 40 seconds, checked between requests.
The first pass spreads calls across orientations, heights and seeds; it does
not exhaust each orientation before considering the others. This is bounded
sampled search, not exhaustive reachability or globally optimal task planning.

Preinsert candidates are ranked against the actual start state, not against
their numerical IK seed. Up to eight best sampled goals are recorded; the
planner can try up to three in score order if an approach path is rejected.
The full insertion/circle/withdrawal trajectory must still validate before
PLAN READY. No hardware execution gates or protection thresholds are changed.

`search_diagnostics` is saved in successful plans, planning failures, and
offline reports. It includes coverage, search termination reason, score terms,
ranked candidates, selected goal, and per-point Cartesian selection/fast-path
decisions. The offline tool also records RPC timing. To increase ONLY an offline
test's budget:

```bash
bash scripts/run_task_offline_replay.sh --ik-search-seconds 90 --ik-search-attempts 120
```

Replay `20260909_171044_715248`: all 91 orientations, five seeds and three heights
were represented among 120 calls (not all their combinations); 61.23 s search,
64 IK failures, 55 collisions and one joint-margin rejection. No valid ranked
goal, complete task trajectory or execution resulted. Default-budget replay
`20260909_170823_925716` covered 65 orientations before its time limit and passed
all 25 MoveIt/FCL contact-contract checks. The code is saved for new/offline
sessions; the existing domain-0 hardware processes were not restarted.

The hardware launcher does not automatically disarm or ramp out support on GUI
exit. It requests task stop and retains the controller. A deliberate unloaded
shutdown is required. Process death, network loss and machine failure are not
covered by the software stop; keep the robot-native stop accessible.

## Hand DDS and Frozen Waist Planning (2026-09-09)

New sessions started by `run_real_task_application.sh` load the articulated
RH56DFX model and enable the HandClient adapter. Startup remains nonactuating:
neither the hand service nor a hand gesture is started automatically. The
existing `hand_service` must already be running on the robot; this application
never opens its serial port or starts a competing owner. Use
`--legacy-fixed-hand` only to reproduce the old manually confirmed static model.

`build_task_hand_client.sh` builds the small C++ adapter against the colleague's
`HandClient`. Only GetState, SetTargets and Hold are exposed. The UI can read
both hands; only the selected hand receives a transition command. Prepare
plans open the selected hand before the arm approach. D-ring plans approach
with the observed posture, switch to index-only at preinsert, insert/rotate/
withdraw, reopen, and return. Recovery plans reopen after returning the arms.
Every finger transition is included in the preview and checked against the
scene with NO ring contact exemption. No finger joint is sent to the arm action.

Each hand command uses six normalized actuators in service order: pinky, ring,
middle, index, thumb bend, thumb rotation. The default postures are open
`[1,1,1,1,1,1]` and index-only `[0,0,0,1,0,0]`. Commands ramp over two seconds,
renew the service watchdog, and require three advancing feedback snapshots
within 0.03 of the final target. ACK alone does not release the next arm stage.
Faults, stale feedback, timeout, lost arm hold or robot displacement interrupt
the stage. STOP requests Hold on hands commanded by this client, independently
of the arm planning worker. Cancellation is checked inside the serialized RPC
lock so a queued target cannot follow STOP. No fault reset or force/current/
temperature setting is changed. This software stop is not a physical E-stop;
network loss can leave Hold unconfirmed and relies on the service watchdog.

The DDS-to-phalanx mapping is explicitly approximate: it interpolates the old
community model's open/task poses. Execution feedback is actuator position,
not optical measurement of passive finger links. Hand movement also need not
exactly follow the synchronized model interpolation under load. Actual finger
mapping, clearance and contact operation still need commissioning. The D-ring
hardware execution gate remains in place.

`--waist-assist` adds separate `left_arm_with_waist` / `right_arm_with_waist`
groups: seven arm joints plus yaw, bounded to +/-35 degrees. Original arm-only
groups remain unchanged. Roll/pitch stay fixed. Waist TOTG limits are 0.4 rad/s
and 0.5 rad/s^2 multiplied by the UI scalings; the common interpolated-path
stretch/check also respects those bounds. Waist participates in IK seeds,
scoring, joint-margin checks, full-body collisions and return trajectories.
Even with only one arm selected, waist motion moves the opposite hand in space.

Capture-time `pelvis_from_torso` is saved with the geometry. Targets, IK, TCP
checks, markers and measured obstacles use frozen pelvis coordinates throughout
the plan. The camera/torso may turn without moving the target. This assumes the
base stays stationary relative to the table; moving the platform requires a
new capture. No later image is requested during execution.

The updated gateway accepts seven same-side arm joints plus optional motor 12
(waist yaw), in arbitrary joint order, when `task_waist_enabled=true`. Enable it
through `run_real_task_application.sh ... --waist-assist` in a new, safely
established session. It retains one command owner and the existing action
endpoints. The diagnostic `task_waist_trajectory_supported=true` is required
by both the UI and execution callback. An older gateway remains preview-only.
All seven arm joints are mandatory; yaw cannot replace one, and roll/pitch or
mixed-arm trajectories are rejected. Yaw position, velocity, acceleration,
tracking and command-step checks include the interpolated segment, not only
its endpoints. Planner timing uses the same conservative Bernstein envelope
as the gateway so a declared READY path does not fail that new derivative check.

Task stop captures measured positions before arm admittance. Yaw remains held;
waist admittance is not implemented. Fresh epoch-based recovery and stale-goal
rejection are unchanged. No domain-0 process was restarted or real motion sent
in this update. Eight-axis hardware behavior is not commissioned; D-ring
execution remains blocked by the hand/contact/stop commissioning gate.

Offline-only verification:

```bash
bash scripts/run_task_offline_replay.sh --articulated-hands --waist-assist --contracts-only
bash scripts/run_task_offline_replay.sh --articulated-hands --waist-assist --rviz --keep-open
```

Report `data/offline_task_tests/20260909_175408_855532/report.json` passed the
contact fixtures including hand-transition contact rejection, both hands'
open/index-only transitions (162 configurations per side), and eight-axis
timing/interpolation fixtures (414 sampled states per side). These are synthetic
clear-scene tests, not successful full D-ring plans or hardware validation.
Missing archived finger state is labeled as an assumed open hand. No hand
service or hardware gateway is started in the loopback-only replay.

Archived full-task replay `20260909_175616_137538` searched 120 candidates with
both articulated hands and waist assistance: 118 self-collision rejections,
two joint-margin rejections, no complete plan or PLAN READY. Thus the successful
fixtures above must not be presented as a successful task replay.

### Hand Geometry and CAD Comparison (2026-09-09)

There is no solid whole-hand collision box in the task model. RH56 collision
meshes are separate palm/finger links, plus a 40 mm long, 60 mm diameter
connector. `scripts/inspect_task_hand_geometry.py` measures actual mesh vertices
in the wrist-yaw frame; use `.venv-cad/bin/python` (trimesh is installed there).
The approximate left-hand aggregate X/Y/Z spans in millimeters are:

| Posture | Hand meshes only | With connector |
| --- | --- | --- |
| Open | 192.5 x 70.2 x 151.9 | 232.5 x 72.8 x 151.9 |
| Index only | 188.8 x 91.0 x 94.4 | 228.8 x 93.6 x 94.4 |

Right-hand geometry differs by less than 0.4 mm in these mesh bounds; both
use the same posture configuration and connector dimensions. These are
aggregate bounding dimensions, not a solid rectangular obstacle. The still
running session `/tmp/g1-real-task-9UL9yR/robot.urdf` has the old fixed-open
hand model. New articulated offline models must not be confused with that
live session. Dimension reports are under offline replay `20260909_175616_137538`.

The previous 91-angle search refined only a small neighborhood of one nominal
orientation. The default search now restores the CAD grid of 160 nominal
orientations (5 tilts, 8 azimuths, 4 twists), changes the preinsert point along
the selected finger axis, and carries that selected nominal through insertion,
arcs and withdrawal. Contact-phase local orientation tolerance, joint margins,
scoring and step bounds remain unchanged. Free-space IK can search for a
collision-free branch internally; accepted results still use the scoped
validator. Up to eight diverse ranked candidates can try complete paths,
including fallback after a continuous-path failure. This is not a complete
port of the old entry-first search or its saved-solution warm starts.

Old saved CAD plans used a 40 mm radius, a moving target model and different
stance/start states. This archived visual target has a 77.7 mm radius and a
conservative static swept target. Old CAD used geometric playback; current
plans also need actual time parameterization and interpolated-path validation.
The old success therefore does not establish feasibility of this different task.

Replay `20260909_182853_667860` found 12 valid preinsert configurations in 80
trials/35.2 seconds with articulated hands and yaw enabled. All eight approach
attempts then rejected the same reconstructed start: open middle-finger meshes
overlap the archived background by 0.1-0.4 mm. Finger state was assumed, not
measured at capture time. No full plan or PLAN_READY was produced.

Diagnostic replay `20260909_183312_711101` tested contact paths from those
eight preinsert candidates without claiming an executable approach. All eight
failed during insertion, before any arc: seven final rejection records involve
non-exempt finger/thumb geometry against the swept target, and one is a joint
margin. This neither proves physical collision nor proves global kinematic
infeasibility. The diagnostic leaves `success=false`, never publishes a planned
path, and cannot enable execution. Reproduce with:

```bash
bash scripts/run_task_offline_replay.sh --articulated-hands --waist-assist \
  --contact-feasibility-only data/offline_task_tests/20260909_182853_667860/report.json
```

To compare the former local search, use `--orientation-search local`; the
default is `broad`. Keep the capture, start state, model and budgets identical.
Replay `20260909_183523_666359` used those same inputs and 35 s / 200 attempt
caps in local mode: it hit the attempt cap with 194 collision and six margin
rejections, zero valid candidates. Broad mode also changes collision-aware IK
branch selection, so this comparison is not an orientation-only ablation.

Gateway reports `logs/task_application_gateway_test{,_waist}/report.json` pass
both legacy seven-axis and new eight-axis simulated tests. These cover both
arms, reordered names, invalid ownership/limits/Hermite overshoot rejection,
stop-to-admittance, explicit resume epoch and stale-goal rejection. The simulator
follows commands immediately: these are not hardware dynamics/latency tests.

After aligning the planner's waist envelope, offline contract replay
`20260909_183750_735866` passed again, with 517 interpolated timing samples
per side. The 106 task tests plus 21 legacy orientation/vision-geometry tests
passed. No full-task success or hardware commissioning is implied.

## Tests

```bash
/usr/bin/python3 -m unittest discover -s scripts -p 'test_task*.py' -v
source /home/syf/unitree_ws/env/setup_unitree_humble_g1.sh
source install/setup.bash
unset CYCLONEDDS_URI
ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  /usr/bin/python3 scripts/check_task_gateway.py
```

The gateway integration test uses simulated LowState and a remapped test-only
LowCmd sink. It never publishes to real robot DDS. Its timings are desktop
software observations, not hardware stopping-distance or latency certification.
The passing fixture uses Fast DDS. The local vendor Cyclone DDS fixture did not
deliver LowState samples; live vendor-DDS transport is not validated by this test.
The Fast DDS fixture uses the repository's loopback-only UDP transport profile,
avoiding the local shared-memory data-delivery failure observed on reruns.

## Steeper Straight-Index Preview (2026-09-09)

The user withdrew the curled-index experiment. Current offline playback uses the
original `index_only` posture, a recalculated articulated-model TCP, and nominal
tilts 25/35/45 degrees from downward. The arc reference rotates at 0.15 times the
arc angle, retaining the existing local orientation and trajectory constraints.
The normal hardware planner's posture and fixed-reference defaults are unchanged.

```bash
bash scripts/run_task_offline_replay.sh --articulated-hands --waist-assist \
  --kinematic-preview --straight-preview --robot-raise-m .07 \
  --ik-search-seconds 30 --ik-search-attempts 220 --budget 420
bash scripts/run_task_straight_preview.sh
```

Planning is isolated to loopback domain93; playback uses domain94 and no action
client. `--max-index-tilt-deg` sets the largest nominal tilt, not the local 5-degree
tolerance. Synthetic height is additional relative to archived RGB-D; it is not a
measurement of the physical platform and must not be reapplied to live targets.

The new FCL experiment checks conservative per-link hand/palm/connector hulls
against a 50mm diameter, 20mm high cylinder. Its top is the existing axis-center
height. Selected shoulder-yaw/elbow/forearm/wrist-pitch links are also checked
against torso/pelvis/head geometry. D-ring geometry and the depth cloud are not
collision objects. This is NOT full self/environment collision validation.
No live scene ACM, safety limit, hardware gate, or running control session changed.

Successful left-arm result: `data/offline_task_tests/20260909_193420_926187/`.
All ten stages completed; planning 100.67s, motion 230.22s at scaling .15.
Actual contact tilt is 34.94–36.20 degrees from downward, with approximately
18 degrees of orientation change. Separate 50ms FCL sampling found 6.10mm minimum
hand/base clearance and 7.68mm minimum checked arm/body clearance. These small,
uncalibrated model clearances do not justify real execution. Reports remain
`hardware_executable=false`, `mode=STRAIGHT_PREVIEW_SCOPED`; the generic
`collision_checked=false` flag explicitly does not assert full collision checking.

Playback unit: `g1-dring-straight-preview-20260909.service`. The former height
gallery is stopped. The new gallery displays the base cylinder and scoped-check
badge, and removes stale straight-pose target arrows. A rendered screenshot is
in `data/offline_task_tests/gallery_straight_20260909/desktop.png`.
125 task unit tests passed, including posture mapping, FK changes, body/base
collision fixtures, isolation and the existing planner regression tests.

## Online Integration And Inward 5mm Adjustment

The current local radial offset is **+15mm**, replacing +20mm: the accepted
insertion point moves 5mm toward the shaft center. Online capture uses this once;
archived replay regenerates derived geometry from the original horizontal ray
intersection, not from the previously adjusted point. The archive is immutable.

`ScopedTaskPlanner` and `StraightIndexPolicy` now share the online and archived
D-ring planning implementation. The isolated adapter only adds the domain93
guard. Online profile `steep_straight` uses articulated straight-index FK,
25/35/45-degree nominal tilts, 0.15 arc-angle orientation tracking, base/arm-body
checks and the existing timing/scaling/continuity constraints. Prepare/recover
retain the original scene validator and motion planner. STOP/admittance/recovery
and hardware protections are unchanged. D-ring execution remains blocked by the
existing UI AND execution commissioning gates, with explicit non-executable plan
metadata. This integration does not certify the uncalibrated geometry or full
self/environment collision clearance.

The launcher now defaults to articulated hands, waist assistance and this profile:

```bash
bash scripts/run_real_task_application.sh eth0 --hardware --connect-vision
```

Only start this after the previous hardware session has been safely ended.
The owner lock still rejects concurrent controllers and startup does not ARM.
`--no-waist-assist` disables waist assistance; `--dring-profile legacy` selects
the former planner; `--legacy-fixed-hand` also selects the legacy profile.
The running old domain0 session was NOT restarted or hot-swapped in this change.

New complete offline result: `data/offline_task_tests/20260909_194905_936465/`.
Same left-arm capture and synthetic extra7cm, now radial offset15mm; planning
89.98s, full ten-stage motion169.16s at scaling .15. Contact tilt35.000–35.016deg,
orientation change18deg. A different IK branch was selected; sampled minimum
hand/base clearance26.34mm and checked arm/body clearance9.01mm are model-only.
The gallery launcher points to this report. Its screenshot is
`gallery_straight_20260909/desktop_inward5.png` under the offline results folder.

Verification:131 task tests +15 vision geometry tests passed. The actual online
TaskNode/Qt Window also initialized in isolated domain95 with new profile,
articulated hands and waist; no `/arm_sdk` publisher or hand DDS transport was
created and Execute remained disabled. No real hardware command was sent.

## Scene Cache Readiness Fix (2026-09-09)

Above-table preparation in session `20260909_201425_946703`, plan0004,
failed with `hand_transition: SCENE_NOT_READY` although the requested RGB-D
snapshot existed in MoveIt. Repeated state-only planning-scene diffs included
103 unchanged link padding/scale entries, continuously invalidating the cache.
`TaskSceneRevision` now compares actual padding, scale and fixed-transform
values, ignoring transform timestamp-only changes. Geometry, attachments,
ACM and actual transform changes still invalidate; collision rules are unchanged.

Build and `task_scene_revision_test` passed. Isolated contract replay
`data/offline_task_tests/20260909_204623_618157/report.json` passed, including
both hands, waist timing and contact-rule isolation; no full task was planned.
Only live validator PID328104 was replaced with PID342760 in user unit
`g1-task-validator-fix-20260909.service`, bound to and PartOf
`g1-real-task-live-20260909-steep.service`. Controller and console were preserved.
Read-only validation of plan0004's captured state returned two initial cache
warm-up responses followed by eight consecutive VALID responses. This verifies
scene readiness and that sampled state, not the entire preparation trajectory.
No ARM, trajectory or hand command was sent. Re-click Plan above-table to replan;
the existing session startup pose does not need to be captured again.

## Retimed TCP Diagnostics And Candidate Retry (2026-09-09)

`TaskPlanner.cartesian` now raises structured `RETIMED_TCP_BOUNDS` instead of
an unclassified RuntimeError. Diagnostics distinguish position, tilt and twist,
with measured errors, limits, actual/reference TCP, sample index/count and
intersample fraction. Per-segment maxima cover the samples checked up to failure
or completion. Candidate trial IDs and prior failures survive later cancellation.
The complete-path candidate loop retries this error within its existing budget
(8 broad candidates, 3 local); cancellation, timeout, stale scene, model errors
and invalid/nonfinite geometry still propagate immediately. Position 2mm,
tilt 5deg, twist 10deg and angular epsilon 0.0001deg are unchanged. No timing,
collision, velocity, acceleration or hardware-gate relaxation was made.

139 task unit tests passed, including classified deviations, unchanged limits,
sample diagnostics, retry success/exhaustion and non-retryable interruption.
Offline replay `data/offline_task_tests/20260909_210403_699097/report.json` used
live archive plan0016 and its capture_state.json, no synthetic height increase.
At scaling .15, candidate 1 failed continuous IK, candidate 2 failed the arm/body
check, candidate 3 (trial67) completed all ten stages in 169.52s planning time.
Maximum sampled model path deviation: insertion 0.013224mm; all contact segments
0.097454mm. Max relative tilt 4.962500deg, twist 9.980165deg. These are model
trajectory errors, NOT physical accuracy or continuous clearance guarantees.
The original failed IK branch was not deterministically reproduced in this run;
RETIMED_TCP_BOUNDS retry itself is covered by regression tests.

Changes are on disk only: running console329090 still has its original imported
planner. Do not restart the armed gateway or live supervisor to load this change.
No real motion/ARM/hand command was sent; offline report stays non-executable.

Deployment follow-up: user-requested guarded GUI migration passed at 21:11 on
2026-09-09. New console PID353148, unit
`g1-task-console-20260909-211140-235817.service`, loaded the new planner and
preserved hand/waist/profile flags, original startup pose and preparation targets.
Gateway328100 and RViz328109 were not restarted. The existing supervisor's
single stop was acknowledged (epoch1), then guarded hold was resumed after
identity/stationarity/drift/counter checks; no ARM or trajectory was sent.
Report: `data/console_handoffs/20260909_211140_235817/report.json`.
Current session: `data/task_sessions/20260909_211146_575896`, HOLD_FOR_REPLAN.
The handoff manifest expires; do not blindly restart this transient console unit.
Migration now accepts the optional measured startup yaw, retains original CLI
arguments, checks fresh idle UI status, includes yaw drift, keeps RViz alive and
can bind the console lifecycle to the retained supervisor unit. 13 handoff tests
passed. D-ring commissioning restrictions remain unchanged.

## Planning Efficiency And Frozen Preview (2026-09-09)

Online session211146/plan0003 completed its geometric trajectory, but the old
120-second observation-age guard rejected publication. For task plans explicitly
marked hardware_executable=false, same-generation frozen RGB-D previews may now
finish and remain visible without wall-clock expiry. Execution requires current
fresh/stationary feedback; measured arm/yaw motion over .025rad is latched even if
it later returns. STOP and new generation invalidate the preview; feedback gaps
retain it with execution disabled until observations recover. This
does not detect external table/base motion: the operator must recapture when the
scene changes. Executable prepare/recover TTLs remain unchanged. Session itself
now also refuses execution of explicitly non-executable plans.

FK compiles URDF joints and memoizes exact-state ancestor transforms per thread,
copying state and return matrices. No approximate cache keys. Trajectory sampling
evaluates each polynomial's unchanged 5ms sample grid as a NumPy batch. Bernstein
enclosures avoid redundant polynomial-root solving only when they prove the old
velocity/acceleration bound; inconclusive enclosures still use exact extrema.
Fixed saved 9755-sample approach benchmark: 2.2965s before, .1791s after for
stretch+sampling only, not a claim of 12x whole-planner speedup.

Cartesian planning retains ranked alternate solutions at recent points. On
continuous IK or retimed-TCP failure, it can reselect within a three-point window
and rebuild the suffix, at most three repairs per segment. The entire modified
segment is retimed and validated again. If unsuccessful, the existing bounded
whole-candidate retry runs. Cancellation/non-geometric errors are not swallowed.
No change to TCP bounds, collision scope, path sample density or final score.
IK service calls remain serial; this change is not a parallel IK solver.

Replay `data/offline_task_tests/20260909_212822_826279/report.json`: current
plan0003 capture-state/RGB-D, 74.49s planning, all ten stages passed. First
candidate's three local insertion repairs failed at waist margin; second complete
candidate succeeded. 6633 IK calls took32.03s total. Previous online capture-to-
failure was about185s including capture, not an identical benchmark scope.
155 task and40 workbench tests passed, including FK equivalence, mutation/thread
isolation, sample equivalence/extrema, local retries, frozen-preview movement
and expiry, and continued execution refusal.

Deployment: managed-console SIGTERM initially hung because rclpy shut down while
Qt continued running. Systemd's old90s timeout eventually killed that GUI and
its existing stop hook entered TASK_ADMITTANCE (epoch2). No automatic hold resume
was sent after the incomplete migration. New GUI now quits when its ROS context
is invalid; new console units use5s TimeoutStopSec and retain the stop hook.
Stopped-mode attachment preserves the stop state without sending commands.
Current console364836, unit `g1-task-console-20260909-213831-638633.service`,
session `data/task_sessions/20260909_213832_263300`. Attachment report
`data/console_handoffs/20260909_213831_638633/report.json` PASS. It loaded the
optimized planner and frozen-preview policy, preserving home/ready poses.
Gateway328100 and RViz328109 unchanged. User must deliberately resume hold before
planning again. D-ring hardware commissioning gate remains in place.
