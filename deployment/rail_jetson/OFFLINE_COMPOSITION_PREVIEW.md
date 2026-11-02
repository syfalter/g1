# Offline reusable-unit preview

This is an offline candidate generator and visualizer, NOT a replacement for the
production NWS_CYCLE executor. It never calls a hardware action/service, Push or
initialization. Exported candidates have hardware_executable=false until a
separate native integration and supervised validation are complete.

Start/stop the dedicated editor, without a real controller:

```bash
bash /home/syf/g1_upper_body_ws/scripts/task_editor_offline.sh start
```

```bash
bash /home/syf/g1_upper_body_ws/scripts/task_editor_offline.sh stop
```

Open the fourth tab, combination preview. Select D-ring or landing gear, count,
speed strategy and requested multiplier; generate, then use play/pause, the
timeline slider or a point on the speed chart. Playback uses planned seconds,
not an arbitrary frame rate. Play sends the remaining trajectory once on
/g1/task/validated_path; the Qt timer only advances the local time cursor.
The dedicated offline playback node interpolates it at 30 Hz and publishes only
/g1/offline_preview/joint_states. A separate robot_state_publisher produces the
offline_preview/ TF frames for a standard RViz RobotModel. Pause/seek uses
/g1/task/demo_preview_state to replace the current pose. Hide keeps the sole
offline model at its last pose; it never represents measured hardware feedback.
The topic names do not confer execution approval.
Chart modes are fingertip speed (mm/s, sampled) and
maximum absolute joint speed (rad/s). Chart samples are not the limit validator.
The save icon writes a compact candidate under data/composition_candidates.

The fingertip chart uses the geometric point Jacobian times the polynomial joint
velocity, including mimic joints and the visual hand schedule; it is not a
smoothed displacement-difference curve. A rippled speed profile is not by itself
proof of spatial jitter. The fastest strategy preserves the fitted path and can
vary local speed around its constraints. Denoising changes the path and needs a
separate bounded-deviation comparison, especially near contact.

## Bounded smoothing comparison

The phase-fastest view can generate both the unchanged path and a separate
spatially smoothed candidate. It applies a Gaussian filter along joint-space arc
length and a quintic interpolant on existing breakpoints, then blends toward it
only as far as the requested bounds permit. It never filters the displayed speed
curve to simulate smoother motion. Source recordings and selected versions stay
unchanged. The original and candidate can be selected and played independently.

Default bounds: 3 mrad per joint, 1 mm fingertip displacement and 0.5 degrees tool
rotation, relative to the original fitted path rather than measured physical
tracking. Joint displacement is bounded over every polynomial interval, not only
at samples. Cartesian/rotation bounds use a conservative kinematic-chain radius
and triangle inequality; they hold for the corresponding points along the path,
not equal wall-clock times after retiming. They are not collision/contact checks.
The standard retimer's small path approximation error is separate from these
pre-retiming smoothing bounds. The candidate retains a 2% blending margin.

Cut boundary positions and derivatives are preserved. For D-ring, only the
recorded prefix is smoothed; the generated inter-stroke connector and the final
parent exit tail are left geometrically unchanged. Gear recorded approach,
stroke and exit phases may be smoothed; generated connectors are retained.
Each candidate is retimed and checked again. Bounds and per-phase provenance
are saved under report.phases[].pathSmoothing; cached artifacts remain nonexecutable.

The initial 1 mm candidates show only modest improvements: the one-pair D-ring
task is 78.44 s versus 78.82 s; the documented gear example is 63.73 s versus
64.67 s. This does not demonstrate elimination of speed ripples. Curvature,
reversals and local limits can still require varying speed even on a smooth path.

### Gear continuous-unit candidate

For gear in fastest mode, enabling the bounded candidate now merges approach,
recorded motion and alignment into complete business units before smoothing and
retiming. There are six reusable units: entry at either rod position, upward and
downward strokes, and exit from either position. Former component endpoints are
interior spline points, not separately constrained stationary endpoints. Short
windows at eligible former joins and recorded cut boundaries are replaced by C2
quintics, with an explicit continuous path-deviation certificate. Direction
reversals and windows that exceed the bound are left unchanged. The local blend
and subsequent smoothing share one deviation budget, rather than each spending
the full tolerance. Grip,
release and rod-reversal boundaries remain stationary. Repetition still uses the
cached units rather than replanning the complete repeated trajectory.

The original variant remains the previous per-phase strategy. The continuous
candidate uses the existing continuously bounded spatial smoothing and local
retiming implementation; it does not claim constant TCP speed or eliminate every
recorded dwell. A trial that rebuilt the entire inverse arc clock magnified jerk
constraints and was rejected. For the current selections, the lower-start,
one-round-trip, upper-finish-with-exit case is 65.56 s versus 64.67 s originally:
removing hard internal stop constraints is not a guarantee of a shorter duration.

Continuous entry/exit units combine transit and possible contact motion. They use
the smaller of operation and transit rates, avoiding a nonzero-velocity boundary
between independently scaled pieces. Stroke units use the operation rate. The
UI labels these combined units as linked-rate units. No source version, saved
recording, single-action executor or hardware authorization is changed.

RViz messages contain only movable variables from the current URDF. Fixed waist
roll/pitch values retained in preparation metadata must not be sent as model
variables, since RobotState display rejects those names.

The offline launcher generates its own RViz configuration, without the MoveIt
RobotState/Trajectory overlays. This avoids their expensive link-visibility
updates on the articulated model. DDS packets are limited to 1400 bytes with
1024-byte DDS fragments: WSL mirrored networking was dropping fragmented IP
packets, starving state/TF updates despite an active publishing timer. These
settings apply only to the localhost-only domain 89 editor, not hardware DDS.

The new tab exists in the disconnected editor only. The formal hardware panel's
single-action workflows and source recordings are unchanged.

## Source and exit provenance

D-ring intermediate paths must be selected trim_tail versions. The stored parent
version must exist and the selected samples must match its retained prefix.
The full parent curve is fitted once; an exact quintic section retains the
derivatives at the cutoff. The final right suffix is taken from that SAME parent
curve, not replaced with a straight line. A final joint-space join returns to
the configured table preparation pose, matching the existing preparation-return
method. New between-action connectors are candidate quintics, not recordings.
They have joint-bound checks but no scene/contact clearance verification.

entry + cycle*(N-1) + final is used for D-ring. Final includes the right suffix
and return; cycle includes the right-to-left connection but no table return.
The standalone left/right action bindings are not overwritten.

## Phase-fastest mode (default)

The default strategy now uses the existing path-preserving fastest-rate search
on reusable phases, not uniform scaling of a complete cycle. It compares the
verified original local strategy with the local-rate candidate. This is the
fastest verified candidate found by that search, not a globally optimal result.
Existing exact timing is read when present; unoptimized edited traces are fitted.
The sources themselves are never rewritten or automatically smoothed.

The operating-rate and approach/return-rate controls each range from 0.1 to 1.0.
1.0 means the cached fastest candidate under 100% of configured motion limits;
0.5 halves its speed, quarters acceleration and reduces jerk by a factor of eight.
These are real trajectory timestamps, not a display-only playback multiplier.
Changing either rate reuses the fastest-phase cache without another optimization.
The old uniform/cruise comparison modes retain their old multiplier semantics.

D-ring approach to the taught ready pose and final return are transit phases.
Insertion, strokes and direction-change connectors remain operating phases.
A moving trimmed endpoint and its connector are optimized together until the
next stationary boundary. They cannot be scaled independently because that
would break velocity/acceleration continuity. The final right stroke retains
its parent exit tail. Landing-gear recorded approach/return paths and distant
joins are transit phases; engagement, alignment, disengagement and lever motion
remain operating phases. These labels follow task semantics, not contact sensing;
the operator still needs to inspect clearance in the recorded paths.

Current configured limits are arm velocity 0.40 rad/s, waist velocity 0.35 rad/s,
acceleration 1.5 rad/s^2 and jerk 5 rad/s^3, further bounded by model velocity.
No global configuration or hardware watchdog was raised. These are application
limits, NOT manufacturer performance limits or payload-dependent torque limits.
Position and continuous polynomial derivative limits are rechecked after rate
scaling and assembly. The phase table displays each phase's class, duration,
reuse count and limiting derivative constraint. Hand waits remain one second.

Landing gear uses recorded entry, lower/upper cycles, any final one-way transfer,
and optional recorded exit. Grip/release events and one-second visual holds are
retained separately. Finger feedback and contact timing still require hardware
integration; do not mistake the displayed presets for measured contact state.

## Timing, caching and limitations

### Single-trace playback transport

The isolated offline trace editor now sends a complete visualization trajectory
once per play/resume, like the composition preview. Timed versions use their
saved Hermite position/velocity/acceleration data; raw recordings retain their
linear sample interpolation. A separate preview process samples the trajectory
on a monotonic 30 Hz schedule. GUI ticks update only the cursor and readouts.
Pause, seek, version/tab changes and editor shutdown explicitly stop playback.
The hardware-connected panel keeps its previous preview path.

The trace clock readout includes planned maximum absolute joint velocity and
the preview publisher's output rate/maximum recent callback interval. Missing
feedback is identified after two seconds. These are NOT RViz rendering FPS:
stable output plus a frozen image still points to a downstream rendering/TF
problem. Initial trajectory loading may appear in the first health interval.

### Entry valley comparison (2026-09-27)

The landing-gear fastest preview now generates three selectable variants:
original phases, the previous continuous-path candidate, and V2 entry-valley
refitting. V2 changes only `entry_lower`; upper entry, lever strokes, exits,
hand events and business endpoints are unchanged. D-ring processing is unchanged.

V2 detects actual low-speed minima within 0.5 seconds of edit/phase markers,
instead of assuming the marker timestamp itself is a stop. Disjoint local
quintic replacements match the original boundary position, velocity and
acceleration. A monotone spatial correspondence is checked continuously with
polynomial bounds; it is not just a nearest-point sampled deviation check.
Spatial replacement and subsequent smoothing share the existing deviation
budget. The final retimer and continuous position/velocity/acceleration/jerk
checks still run; local duration estimates alone do not authorize a candidate.

The current saved lower-entry candidate takes 8.86 seconds versus 9.31 seconds
for the previous candidate. Three low-speed valleys were replaced. This is a
limited offline experiment, not a claim that every hesitation has disappeared
or that timing is globally optimal. Start with LOWER, both rates at 1.0, build,
then switch between the previous candidate and V2 and replay the first ten
seconds. Candidates remain visualization-only and are not pushed to Jetson.

CLI: append `--entry-valley-refit` to the existing landing-gear `--mode fastest
--smooth` check. Artifacts use the separate `-smooth-valleys-v2` suffix, leaving
the previous candidate artifacts intact.

The original option uses bounded uniform scaling of each complete unit. Cruise
uses the existing local arc-clock retimer and chooses the original strategy if
cruise fails or takes longer. The UI reports fallback counts rather than hiding
them. Both options preserve the selected unit path, not necessarily its clock.
Joint-space connectors carry source position/velocity/acceleration at their joins.

Unit boundaries remain zero velocity/acceleration, with no added execution wait.
This is NOT a proof of non-stop periodic motion. A fully nonzero-speed periodic
boundary needs an additional periodic timing/controller contract. The existing
cloud executor still submits separate complete actions; it has not been changed
to play these new candidates in this implementation.

Cache identity includes sources/parents, preparation poses, model, parameters,
speed strategy, multiplier and planner code. Repetition count is excluded.
Changing count assembles cached units and samples a chart; it does not refit or
retime the repeated recording. Assembly is linear in output points, capped at
200,000. A source version change invalidates the panel result. Original data is
never overwritten. Tests and evidence use an isolated model, not robot DDS.

On the saved left_5 v7/right_4 v10 selections, phase-fastest at both rates 1.0
produced 78.82 s for one left/right pair including entry and final return (the
previous whole-unit cruise candidate took 176.02 s). Each additional pair takes
63.57 s. Landing gear LOWER start, one complete round trip, final UPPER, with
exit takes 64.67 s (previously 191.62 s), with 29.11 s per additional round trip.
These are offline planned durations, not measured robot execution times.

Before real testing: review the full path and new connectors, integrate the
candidate format into the native execution path, verify hand schedules and
cancel/watchdog behavior, perform offline native acceptance, then supervised
low-speed physical validation. Current candidates cannot be uploaded as a
hardware-authorized release merely by changing a flag.
