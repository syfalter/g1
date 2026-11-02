# D-ring offline planning: method and integration evidence

Updated 2026-09-18, Asia/Shanghai. This is a research and engineering decision
record, not evidence of 95% G1 success or of hardware commissioning.

## Decision

Keep the existing model, straight-index TCP, prescribed hand sequence, scoped
collision validator, retimer, archive loader, and final dense checks as the
acceptance boundary. Improve candidate generation and whole-task selection
behind that boundary. A solver returning a pose or a contact path is insufficient.

The first implemented step is an MTC-inspired terminal feasibility callback:
Hand open and Return ready must pass before a contact branch can be accepted.
Failure can resume withdraw/rotate/insert alternatives. This is a bounded custom
stage search using existing generators, not an integration of the MTC library,
not a complete global graph search, and not an optimality guarantee.

An explicit offline bounded-IK candidate backend is implemented; its initial
two full runnable-subset evaluations are recorded below. Descartes-style finite candidate layers remain an
architectural reference, not an integrated library. No whole-stack migration or
default live-panel backend change has been made.

A later opt-in return-only OMPL 1.7.0 adapter was first tested in a full
derived-v2 evaluation; subsequent candidate experiments retain it. It uses the
original scoped collision predicate, not MoveIt's broader collision scene or a
new constrained contact model.

## Implemented Candidate Experiment

`--ik-backend bounded` uses the installed SciPy trust-region reflective least
squares solver and an analytic root-frame URDF Jacobian. Finite-difference tests
cover calibrated TCP offsets and revolute mimic contributions. The existing
ordered preinsert/contact seeds are unchanged. The optimizer uses a 0.021 rad
internal margin, at most 60 evaluations, and a per-call time bound. Acceptance
requires position residual <=0.0002 m and rotation residual <=0.002 rad, then
the original scoped collision, joint, tangent and full-path validators.

This is candidate generation, not a new acceptance contract or proof of global
IK/path completeness. Backend/library versions and seed policy are in each
replay report. There is no randomized local optimizer, but deadline truncation
and the existing free-space planner can still change outcomes under load.

Five preselected development tasks gave 4/5 passes in the first bounded run
and 1/5 in the MoveIt control. Auxiliary work overlapped the control, so timings
do not isolate backend causality. A preinsert geometry correction was also
needed: inclined saved rays can change the physical-center XY bearing between
entry and preinsert. Unconditional correction fixed one case but regressed a
previous right sweep (4/5). Preserving already-valid trial quaternions exactly
and correcting only invalid ones gave 5/5 in `bounded_ik_parallel5_v3`, complete
case times 45.65-86.23 s. All hard gates remain unchanged.

The unchanged 109-task selection then gave 84 complete archive passes, five
failures and twenty timeouts in `all_replayable_bounded_parallel6_v1`, six
workers on domains 94-99. All successful cases included export and actual reload
within 180 s (maximum 154.83 s). All five development regressions passed again.
Complete required-direction pair coverage was 73/337, not 95%. The final task
denominator and independent physical scene split remain unresolved. Brief
auxiliary RViz, environment setup and perception work overlapped the benchmark;
this is not a clean single-factor timing comparison.

The two generic TOTG errors exposed a stage-search integration defect: numerical
retiming failure aborted the case instead of rejecting its candidate. The
post-batch fix maps only known TOTG/Ruckig numerical failures into the existing
recoverable failure class; contract errors, cancellations and deadlines retain
their fail-closed behavior. A separate five-case run passed all five, including
both former failures with recorded TOTG failure followed by successful branch
recovery. These results are not pooled into the full 84/109 numerator.

The unchanged 109-task cohort was then independently repeated with these fixes
in `all_replayable_bounded_parallel6_v2`: 86 complete passes, two candidate-path
failures and 21 timeouts; 75/337 conservative pair coverage. All 84 prior full
passes repeated, and both former TOTG failures recovered within the deadline.
Accepted complete-case times were 21.64-150.16 s; all code audits passed. The
extra timeout was a prior return-path failure, not a previously successful
case. This establishes one additional run, not a general 95% guarantee or a
pooled numerator. Only lightweight evidence audits/preparation overlapped;
worker scheduling and wall-clock truncation still prevent a causal speed claim.

Archive reuse also now uses hash-verified archived meshes for collision-model
construction. The live model must match mesh bytes as well as URDF text. A
separate nine-job validation of relocated archives passed all existing checks
at three speed/acceleration settings, preserving the normalized path within
4.30e-9 coefficient rounding error. This neither changes collision scope nor
provides hardware/contact-tracking validation.

## Initial-Candidate Diversity Experiment

The existing full preinsert grid was enumerated independently for the two hard
right sweeps, with the same bounded pose IK and production scoped state checks.
`preinsert_grid_probe_v1/probe.py` creates no ROS nodes or service calls. It
checks the archived geometry record, measured capture state, calibrated TCP,
frozen transform and base center before searching. Each grid has 504 jobs.
`plan0052.v1.json` found 63 accepted states in orientations 7/15/23;
`plan0080.v1.json` found 65, including two orientation-18 states. All seven seeds
were covered in the full grid. Respectively 25/25 accepted states were at least
0.25 rad in maximum joint-coordinate distance from every previously attempted
complete candidate. This establishes missed sampled states, not independent
kinematic components or full-path feasibility.

The opt-in `--preinsert-policy diverse` leaves the first batch exactly in the
legacy order, then interleaves orientation/height/seed combinations without
adding or dropping grid jobs. Across subsequent batches, it prioritizes
orientation families with fewer delivered candidates; within equal counts,
it maximizes minimum joint-coordinate separation from previously delivered
states in the same orientation family, then uses the original score and trial
index to break ties. This is a coupled scheduling/selection experiment, not a
single-factor attribution to seed ordering. The legacy policy remains default.
IK tolerances, physical tangent branch, collision checks, candidate caps,
retiming, stage backtracking and whole-case deadline gates are unchanged.

`diverse_preinsert_parallel6_v1` completed 26 directions from 20 explicit
contexts: 23 passes, two failures and one timeout; batch wall 378.679 s.
Successful cases took 24.396-120.270 s. Seven of the eight old full-run timeouts
passed, and all 15 old passes in this selection remained passes. The already
startup-repaired direction passed too; it is not credited to scheduling.
`preinsert_grid_probe_v1/targeted_audit.v1.json` freshly reloaded all 23 archives
and verified source/native snapshots and first-batch scheduled requests.
These results are not pooled into the previous 100/111 full-run score.
The subsequent complete `all_replayable_diverse_preinsert_parallel6_v1` run of
the same frozen implementation passed 108/111 directions, with two failures
and one timeout, batch wall 992.154 s. All 100 old passes repeated, and the
same seven old timeouts passed again. Successful case times were
24.493-126.130 s (median 41.070 s). Required-direction pair coverage is
96/337 (28.49%), still including all 238 blocked pairs. The 97.30% runnable
direction rate is NOT the user's all-valid-history 95% metric. Defaults remain
unchanged; use all three explicit bounded/OMPL/diverse options to reproduce it.
`preinsert_grid_probe_v1/full_audit.v1.json` independently reloaded all 108 new
archives and checked all 111 integrity records. The archives were then copied
unchanged into the default trajectory library; the delivery audit reloaded all
439 library entries without errors. Repeated contexts across batches are not
additional unique successes. Reported geometry maxima and dense-sample counts
are audits of stored validation summaries, not new dense FK recomputation.

Both hard right sweeps still failed. The newly selected orientation-18 states
in `plan_0080` were rejected by existing approach collision validation, so
preinsert-state coverage alone did not repair the complete path. The remaining
`20260912_155322_095139/plan_0035` right timeout in the targeted batch retained
a failure report after all 16 candidate attempts; insert/rotate IK and approach
collision failures are recorded. A completed failure report does not override
the supervisor's whole-process deadline classification. In the full batch that
same task timed out without a report, so its partial validation timing cannot
be reconstructed. None of these negatives proves global continuous-space
infeasibility.

Three newly recovered left-sweep archives were relocated and revalidated via
the existing `prepare_range` path at speed/acceleration (.2,.2), (.4,.5),
(.8,.9). All nine checks passed, with original waypoints and hand timing intact,
and maximum normalized polynomial difference 3.87e-9. This is saved-range
reuse validation, not new planning credit. The first RViz smoke attempt failed
to complete TF/viewport readiness in 80 s despite receiving simulated joint
messages; it remains in `rviz_diverse_recovered_left_v1`. An unchanged-code
second run, `rviz_diverse_recovered_left_v2`, passed all seven stage state/TF
and viewport checks. A passive domain-93 observer also received the required
transforms. This repeat does not establish the cause of the first startup
failure or erase it. Three stage screenshots were visually inspected.

The 293 selected regression tests passed, including schedule coverage/ownership,
cross-batch history, default behavior, scoped validators, archive handling,
panel control mocks, retiming and runtime provenance. Auxiliary archive reuse
and visualization overlapped the full batch; timing is not a controlled
single-factor performance comparison. No hardware commands were sent.

## Recovery Evidence

Original camera C receipt windows omitted by the corpus artifact allowlist
were checked independently against raw files and latest session snapshots.
All 24 unique RGB-D associations pass file integrity; 19 pass the existing
stationary software association contract, while five retain their original
provenance/clock rejections. The 19 have no measured hand state or valid raw
D-ring target geometry. Stationary RPC-window association is not exposure
synchronization, and a calibration arm does not establish a sweep task arm.
No task input or success numerator was promoted from those windows.

The separate `recorded_ready_recovery_v1/evaluation.v2.json` uses an explicit
terminal planned return goal from the SAME complete historical dual-sweep
plan to unblock one pair's two direction tasks. It never claims the goal is
measured feedback and does not replace the original capture state. Original
inputs and v1 evaluation remain immutable. The new view has 111 runnable
directions instead of 109, but the conservative denominator remains 337;
any resulting test is reported separately from the unchanged-v1 full runs.

Both recovered directions were tested concurrently under the original 180 s
contract: right passed all stages/export/reload in 50.368 s; left timed out in
continuous rotate search. This is not a new complete successful pair. The right
archive was published with its planned-goal provenance; no measured ready pose
was invented and no original benchmark numerator changed.

`targetless_intent_audit.v1.json` then reread the original plan/failure records
for all 83 targetless captures with valid raw 3D points. They are 73 prepare and
ten recover contexts; none records a sweep direction. All have a stored camera
transform, so it would be inaccurate to say no transform exists. Nevertheless,
geometry-only conversion cannot establish a historical sweep task, and their
geometry files lack complete calibration snapshots. No inputs were promoted.
The derived-v2 readiness audit still has 238 blocked pairs.

`unknown_task_intent_audit.v1.json` additionally checked all fourteen unknown
task contexts against original local capture/session JSON metadata. No nested
or top-level explicit sweep/schema/requested-direction field was found, and
no original metadata mismatched the dataset copies. Only two unknown tasks
are otherwise input-complete. The seventeen otherwise-complete blocked pairs
are four prepare tasks, eleven center checks and two unknown tasks. This audit
recovers no inputs and changes neither the denominator nor any success count.

## Startup Dependency Evidence

The single full-run service-availability failure occurred before native timing
service creation: a `PlanningSceneInterface` member waited on scene services
although no demo objects were enabled. Its only use was `add_demo_scene`.
Creating it only when a demo is requested removes that unused constructor
dependency without changing geometric planning, retiming math or collision
scope. The source/binary before the change are retained for native A/B tests.

Installed MoveIt planning-interface version: `2.5.9-1jammy.20260804.223930`.
The official 2.5.9 source confirms the default interface constructor waits for
GetPlanningScene and ApplyPlanningScene services. Retrieved source:
`https://raw.githubusercontent.com/moveit/moveit2/2.5.9/moveit_ros/planning_interface/planning_scene_interface/src/planning_scene_interface.cpp`.
Local copy: `lazy_scene_startup_v1/research/moveit_2_5_9_planning_scene_interface.cpp`,
SHA256 `e308be6f93719423b0bd6b4d1a56415e0dae6d7e9397d1ea2f84bb830fb99654`.

Eight final v6 old/new native fixtures cover both demo flags independently and
together. They withhold ApplyPlanningScene for 12 s after both MoveGroup clients
initialize. The no-demo new constructor completes and exposes the timing
service without it; the old constructor does not. Enabled demo objects still
wait. Retimed trajectory messages and applied demo-object requests match
exactly in all four A/B pairs. These are dependency/equivalence tests, not
whole-case planning successes or proof of general DDS response reliability.

In particular, v5 recorded a new/no-demo early-response failure despite native
readiness: twelve replies were not delivered before the scene provider was
released. It remains a negative result. Final v6 reports constructor completion,
advertisement and early replies separately, rather than accepting graph service
names as evidence of available servers. Production RPC retries are unchanged.
Installed RMW version: `6.2.9-1jammy.20251108.023501`; its official response code
identifies the logged timeout as failure to match the client's response reader:
`https://raw.githubusercontent.com/ros2/rmw_fastrtps/6.2.9/rmw_fastrtps_shared_cpp/src/rmw_response.cpp`.
Local copy: `lazy_scene_startup_v1/research/rmw_fastrtps_6_2_9_response.cpp`, SHA256
`822de5aff8a10e984e6c1a6033d2cdba4329513c99310d8f07a1de7c185a1a03`.
This locates the failure path, not the underlying cause of delayed matching.

The independent eight-direction real regression passed all validation, export
and reload gates in 24.536-102.733 s/case with four workers. The previously
unavailable right task passed in 46.898 s. Original full-run counts remain
100/111 directions and 88/337 pairs; no retrospective retry substitution is
made. Native source/binary, launch and DDS identities are now frozen alongside
scripts and included in fresh archive provenance. No shared ROS packages or
system dependencies were upgraded; external ROS shared libraries are not
copied into the snapshot. Evidence: `lazy_scene_startup_audit.v1.json`,
`lazy_scene_regression_parallel4_v1/summary.json` and
`lazy_scene_delivery_audit.v1.json`.

## Position Search Diagnostic

`position_search_half_selection_v1/selection.json` freezes two prior hard right
sweeps and two regression controls before the experiment. The only effective
change is `position_search_fraction: 0 -> 0.5`, using 1.5 mm axis offsets after
nominal continuation fails; acceptance still uses the original nominal path
and unchanged 3 mm Euclidean gate. The production/default YAML is unchanged.

`position_search_half_parallel4_v1` passed the two controls (61.638 s for latest
left plan0011, 174.531 s for earlier right plan0020), while both hard right sweeps
timed out. All four code-integrity checks passed. No hard failure was repaired,
so this setting was not adopted. Its four workers shared the host with the
separate two-worker ready-recovery test, on six exclusive domains in total;
the full-run baseline had a different scheduling mix. This is a limited
single-factor diagnostic, not causal latency evidence or a whole-history gain.

## Continuation And Return Experiments

`bounded_task_parallel6_v1` tests actual-previous-state +/-0.119 rad bounds and
candidate-only 2 mm / 2-degree residual acceptance. Strict preinsert IK and all
final task gates remain. This is not an exact tangent-constrained optimizer:
the least-squares objective still targets a complete sampled orientation, and
the physical tangent gate is checked afterward. Three controls passed, two hard
right cases failed all 16 candidates at orientation checks and the right control
timed out. No new hard case was repaired; the backend stays opt-in and is not
adopted. SciPy 1.8.0 official least-squares documentation was read and saved as
`research_continuation_v1/scipy_least_squares_1_8_0.html`; source:
`https://docs.scipy.org/doc/scipy-1.8.0/reference/generated/scipy.optimize.least_squares.html`.

The bounded-v2 failure at `20260910_205306_042305/plan_0008` left recorded eleven
return collisions among sixteen complete-candidate failures. Repeating contact
search cannot substitute for a free-space return planner when only a straight
joint line is attempted. The next experiment therefore uses the installed OMPL
1.7.0 library for this return stage only, not for constrained contact motion.

The optional pybind11 adapter uses RRTConnect in the selected arm/waist joint
space with the existing scoped Python collision predicate. It accepts exact
solutions only, preserves passive/hand state, uses local seeded sampling and
deterministic linear nearest-neighbor lookup, and bounds search to 4 s inside
the unchanged whole-case deadline. Candidate edges are sampled at <=0.02 rad
L2 intervals; this is not continuous-collision proof. The original final dense
timed validator remains mandatory. Checked shortcutting and stopped quintic
joint-line pieces preserve the candidate polyline without interpolation overshoot.

`ompl_return_parallel6_v1` passed all six preselected direction tasks, including
both directions of the old return case and four controls. The repaired left
archive records its original straight-path collision and the accepted OMPL
detour. Complete cases took 56.352-97.447 s. This subset does not establish
repeatability or any full-history gain; full derived-v2 evaluation is separate.

The subsequent `all_replayable_ompl_return_parallel6_v1` attempted all 111
derived-v2 directions: 100 complete archive passes, three failures and eight
timeouts, all original gates and 180 s deadlines intact. Complete-case passes
took 23.216-134.572 s; batch wall was 1092.956 s. Pair coverage is 88/337, and
the final all-history metric remains unresolved, not 95%. The 109 common tasks
were verified input-identical: 86 -> 99 passes, with fourteen previous timeouts
recovered and one previous pass failing service readiness. Added ready-goal
directions contributed one right pass and one left timeout, reported separately.
All fourteen recovered common tasks used accepted OMPL returns. Forty-seven
successful full-run archives record this fallback; one coarse candidate was
rejected by final dense collision validation before another branch succeeded.
Thus an OMPL exact endpoint result is still not a final trajectory acceptance.

The repaired return archive also passed independent relocated-model reuse at
three speed/acceleration settings. Its joint positions and hand timing were
unchanged, with normalized polynomial error <=3.9014e-9. The published full-run
version passed seven-stage RViz state/TF/render checks. These are reuse/playback
checks, not additional planning successes or hardware commissioning.

Primary 1.7.0 sources were downloaded and inspected under
`research_continuation_v1/` from `https://raw.githubusercontent.com/ompl/ompl/1.7.0/`:

- `src/ompl/geometric/planners/rrt/src/RRTConnect.cpp`, SHA256
  `1817e39ce3c20ae924a8c4d5476d2353083ef1c1cbfda58df9bfaeb83d604241`.
- `src/ompl/base/src/DiscreteMotionValidator.cpp`, SHA256
  `432ed37bc4af87cc4e8cfd6df548183d7ffd9d31f090d015d99da651dc88e058`.
- Installed `/opt/ros/humble/lib/x86_64-linux-gnu/libompl.so.1.7.0`, SHA256
  `25e9cbdc7c1c3b1e7c56221e954e72c4f5cb96e6c45253631773655cac71499e`.

The source headers carry the BSD-3-Clause terms. This existing binary/version
is distinct from the earlier unbuilt repository-head pin below. The adapter
was compiled locally against the installed headers and Python 3.10; its binary
hash is recorded in each replay and frozen benchmark. No library was downloaded
or system-installed by this experiment. The OMPL implementation was inspected
for exact/approximate result handling, edge checks and cancellation cleanup.

## Direct Task-Constrained IK Experiment

The installed SciPy 1.8.0 SLSQP interface was fetched from the official versioned
manual, not inferred from an untested current release:
`https://docs.scipy.org/doc/scipy-1.8.0/reference/optimize.minimize-slsqp.html`.
The retrieved HTML is `contact_constraint_probe_v1/scipy-slsqp-1.8.0.html` under
the existing offline-history root, SHA256
`2b077ef3090fad9025b1e46bf526c1cd3392ca0d3c103305a24f55626dbaa397`.
The implementation uses the already-installed optimizer, with no new package,
native rebuild, controller, gateway or hardware dependency.

`--ik-backend constrained` is a separate, opt-in experiment, not a new default.
Preinsert and free-space candidates still use the bounded TRF backend. Contact
candidates directly constrain the Euclidean waypoint ball (2 mm internal),
physical-center XY tangent with fixed branch, tilt/twist at both nominal and
actual TCP reference positions, previous-orientation step, previous-joint
continuity (+/-0.119 rad), and model limits (+0.021/-0.021 rad). The selection
angles have an additional 0.01-degree numerical guard. All existing outer
candidate, scoped collision, retiming, dense geometry, export and reload gates
remain in place. The default 3 mm / 5-degree task contract is unchanged.

The objective favors position tracking, small joint motion and reference
orientation; the seven inequality rows have analytic joint derivatives via
the existing TCP geometric Jacobian. Swing/twist uses the actual Z swing/twist
decomposition, not a rotation-vector Z component. Cancellation and whole-case
deadlines propagate; only a distinct local-budget exception becomes a rejected
candidate. Optimizer success alone never authorizes acceptance: FK geometry
and guarded joint bounds are independently rechecked before scoped validation.
This is a local sequential solver, not a whole-trajectory optimizer or
complete constrained planner. It uses one solve per existing joint seed rather
than the original angular-candidate grid; losing useful alternate branches is
a material risk that the complete cohort must measure.

### Rejected-State Evidence

`contact_rejection_probe_parallel2_v1` kept ordinary `bounded` behavior and
captured both hard right sweeps' actual seed, previous state, target transform,
reference/previous quaternions, waypoint index and rejected answer. The two
complete attempts failed in 118.756 and 147.297 s; their turn-stage searches
reached at most 39 and 37 accepted waypoints. Counts include 4,753/4,034 joint
continuity rejections and 711/1,915 local IK residual failures. Target-heading
prefilters are counted separately and do not have solved-state snapshots.

The standalone `contact_constraint_probe_v1/probe.py` tested 184 and 214 saved
rejection examples without ROS. Every example found a candidate passing the
independent local geometry/continuity and existing scoped collision gates.
397/398 also met the prototype's tighter normalized solver-slack threshold;
one optimizer status-8 candidate missed that numerical threshold but still
passed the actual local gates. Analytic-vs-central-difference relative-scaled
derivative discrepancy was at most 3.683e-7. These are latest examples per
code/segment, not 398 failed tasks or 398 complete paths. The prototype inferred
the branch from the recorded valid previous state; the integrated backend uses
the planner's explicitly fixed branch.

The first complete targeted experiment, `constrained_contact_parallel6_v1`,
passed eight control direction tasks but failed both hard right sweeps. Their
maximum accepted rotate progress became 43 and 39 waypoints; no complete hard
case was repaired. Replaying the new failed states with a 0.25 s diagnostic
local budget produced only 2/17 and 2/16 independently valid candidates.
Additional time at one waypoint is therefore not an established complete-case
solution. This result does not prove that other local seeds or paths cannot
work.

The frozen complete cohort subsequently produced 97/111 passes versus the
previous bounded/OMPL run's 100/111, with four pass-to-fail regressions, eight
timeout-to-failure transitions and one already startup-repaired service case
passing. Pair coverage was 85/337 versus 88/337. The comparison and failed
dense-validation timing are retained in `ompl_constrained_full_comparison.v1.json`
and `constrained_experiment_audit.v1.json`. This is a negative adoption result:
the direct local solver is not the default, even though it solved the earlier
individual rejected states. Preserving useful alternate paths and satisfying
retimed geometry remain separate problems from finding a local feasible IK
state. The result does not isolate a single causal source of the regressions.

### Endpoint Screening Diagnostic

The separate `endpoint_probe.v1.py` searched 24 nominal orientations with three
seeds each at the final rotate point. It deliberately omitted predecessor
orientation/joint-step constraints because an isolated endpoint has no
predecessor, while retaining the physical tangent, tilt/twist, position, model
margin and existing collision checks. It found 9/72 and 11/72 endpoint states.
These are necessary-condition probes, not relaxed path passes or global
infeasibility certificates.

`endpoint_probe.py --stage preinsert` then tested one recorded height per
orientation, not the full preinsert-height grid. Both cases had preinsert/end
solutions for orientation family 16. However, `continuation_probe.py` failed
to extend all three such initial states even through insertion: 47/59 accepted
insert waypoints for plan0052, and 31/59 for both plan0080 seeds. These probes
use a 0.25 s local budget, waypoint FK and sampled linear-edge scoped collision
checks, but have no approach, retiming, dense timed-path acceptance, return,
export or whole-case deadline credit. Endpoint feasibility alone did not solve
the intervening contact path. The source hashes and all negative outputs are
preserved beside the scripts. No endpoint-screened seed policy was adopted.

### Production-Reference Correction And Reverse Continuation

The old endpoint probe above did not exactly reproduce production's tangent
alignment: it applied only the change in physical radial bearing to a recorded
preinsert quaternion. Some recorded quaternions are within the candidate
tangent tolerance but are not exactly aligned. Production realigns them in
`StraightIndexPolicy.tangent_reference` before moving the reference. This
difference can change the tilt/twist result. The old source reports are
`constrained_contact_parallel6_v1/0005/replay/report.json` and `0006/replay/report.json`;
their hashes match the saved endpoint provenance.

`reverse_contact_probe_v1/reverse_probe.py` reconstructs the actual scoped
planner and captures its production contact specifications without ROS calls.
It rechecks the calibrated TCP, contact hand posture, unselected joints,
physical/arc centers, model and collision scope. Of the 9/11 old endpoints,
only 3/4 pass current endpoint checks; all three old orientation-16 endpoints
fail current tilt bounds. This qualifies the earlier orientation-family
intersection claim; those old results must not justify a production seed policy.

`endpoint_seed_probe.py` uses the exact production reference, the same finite
24-orientation/three-seed grid and the existing constrained-IK equations.
The isolated endpoint has no predecessor, so joint/orientation predecessor
constraints are explicitly omitted there only. It found 13/72 and 14/72
collision-checked endpoint states, all freshly revalidated by the reverse probe.
The latter restores the .119 rad candidate interval and 2-degree orientation
step, and checks sampled linear edges with the existing scoped batch validator
and actual-TCP moving reference. All 27 attempts failed during reverse rotation;
the best traces reach 98/120 and 91/120 waypoints. None reaches insertion.
The three `RETIMED_TCP_BOUNDS` errors here are sampled-edge tangent violations,
not a claim that retiming was performed. No approach, hand transitions,
withdrawal, return, timed dense validation or archive was generated.

These finite local negative probes do not prove continuous-space infeasibility.
They show that endpoint feasibility plus single-seed reverse continuation did
not solve the complete task at unchanged gates. They add no success count and
do not justify replacing the bounded/OMPL/diverse complete-run configuration.
All scripts, positive isolated states, negative path traces and source hashes
are retained under `reverse_contact_probe_v1`; live defaults remain unchanged.

## Evidence And Version Pins

Primary repository metadata and README snapshots are in
`data/offline_history/20260917/research_v1/sources.json`. Trees, selected build
files, licenses and file hashes are in `research_details_v1/details.json` under
the same parent. These are retrieved snapshots, not assertions that a branch
will retain these contents. That original repository collection built no
alternative solver; the later adapter above links an already-installed OMPL
1.7.0 binary, not the unbuilt head pin in this table.

| Repository | Evaluated commit | License evidence | Humble / Python 3.10 integration assessment |
| --- | --- | --- | --- |
| swri-robotics/descartes_light | `49e2ee3305b5d7ba5018f28cf706e0dc71ecf6cf` | Apache-2.0 root and sampled interface headers | C++/CMake library, not a Python drop-in. Needs G1 waypoint sampler, edge evaluator, cancellation and binding/service adapter. Not built here. |
| tesseract-robotics/tesseract_planning | `817a62295ee74142f78659632006395dd9f43cae` | Per-file Apache-2.0/BSD-2-Clause/BSD-3-Clause; umbrella LICENSE is not a single blanket grant | C++ stack with Tesseract model/environment dependencies. A second model/scene representation needs equivalence tests. Pinned head not verified with local Humble. |
| tesseract-robotics/trajopt | `901d3432f30fc3bd793692f024d3ef6afc9a0c9d` | Sampled core LICENSE files BSD-2-Clause; package declarations also include Apache-2.0. Audit intended components separately. | Custom G1 residuals/Jacobians and constraint adapters required. Not a replacement for final hard gates. No Humble/Python build tested. |
| ompl/ompl | `68614a0b1a0124049d2bbcdbbfe40cc8089c6bfa` | BSD-3-Clause LICENSE | Existing MoveIt free-space use is distinct from a custom constrained state space. Need manifold projection and ordered path-progress treatment for contact. Pinned head not built. |
| moveit/moveit_task_constructor, humble | `756634951326ae17ae099882f7110c6f1d0a98c0` | BSD-3-Clause LICENSE.txt | Explicit Humble branch exists. C++ task integration still needs scoped validation, custom contact stages, deadlines and current Python-panel adaptation. Not built here. |
| HIRO-group/trac_ik, master | `2e63d6b2b05095e34b51672f8475a7390f75125c` | Sampled `trac_ik_lib/src/trac_ik.cpp` has BSD-3-Clause header; root license absent | Requested `ros2` branch did not exist. Actual source includes ROS1 headers/catkin. Do not present this pin as Humble compatible. |
| TAMS-Group/bio_ik | `1de267840678fc3237ee90c8788e835e1966fbf4` | Package BSD declaration and sampled `include/bio_ik/goal.h` BSD-3-Clause header | Snapshot uses ROS1/catkin. Goal API is useful evidence but a ROS2 port is separate work. No Python 3.10 integration verified. |
| PickNikRobotics/pick_ik | `c476d3aab7879c54f6167ecdf7a1a7fa769de4de` | BSD-3-Clause LICENSE | MoveIt2 plugin. Pinned installation guide names Humble binaries; local availability/build not tested. Standard IK service is usable from Python, but custom C++ cost callbacks are not transmitted by a generic pose-only request. |
| pantor/ruckig | `aa876200e48ce63f746472c32cce172acaeb0ba3` | MIT root license; bundled third-party licenses separate | C++ and documented Python usage. Local Python3.10 build not tested. Joint time generation is not contact geometry planning. |

Repository URLs are `https://github.com/<repository>` with the names above.
Reproduce exact contents with `/tree/<commit>` or raw URLs recorded in the JSON
evidence, not an unpinned checkout.

An additional official upstream TRAC-IK branch inventory was retrieved from
`https://api.github.com/repos/traclabs/trac_ik/branches`; rolling was pinned at
`90162ac2ecc6ea8f88c6e99df6ee01efd217a3fb`. Its sampled library README includes
old ROS1 text. This is evidence of an upstream alternative, not a tested Humble
build. Do not equate the HIRO fork, upstream rolling, and Humble releases.

## Interface Findings

Descartes Light exposes `WaypointSampler::sample()` for finite joint candidates
with initial costs, and `EdgeEvaluator::evaluate(start,end)` for validity and
cost. For this task, those callbacks would have to preserve the Euclidean TCP
tube, physical-center tangent branch, model margin and interpolation checks.
Finite sampled connectivity does not prove continuous-space feasibility.

Pick IK exposes local gradient-based and global evolutionary modes. Its pinned
usage guide describes displacement/centering objectives and C++ `IKCostFn`.
For contact, raw pose thresholds or approximate IK must never replace our final
FK gates. The guide's prose for `stop_optimization_on_valid_solution` is
internally confusing; inspect the implementation and test the exact chosen
configuration instead of copying that sentence into a production assumption.

The current scoped planner calls MoveIt IK through `GetPositionIK`, then its
own validators. Switching plugin YAML does not automatically implement a
3 mm spherical tube, the one-branch XY tangent constraint, or whole-path
connectivity. These need explicit residuals/candidate construction and tests.

## Original Papers Read

### MoveIt Task Constructor

Michael Gorner, Robert Haschke, Helge Ritter, Jianwei Zhang, "MoveIt! Task
Constructor for Task-Level Motion Planning", ICRA 2019. Read sections III-IV
and the demonstrations from the repository mirror:
`https://noah.nrw/ubbihs/download/pdf/5137431`.

Local valid file: `research_v1/mtc_icra2019_repository.pdf` and extracted text.
The earlier `mtc_icra2019.pdf` is an HTML bot response, NOT a valid paper.

The paper propagates solution hypotheses through stages in both directions,
uses containers/scheduling, and propagates failure when a partial solution
cannot be extended. This directly motivates checking the exit and return path
before accepting a contact branch. The pouring demonstration's 15.6 s mean
first-solution time is not a G1 benchmark, an end-to-end 180 s bound, or a 95%
claim. Our callback implementation borrows the stage-search principle only.

### TrajOpt

John Schulman et al., "Motion planning with sequential convex optimization
and convex collision checking", IJRR 2014.
`https://rll.berkeley.edu/~sachin/papers/Schulman-IJRR2014.pdf`.

Local files: `research_v1/trajopt_ijrr2014.pdf` and `.txt`. Text extraction
repaired an xref warning; the PDF content and tables were inspected.

The work optimizes a whole trajectory using sequential convex approximations.
Table I reports 0.843 single-initialization and 0.990 multi-initialization
success on 198 PR2 arm problems; Table II gives 0.73/0.88 on 96 full-body
problems. These tasks, models, budgets and constraints differ from this G1
dataset. Section IX discusses initialization dependence and nonconvexity.
Continuous self-collision checking was not implemented in that evaluation;
discrete self checks were used. It cannot justify removing our existing checks
or transferring the published success rate.

### Ruckig

Lars Berscheid and Torsten Kroger, "Jerk-limited Real-time Trajectory
Generation with Arbitrary Target States", 2021.
`https://arxiv.org/pdf/2105.04830`.

Local files: `research_v1/ruckig_2105.04830.pdf` and `.txt`.

This is third-order, synchronized joint trajectory generation, not redundant
IK or contact-path discovery. The reported 7-DoF mean calculation time of
19.8 microseconds is on the paper's hardware and input distribution, not whole
task planning. Random-test robustness is qualified by valid input conditions
and numerical ranges. Any new time law used here must preserve boundary stops,
hand sequencing, velocity/acceleration/jerk and waist checks. If interpolation
changes the geometric curve, repeat dense FK/tangent/collision validation.

No claim is made to have read original papers for every repository listed
above. Repository inspection, paper evidence and G1 experiments are separate.

## Controlled Experiments Still Required

Use frozen pair/direction tasks, not the easiest historic context. The present
scene grouping is a calibration proxy and the manifest was frozen after early
four-case development, so current results are descriptive, not an independent
holdout. Preserve all blocked/unresolved retained pairs in coverage reporting.

1. Repeat complete comparisons at unchanged hard gates, including preparation,
   cold startup, all retries, seven stages, retiming, dense validation, export,
   integrity verification and actual archive reload within one 180 s deadline.
2. Compare exit-feedback enabled/disabled on the same tasks and process layout;
   repeat the latest failures and right sweeps. Do not pool lucky runs.
3. Test deterministic local IK only as a candidate generator against the exact
   existing FK/model, with a separate bounded multi-start policy. Compare
   accepted residuals, margins, unique branches, total service time and failure
   progress. Fix/report seeds and solver versions; deadlines can still affect
   reproducibility under load.
4. Test one-factor search changes: batch width, branch width, seed diversity,
   candidate time slices, cache scope and coarse-to-fine samples. Limit the
   experiment count in advance; retain full dense final validation.
5. Active position-tolerance search can sample inside the 3 mm Euclidean ball,
   but success must be measured against the original nominal path, not the
   shifted target. Start with fractions 0, 0.25, 0.5 as diagnostics; default is
   unchanged. Do not alter the physical pivot, calibrated TCP or hand posture.
6. Study scoring weights separately from hard constraints. Do not soften the
   5-degree physical tangent, fixed branch, tilt/twist/step policies, 0.12 rad
   joint step, or 0.02 rad joint margin without a separately approved contract.
7. Verify peak sampled CPU/RSS and per-case wall time under each worker count.
   Lower-bound process samples are not precise peak memory/PSS or full CPU
   accounting. Queue wait is reported separately and never divided into a
   claimed per-case planning latency.

All actual archive passes must remain scoped offline passes. Neither the
current validator nor these papers certify full environmental collision
coverage, physical contact tracking, or safe hardware execution.
