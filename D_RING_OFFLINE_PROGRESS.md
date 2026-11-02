# D-ring offline planning progress

Local evidence recorded on 2026-09-18 (Asia/Shanghai).
Task definition: `/home/syf/G1_D_RING_OFFLINE_PLANNING_HANDOFF.md`.

## Status

Current user acceptance scope (2026-09-18): the input-ready, verifiable cohort,
96/99 pairs (96.97%), meets the requested 95% threshold. The 238 evidence-blocked
historical pairs remain a coverage limitation, not runnable failures and not
assumed successes. The previous all-history objective below is retained as an
audit statement, not the current panel-integration acceptance denominator.

The >=95% all-valid-history, <=180-second complete-task objective is NOT met.
The final task denominator, physical scene grouping, and independent validation
split are unresolved. All results below are scoped offline planning, not full
environmental collision coverage, contact tracking, or hardware validation.
No controller, real panel, ARM, hardware gateway, state bridge, shadow executor,
or hand/robot command was started in this work.

### Panel Integration, 2026-09-18

- Fresh D-ring planning now selects bounded IK, OMPL return routing and diverse
  preinsert scheduling, after checking the native backend. Prepare/recover and
  center checks retain their existing solver selection. Final validators and
  collision scope are unchanged. Earlier statements that live defaults were
  unchanged describe the benchmark checkpoint, not this subsequent integration.
- Saved-library preview defaults to a fresh RGB-D capture. Capture-time measured
  FK freezes the scene in pelvis coordinates; archive model and calibration must
  match. Target displacement is recorded, with no path transformation and no
  claim of current-scene collision validation or execution authorization.
- Complete fresh plans are automatically published under
  `data/offline_trajectory_library/live/`, including captured RGB-D, measured
  state, model/mesh snapshots and checksums. Publication is atomic and passes
  archive reload before it becomes visible. The stored trajectory is the
  validated baseline, before any arbitrary preview multiplier is applied.
- Final user scope: `RViz speed (x)` accepts a positive finite custom number,
  not a fixed set of choices or the earlier 0.05-32 range. The requested value
  is applied to all seven preview stages, including hands, without clamping
  to software velocity/acceleration/jerk caps. Timestamp representability and
  a 1e-7 rad maximum interpolation-path difference are still checked. Per-joint
  peak velocity/acceleration/jerk are recorded. This is NOT hardware timing
  approval. Hardware-range loading keeps its separate existing limit checks.
- `Apply to planned path` retimes a completed live D-ring path without IK,
  preserving the original plan files and capture age. Speed changes revoke UI
  confirmations. Arbitrarily retimed live previews are explicitly denied
  hardware authorization. Gallery speed edits always reference the original
  baseline, so repeated speed changes do not compound.
- Initial guarded-mode comparison (superseded for RViz): left sweep
  `2ca166b9c0876d6596e7` applies 1.478196x, 717.434 -> 486.638 s; right sweep
  `d42a74b2e4491e7187d4` applies approximately 1x, 640.490 -> 640.490 s.
  Maximum normalized polynomial differences were 3.71e-9 and 2.84e-9.
- Final unrestricted-preview checks: both examples use the LEFT ARM, sweeping
  left/right respectively. At 24x their complete seven-stage durations are
  29.893103/26.687091 s, including withdrawal (3.74125/1.96049 s), hand changes
  and return. At 73.125x they are 9.811070/8.758840 s. Maximum joint-path
  differences are below 2.5e-9 rad. Original archive hashes are unchanged.
- Scene-overlay and automatic-publication integration use recorded captures explicitly as offline
  fixtures; a fresh physical-camera round trip and hardware execution have not
  been tested in this change. Frozen benchmark inputs and archives are untouched.
  No 5 mm tolerance experiment was performed and geometry gates were not relaxed.

Best recorded complete full runnable-cohort run:
`all_replayable_diverse_preinsert_parallel6_v1` passed 108/111 runnable direction
tasks (97.30%), with two failures and one timeout. All 108 passes completed
planning, validation, export and reload within 24.493-126.130 s. Complete
required-direction pair coverage is 96/337 (28.49%), not 95%; there are still
238 blocked pairs. The input-ready subset alone has 96/99 passing pairs.
All 100 passes from the earlier bounded/OMPL full run remained passes, seven
old timeouts recovered, and the already startup-repaired task passed. No runs
are pooled and no tolerance, denominator or live default was relaxed/changed.

The earlier `all_replayable_ompl_return_parallel6_v1` passed 100/111 directions
(90.09%), with three failures and eight timeouts, covering 88/337 pairs.
Its 109 unchanged common tasks improved from 86 to 99 passes, with fourteen
former timeouts recovered and one startup regression.

A later startup-dependency fix passed a separate eight-direction regression,
including the previously unavailable timing service case. All eight completed
validation/export/reload within 180 s. This does not replace the full-run
failure or raise its numerator. Early DDS response delivery remains a separate
observed risk; constructor readiness is not a guarantee of response delivery.

The newer opt-in `all_replayable_constrained_parallel6_v1` run passed only
97/111 direction tasks (87.39%), with fourteen failures and no timeouts.
Required-direction pair coverage was 85/337 (25.22%). Four former passes
regressed, eight former timeouts failed before their deadlines, and the earlier
startup-service failure passed. That startup case had already passed a separate
bounded-backend regression after the native fix; it is not evidence of a new
constrained-solver benefit. The experiment is not promoted to default and is
not merged with earlier results. The later diversity experiment uses the
bounded backend, not the regressing constrained backend.

## Frozen Evaluation

- Original inventory: 420 unique RGB-D pairs, unchanged. Inventory SHA256:
  `458602265893bb0a3eb7b8e1b513d03bc02485e24fb97a6fa0cc633b8ea4f92a`.
- Original frozen evaluation used by the full batches:
  `data/offline_history/20260917/evaluation.v1.json`, SHA256
  `437643313d70a744caa2b6415e321a279f90a265f902cad1467f3165d42472c3`.
- 83 visually no-target pairs with no recorded task evidence are excluded from
  diagnostic coverage. 337 are retained: 324 visibly contain a ring and 13
  remain ambiguous/occluded. Detector failure alone never excludes a pair.
- 98 pairs are input-ready; 239 are blocked. There are 111 required explicit
  direction tasks, of which 109 are runnable. A pair passes only when ALL its
  required directions pass; unknown intent blocks pair success.
- Blocked does not mean all robot states are missing: 157 blocked pairs have
  capture state plus geometry; 55 have contexts without those files; 27 have
  no context. Other blockers include task intent, direction, historical
  ready/home/calibration, or missing task evidence.
- Twenty center-check contexts were distinguished from D-ring sweeps; fifteen
  legacy dual-sweep contexts require both directions. Fourteen task contexts
  still have unresolved intent/direction.
- Byte-exact original-manifest recovery is available for 180 intrinsics files,
  9 annotations, and 9 clouds. `history_recovery.json` records copied evidence.
  Original captures are never edited, and no current/synthetic state replaces
  missing historical state.
- Seven of twenty formerly ambiguous images were resolved as ring-visible at
  full resolution (indices 4, 32, 166, 190, 218, 388, 417). The thirteen remaining
  uncertain images stay in coverage. See `visual_review.refinement.v1.json`.
- All retained pairs share an unregistered calibration-derived scene proxy.
  It is NOT a verified physical scene group or an independent holdout split.

`input_readiness_audit.v2.json` further separates the 239 blocked pairs: 120
have state and geometry files but no stored target points, 82 lack a context
containing both state and geometry, 36 have a valid geometric seed with other
blockers, and one has a rejected geometric seed. Of the 120 targetless records,
83 already have a valid raw historical 3D detector result and 37 do not. Only
17 blocked pairs are otherwise input-complete after removing applicability and
direction blockers. Presence of `geometry.json` is not proof of replayability.

`calibration_receipt_audit.v1.json` checks original camera B/C session metadata
and receipt windows that the original corpus artifact allowlist did not copy.
The 48 receipt directories are duplicate capture/session copies for 24 unique
retained pairs. All 24 pass source/sample checksums and exact RGB-D, camera and
capture identity checks. Re-running the existing full-window static/provenance
validator agrees with all 24 historical decisions: 19 stationary software
associations pass; five fail (one missing publisher GID, one clock discontinuity,
three stale/mismatched discovery observations). The valid windows have only 15
arm/waist joint readings, no measured hand state, and no valid raw D-ring 3D
target. They do not establish task arm/direction or exposure synchronization.
Zero complete task inputs were recovered from this evidence; no denominator
or original capture was changed. Reproducer: `audit_calibration_receipts.py`.

A separate append-only dataset view, `recorded_ready_recovery_v1/`, recovers the
eight arm/waist goal values from the explicit terminal `Return ready` trajectory
in `20260911_113650_819948/plan_0010/plan.json`. The full saved legacy dual-sweep
sequence, arm, joint ownership, finite values, strictly increasing time and
stationary endpoint are checked. This is a RECORDED PLANNED GOAL, not a newly
measured ready state. It applies only to this context; no neighboring/current
state is substituted. Capture state, hand state, geometry and calibration stay
byte-identical. All 7,801 original files are hard-linked in the separate view;
only the ready record and provenance are added, never overwritten in place.
Do not rebuild the original corpus from a broad recursive data scan.

The derived `recorded_ready_recovery_v1/evaluation.v2.json` has SHA256
`5f1fce196778b0198c74bfd9d5fa172014fd4698e94422b5e3ea0ab1560d368f`.
It retains the same 337-pair coverage denominator and 111 required directions,
with 99 input-ready pairs / 238 blocked and 111 runnable direction tasks.
This does not revise any v1 full-batch numerator. Reproducer:
`recover_recorded_ready.py`; audit/recovery tests: 11 passed.
`recorded_ready_bounded_parallel2_v1` then attempted both directions on domains
98-99: right passed all seven stages, export and reload in 50.368 s; left timed
out during continuous rotate search at 180 s. Both code audits passed. This
is one successful direction, NOT an additional complete pair. The later
111-task derived-v2 full run is reported separately below.

`input_readiness_derived_v2.json` confirms the remaining 238 blocked pairs:
120 targetless, 82 without a context containing state plus geometry, 35 valid
geometric seeds with other blockers, and one rejected seed. The separate
`targetless_intent_audit.v1.json` rereads original plan/failure records for the
83 targetless pairs with valid raw 3D points: 73 are `prepare`, ten `recover`,
and zero record a sweep direction. All 83 retain a camera transform, but none
has a full geometry calibration snapshot. Existing transforms/points do not
by themselves establish a historical D-ring task contract. No geometry, state,
intent or calibration was substituted, and zero complete task inputs were
recovered. Reproducer: `audit_targetless_intent.py`.

`unknown_task_intent_audit.v1.json` then checked all fourteen unknown task
contexts, their original local capture metadata, and immediate session JSON
records. Frozen input hashes match; no original metadata differs from its
dataset copy. No explicit `sweep_direction`, `task_schema`, or
`requested_directions` field was found, including nested records. Only TWO
of these unknown tasks are otherwise input-complete. The seventeen otherwise
input-complete blocked pairs are four prepare tasks, eleven center checks and
two unknown tasks, not seventeen recoverable sweeps. The two unknown records
say `Operation cancelled or superseded`. Zero complete inputs were recovered;
the 238 blocked pairs and 337 denominator remain unchanged. Reproducer:
`audit_unknown_task_intent.py`; arm side and partial failed-stage labels are
not promoted to a complete requested sweep.

An isolated CPU perception diagnostic used the bundled two-stage models,
Ultralytics 8.3.102, PyTorch 2.8.0+cpu, two single-thread workers, and a selection
frozen before inference. `perception_recovery_probe_v1` checked 24 blocked
ring-visible pairs plus eight ready and eight no-target controls in 9.80 s.
Five blocked cases produced three camera points, but ALL five already had a
valid historical 3D result. There were zero newly recovered invalid-3D cases.
The local checkpoint missed all eight ready controls; all eight negative
controls remained negative. This diagnostic therefore does NOT establish a
usable recovery detector. Model hashes, software versions and annotated images
are saved; no new inference output became a planning input. The dedicated
`.venv-vision-offline` does not replace ROS or system Python packages.

## Completed Full Runnable Batches

All four batches attempted the same 109 runnable direction tasks, with a strict
per-case 180 s supervisor including input preparation, startup, planning, all
retries, validation, archive export and actual reload. Queue wait and process
cleanup are separate. No pass is assembled from retries across different runs.

| Batch | Workers / domains | Archive passes | Failed | Timeout | Complete pairs / retained | Batch wall |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `all_replayable_parallel7_v1` | 7 / 93-99 | 27 | 67 | 15 | 27/337 = 8.01% | 957.2 s |
| `all_replayable_parallel6_v2` | 6 / 94-99 | 40 | 36 | 33 | 32/337 = 9.50% | 2291.7 s |
| `all_replayable_bounded_parallel6_v1` | 6 / 94-99 | 84 | 5 | 20 | 73/337 = 21.66% | 1354.5 s |
| `all_replayable_bounded_parallel6_v2` | 6 / 94-99 | 86 | 2 | 21 | 75/337 = 22.26% | 1342.8 s |

These percentages are conservative diagnostic coverage, NOT the requested
final all-task success metric. `full_task_rate` remains null. Runnable-task
success was 40/109 (36.70%) in MoveIt v2, 84/109 (77.06%) in bounded v1 and
86/109 (78.90%) in bounded v2. These are descriptive subset results, not
independent holdouts.

The first bounded batch has 59 left and 25 right passes, all 109 code-integrity audits
passed, and accepted complete-case times were 21.54-154.83 s. Failures were
20 deadlines, three complete-candidate path failures and two uncaught TOTG
failures. No input/startup/preinsert-IK/RPC-timeout failures were recorded.
The five development regression tasks all passed again within this single run:
older right plan0046 24.66 s, earlier left plan0018 36.94 s, latest plan0013
28.00 s, earlier right plan0020 81.52 s, and latest plan0011 39.75 s.

The first bounded run overlapped a short isolated RViz test, vision-environment
installation and a 9.80-second two-worker perception diagnostic. This changes
host load; do not attribute timing differences solely to one algorithm change.

The post-retimer/resource-fix bounded v2 run has 61 left and 25 right passes,
21 deadlines and two complete-candidate rotate failures. All 109 code-integrity
audits passed; accepted complete cases took 21.64-150.16 s, median 35.86 s.
All 84 bounded-v1 successful tasks passed again. The two former TOTG failures
passed in 74.148 s and 26.560 s, each retaining recorded TOTG failures followed
by candidate recovery. The former left return-path failure instead exhausted
the deadline; the previous 20 deadlines remained deadlines. No successes from
other batches were added. See `bounded_full_v1_v2_comparison.json`.

All five development regressions passed again: older right plan0046 26.178 s,
earlier left plan0018 35.591 s, latest plan0013 26.829 s, earlier right plan0020
81.515 s, latest plan0011 37.234 s. Maximum sampled worker-group RSS was
1,512,194,048 bytes; summed sampled CPU was 6,782.91 s, both lower-bound samples,
not simultaneous host peak or PSS. Service readiness was 8.52-35.83 s.
Only lightweight local evidence audits/preparation overlapped this run; no
auxiliary RViz or perception inference ran during it. This is still not a
controlled causal timing comparison.

Initial MoveIt v1 failures included 46 unrecognized legacy geometry metadata cases and 11
startup readiness failures. MoveIt v2 derives legacy offsets only when the full saved
arc proves the interpretation to 1e-9 m, without moving geometry or TCP. All 109
selected inputs passed the resulting pure input gate. A shared 60 s readiness
allowance remains INSIDE the unchanged whole-case deadline.

MoveIt v2 failures: 33 deadline, 15 preinsert IK, 14 complete-candidate path failures,
7 RPC timeouts. `APPROACH_PATH_FAILED` is a misleading historic aggregate label:
its nested failures can be insert/rotate/withdraw/return failures, not just
approach motion. There were no v2 input/startup failures. All 109 code-integrity
checks passed. Slowest accepted case: 179.7916 s.

MoveIt v2 direction passes: left 29, right 11. The fixed required-direction totals are
74 left and 37 right (including blocked tasks); report these separately from
the 109 runnable-task count. Recorded historical 1970 dates are retained as
metadata and are not treated as verified acquisition dates.

Latest regression pair contexts in MoveIt v2:

- `20260917_200510_758963/plan_0013`: archive reload pass, 137.24 s.
- `20260917_200510_758963/plan_0011`: timeout, not a pass.

Host: 24 available CPUs, 16,619,999,232 bytes physical memory. Numerical libraries
are limited to one thread per worker. MoveIt v2 maximum sampled single-worker process
group RSS was 1,477,914,624 bytes; aggregate sampled CPU was 12,638.84 s. These
are sampled lower bounds, summed RSS is not PSS, and a per-worker peak is not
a measured simultaneous whole-batch peak. Service readiness ranged 8.76-36.07 s.
For bounded v1, maximum sampled single-worker group RSS was 1,554,190,336 bytes
and summed sampled CPU was 6,768.77 s, with the same measurement limitations.

### Full Derived-V2 OMPL Return Run

`all_replayable_ompl_return_parallel6_v1` used six exclusive domains 94-99,
the unchanged `bounded` IK and opt-in scoped OMPL return fallback. All 111
directions for 99 input-ready pairs were attempted. Results: 100 complete
archive reload passes, three failures and eight timeouts; batch wall 1092.956 s.
Every code-integrity audit passed, including C++ source and compiled adapter.
Accepted complete-case times were 23.216-134.572 s (median 38.394 s), including
input preparation, startup, all retries, validation, export and reload.
There were 67 left and 33 right passes. Pair coverage is 88/337 = 26.11%; the
unresolved final all-task success rate remains null and is not claim-eligible.

`bounded_v2_ompl_return_full_comparison.json` verifies identical source hashes
and semantics for the 109 common tasks. Of the previous 86 successes, 85
repeated and one failed at startup. Fourteen old timeouts passed, seven remained
timeouts, and both hard right-sweep candidate failures remained failures. Thus
the common cohort is 99/109, not a pooled 100/109. The two appended ready-goal
directions are separate: right passed in 24.979 s; left timed out. Reproducer:
`compare_derived_complete_batches.py`, with five cohort-integrity tests.

All fourteen newly passing common tasks record an accepted OMPL return.
Forty-seven successful archives use the fallback. One additional recorded
OMPL candidate failed the ORIGINAL final dense collision check (ring finger
against axis base); the same case later found an accepted branch. Approximate
or coarse-edge-only paths were never accepted. The known return case left
passed again in 23.676 s, independently of its 61.528 s development pass.

The startup regression is `20260911_135303_399867/plan_0011` right:
`/g1/task/time_path` was unavailable within readiness budget, at 71.720 s
whole-case time. Its planner-server log was still waiting for
`/apply_planning_scene`; this is not evidence of an IK/geometric failure.
It remains a failure in this batch, with no posthoc replacement.
Maximum sampled worker-group RSS was 1,470,746,624 bytes and summed sampled
CPU 4,662.40 s; these remain lower-bound samples, not whole-host peak/PSS.
Only lightweight evidence audits overlapped this run. Auxiliary reuse, RViz
and the larger unit-test batches ran before or after it. Scheduling still
precludes a clean causal timing claim.

## Planner Changes And Development Ablation

Return-ready and hand-open validation now participate in branch acceptance.
A failed exit can resume earlier contact generators. Deadlines kill the whole
offline worker process group and never extend acceptance by a cleanup grace
period. Hard gates remain 3 mm Euclidean TCP, 5-degree physical-center XY
tangency with fixed branch (4.9-degree candidate margin), existing tilt/twist/
orientation-step limits, 0.12 rad joint step and 0.02 rad model margin.
Straight-index TCP, specified hand posture, PD gains and collision scope are
unchanged.

An explicit OFFLINE experiment is now available as `--ik-backend bounded`.
Default remains `moveit`. It uses SciPy bounded least squares with an analytic
URDF Jacobian, deterministic existing seeds, and a 0.021 rad internal interval.
Candidate generation still goes through the original validators. It does not
soften hard constraints, synthesize state, or modify live-panel defaults.

The Jacobian was checked against finite differences, including TCP offsets and
mimic joints. Cancellation propagates, local candidate timeouts fail closed,
passive state remains unchanged, and a collision-free pose alone earns no
task success. The candidate-only probe produced 28 pose solutions / 64 trials,
17 passing scoped collision, in about 1 s; it DID NOT check the complete tangent
policy or whole path and is not part of any success numerator.

`bounded_ik_parallel5_v1`: five preselected development tasks, five workers,
unchanged constraints and archive gate. Four passes: latest plan0011 57.89 s,
latest plan0013 51.66 s, earlier left plan0018 53.78 s, earlier right plan0020
75.58 s. The right-sweep `20260912_155322_095139/plan_0046` failed preinsert.
This is one small development run, not repeatability or generalization evidence.
A same-layout MoveIt control completed with 1/5 passes: latest plan0013 took
125.23 s, the older preinsert case failed, and three cases timed out. Concurrent
auxiliary validation affected host load, so this is not clean causal latency
evidence.

Saved inclined preinsert rays can have different XY bearings at preinsert and
entry. Candidate generation now corrects the bearing at the actual trial point,
and establishes the fixed tangent branch there. It preserves the original
quaternion exactly when that trial already meets the 4.9-degree candidate gate.
Degenerate projections are rejected; final FK and dense path gates are unchanged.

`bounded_ik_parallel5_v2` unconditionally realigned candidates. It fixed the
older preinsert failure but regressed earlier right plan0020 to a timeout (4/5).
The preservation fix was then tested in `bounded_ik_parallel5_v3`: 5/5 complete
archive reload passes, all code-integrity audits passed, batch wall 86.47 s:

| Context | Direction | Complete case seconds |
| --- | --- | ---: |
| `20260912_155322_095139/plan_0046` | right | 49.575 |
| `20260917_191654_927130/plan_0018` | left | 52.385 |
| `20260917_200510_758963/plan_0013` | left | 45.648 |
| `20260917_191654_927130/plan_0020` | right | 86.228 |
| `20260917_200510_758963/plan_0011` | left | 57.083 |

These are development cases, not five independent held-out scenes. Earlier
failures remain recorded; passes from different batches are never pooled.
The completed full bounded run above provides another pass for each of these
five tasks, but broader repeated-run success remains unproven.

After that frozen batch, known TOTG/Ruckig algorithm failures were converted to
recoverable candidate failures. Invalid requests, model contract errors, STOP
and timeouts still propagate and are never waived. Previously these numerical
retiming failures raised a generic exception and aborted the whole case.
`bounded_retime_recovery_parallel5_v1` then passed 5/5: former TOTG failures
`20260917_181601_998171/plan_0015` 67.317 s and
`20260917_182341_096779/plan_0011` 53.149 s, latest plan0013 48.549 s, earlier
right plan0020 89.798 s, and latest plan0011 58.177 s. Both former failures
recorded another TOTG failure and then completed through branch recovery.
This is a separate development batch, not an 86/109 revised full-run score.

`position_search_half_parallel4_v1` is a preselected single-factor diagnostic:
only `position_search_fraction` changes from 0 to 0.5, giving 1.5 mm axis offsets
when nominal continuation has no accepted solution. All original 3 mm and
orientation/joint/collision gates remain. Default configuration is unchanged,
verified byte-identical to the full-batch baseline. Two known hard right sweeps
(`20260915_160757_112051/plan_0052`, `20260915_144425_432725/plan_0080`) both
timed out at 180 s. Regression controls latest left plan0011 and earlier right
plan0020 passed in 61.638 s and 174.531 s. All four code audits passed.
The result is 2/4 with no newly repaired hard case, so this setting was NOT
adopted. Four workers on domains 94-97 overlapped the separate two-worker
ready-recovery test on 98-99; timings are not a matched causal comparison.
Selection, sole effective parameter diff, hashes and reproducer are under
`position_search_half_selection_v1/` and `prepare_position_search_ablation.py`.
Successful experimental archives remain in their batch output, not the default
published library. They are not added to either full-batch numerator.

### Continuation-Tolerance Diagnostic

`--ik-backend bounded-task` is a separate opt-in offline experiment. Contact IK
uses the actual previous state, not the perturbed seed, to intersect the guarded
model interval with +/-0.119 rad. Its candidate-only residual thresholds are
2 mm and 2 degrees; preinsert stays at the strict bounded thresholds. All
original FK/tangent/tilt/twist/step, collision and dense trajectory gates remain.
There is no added seed regularizer. The default `moveit`, original `bounded`,
and YAML settings are unchanged. Real-model, passive-state and gate tests pass.

`bounded_task_parallel6_v1` tested two hard right sweeps and four regressions on
domains 94-99: 3 passes, 2 failures, 1 timeout; all six code audits passed.
Earlier left plan0018 passed in 69.710 s, latest plan0013 in 57.061 s and latest
plan0011 in 52.986 s. Hard right plan0052 and plan0080 failed in 179.028 s and
171.503 s: all 16 complete candidates in each ended at orientation rejection,
mostly the unchanged physical-center tangent gate. Earlier right plan0020,
previously successful with `bounded`, timed out. Batch wall was 180.263 s.
This backend was NOT adopted; none of its passes changes a full-run numerator.

### Scoped OMPL Return Experiment

`--joint-route-backend ompl` adds an opt-in return-only fallback. The unchanged
joint-line path is tried first; only a `return_ready` collision triggers OMPL.
Approach/contact/preparation/recovery and default live behavior are unchanged.
It links the already-installed OMPL 1.7.0 using a small pybind11 adapter; no
system package or alternative robot model was installed. Exact endpoint paths
only, local seed 19, 0.25 rad L2 expansion, 0.02 rad candidate-edge L2 resolution,
0.021 rad internal model margin and a 4 s local budget. The existing scoped
validator is the state predicate; no new collision scope is introduced.
Shortcut candidates are checked by the existing batch validator, then each
retained joint line uses the existing quintic curve with stationary boundaries.
Final retiming/jerk/waist/joint/dense collision checks, export and reload remain
mandatory. Cancellation/contract errors propagate; approximate paths are rejected.

`ompl_return_parallel6_v1` passed 6/6 complete archives with all code audits:
the old return-collision case `20260910_205306_042305/plan_0008` left 61.528 s
and right 62.340 s, earlier left plan0018 56.352 s, earlier right plan0020
97.447 s, latest plan0013 61.323 s and latest plan0011 61.161 s. The repaired
left case records the original shoulder/torso straight-path collision, then
349 state checks, eight shortcut tests and an accepted three-waypoint route.
Return search took 0.080 s; all 6,120 final return samples passed. Maximum
contact error was 0.649 mm and physical-center tangent error 4.8773 degrees.
This is development evidence, not a full-cohort score or a new 95% claim.

161 focused tests passed before this frozen subset, including native OMPL
repeatability, exact endpoints, rejection of approximate paths, cancellation,
unchanged default routing, piecewise quintic interpolation and archive checks.
Frozen benchmark snapshots and archive provenance now include `.cpp` and `.so`
hashes as well as Python/shell sources. Never rebuild the adapter during a run.

## Trajectory Library And Playback

The 40 v2 successes were copied byte-identically and reloaded into
`data/offline_trajectory_library/all_replayable_parallel6_v2/`.
`publication.json` records source benchmark hash, archive hashes and ORIGINAL
complete-case times. Publishing does not reset planning deadline credit.
The 84 first full bounded successes and five retiming-regression successes were
also published under their own batch directories, producing 130 archives.
The 86 bounded-v2 successes and one ready-recovered right sweep have now been
published and independently reloaded as byte-identical copies. That catalog
checkpoint found 217 archives (154 left / 63 right), all seven-stage, zero
errors. These include repeated task contexts from separate runs and are NOT
217 unique successful tasks. The older
standalone plan0011 archive is a separate posthoc export and is not added to
any original benchmark numerator.

Six OMPL development archives and all 100 OMPL full-run successes were then
published as byte-identical copies under their distinct batch directories.
`ompl_full_delivery_audit.v1.json` independently reloaded the entire default
library: 323 seven-stage archives (225 left / 98 right), zero errors. This is
not 323 unique successful tasks. Its separate stored-summary audit of the
100 full-run reports found maximum TCP path error 2.208934 mm, endpoint error
below 1e-6 mm, tangent error 4.999385 degrees, tilt 4.987606 degrees and twist
9.903688 degrees. Minimum clearance beyond the existing 0.02 rad model margin
was 0.000965069 rad. Reports record 5,083,335 dense arm samples. These are audits
of stored validator evidence plus fresh archive reloads, NOT another independent
FK sweep or additional task-success credit. At that checkpoint code matched the
full run's frozen Python/shell/C++/binary snapshot. The startup changes below have
their own later snapshot and do not retroactively change that evidence.

The original panel has library selection and isolated playback controls.
Catalog hashing runs in a background worker rather than blocking feedback/Qt.
Selecting an archive resets confirmations and never loads or executes it.
Existing start/hand/model/calibration/scene and explicit execution gates remain.
Panel tests use a fake node; the real panel was NOT launched.

Archive mesh URIs are relocated to verified immutable snapshots in playback and
temporary collision-model copies. The source model/hash stays unchanged.
When loading against a live model, current mesh bytes must also match archived
hashes; matching URDF text alone is insufficient. Legacy archives without mesh
snapshots retain their explicit external-resource behavior, not a portability
guarantee. URI parsing supports encoded paths, including relocated directories
with spaces, and rejects unsupported remote mesh schemes. Multi-archive playback
requires both identical original model and mesh hashes. The gallery uses only
localhost domains 93-99 with exclusive locks and publishes simulated states.

RViz diagnostics proved full TF connectivity and revealed two test defects:
an early screenshot preceded rendering, and the modified-title `*` defeated
exact window matching. Viewport-only checks, stage state/TF checks and image
change evidence replaced whole-window standard deviation.

`rviz_archive_stage_smoke_v4` rendered all seven stages with 572 observed joint
messages; viewport change was 8.58%, but return-hand framing was clipped.
v5 fixes framing; later stages show the full robot/hand and point cloud, but
its first stage captured only the cloud before robot mesh readiness. Its
automatic `passed=true` therefore does NOT prove every stage rendered the
robot. v6 adds a robot-only readiness phase before publishing cloud/markers:
all seven stages passed with 410 observed joint messages, matching published
states and required TF frames. Robot-only foreground was 4.64%, scene addition
changed 6.92% of viewport pixels, and stage change was 4.32%. Robot-only, first,
and final-stage screenshots were visually inspected for meshes and full framing.

The newly published right-sweep archive also passed
`rviz_published_bounded_right_v2`: seven stages, 265 observed joint messages,
all stage state/TF checks true, robot-only foreground 4.59%, scene-added change
6.98%, and inter-stage viewport change 3.54%. Approach, rotate and final-stage
screenshots were visually inspected; meshes, cloud and full framing are present.

`rviz_ompl_return_full_v1` tested the newly published full-run repaired left
archive on domain 93: all seven stages, 191 observed joint messages, exact
stage state/TF matches and no smoke errors. Robot-only foreground was 5.01%,
scene addition changed 4.56% of viewport pixels, and stage change was 3.97%.
Approach, rotate and return screenshots were visually inspected: full robot,
hand meshes and cloud are present without clipping. Stage selection deliberately
seeks between saved poses; its observed joint-message jump is NOT a trajectory
continuity or execution measurement. The viewer sent no hardware commands.

Independent reuse tests exercise `prepare_range` at several speed/acceleration
settings using archived state. They must preserve the immutable input, original
positions, normalized interpolation curve, hand timing and full existing
velocity/acceleration/jerk/waist/scoped collision checks. They neither authorize
execution nor contribute new whole-case planning passes.

`library_retime_parallel3_v1` completed 9/9 checks: three bounded-v1 archives
at velocity/acceleration factors (0.2,0.2), (0.4,0.5), and (0.8,0.9), all seven
stages, three CPU workers without ROS services. Original positions and hand
timing remained unchanged; normalized polynomial coefficient difference was
at most 4.30e-9. Batch wall was 143.66 s, individual validation 18.77-75.74 s.
Execution durations are separate and substantially longer than planning time.
That first validation preceded the mesh fix and depended on unchanged external
assets, so it alone did not establish portable collision validation.

After the fix, `library_retime_snapshot_parallel3_v2` passed another 9/9 checks
using three newly published full-batch archives, copied byte-identically to
directories containing spaces. All 197 mesh references in every constructed
collision URDF pointed inside its archive. All seven stages, original positions,
hand timing, existing scoped collision and motion-limit checks passed at the
same three speed/acceleration settings. The maximum normalized coefficient
difference was 4.30e-9; validation took 18.70-77.93 s per job, batch 144.95 s.
This test used only archived state and sent no hardware commands. Reuse
validation times are not execution durations or new planning deadline credit.

`ompl_return_reuse_parallel3_v1` checks the repaired development left archive
at the same three velocity/acceleration settings, relocated to a path containing
spaces. All three seven-stage jobs passed, all 197 collision mesh references
resolved inside the archive, and original joint positions and hand timing were
preserved. Maximum normalized polynomial difference was 3.9014e-9; individual
checks took 18.848-62.502 s, batch wall 62.518 s. The piecewise quintic return
remained on its immutable geometry under the existing speed/acceleration/jerk/
waist/scoped collision gates. Reproducer: `validate_ompl_return_reuse.py`.

## Native Startup Dependency

The full-run right task `20260911_135303_399867/plan_0011` failed because
`/g1/task/time_path` was unavailable. Its native planner constructor was waiting
inside an eagerly constructed `PlanningSceneInterface` for
`/apply_planning_scene`, even though both demo-object flags were false. That
member was only used by `add_demo_scene`. It is now constructed locally only
when at least one demo object is enabled. No retiming math, collision predicate,
model, gains, hand posture, execution gate or solver policy was changed.
The installed server was rebuilt; its SHA256 is
`9297c4967f1e13fb53ebd6a4c03221367439f2458763bfdb843b5c8255949fff`.
The prior source and executable remain in `lazy_scene_startup_v1/before/`.

`check_lazy_scene_startup.py` runs old/new native nodes with isolated MoveGroup,
trajectory execution disabled, no hardware nodes and no action requests. The
final v6 test withholds the ApplyPlanningScene provider for 12 s AFTER both
MoveGroup clients initialize. In all four configurations (none/table/obstacle/
both), the eight constructor-dependency tests pass. With no demo objects the
new constructor completes and advertises timing while the provider is absent;
the old one does neither. Demo objects still block constructor completion until
the provider is released. All four old/new pairs produce byte-equivalent JSON
retimed trajectories and identical demo-object service requests. Evidence:
`lazy_scene_startup_audit.v1.json` and `summarize_lazy_scene_startup.py`.

This is NOT a general DDS startup guarantee. Earlier stub/config fixture errors
are retained. The stricter v5 new/no-demo experiment also recorded twelve
unanswered early probes after constructor readiness, with native RMW logs
reporting an unmatched response reader. That negative result remains valid;
v6 separates constructor readiness, service availability and response delivery.
In v6 the new/no-demo reply arrived during the withheld window without retry,
but one later observation does not erase the v5 failure. Probe retries are
test-only; production replay retry behavior and deadlines are unchanged.

`lazy_scene_regression_parallel4_v1` passed 8/8 complete direction tasks on
domains 96-99, wall 117.806 s, individual complete-case times 24.536-102.733 s.
It covers both directions of the startup case, both OMPL repaired-return
directions, earlier plan0018/plan0020 and latest plan0011/plan0013. The formerly
unavailable right task passed in 46.898 s. These are separate regression
results, not replacements for the original 100/111 full-run outcomes.
Auxiliary startup probes used other isolated domains on the same host; do not
interpret latency differences as a clean single-factor speed improvement.
Summary SHA256: `f40c41452e3f4f4ea81ce023f051c32afc188c97ecd1a06a22ffb539d21344d5`.

Benchmarks now copy and verify a separate `runtime_snapshot` for native
planner/validator sources, binaries, replay launch and DDS profile. Fresh replay
reports and archive provenance retain the same identity; changed native code
invalidates a pass/export. Old reports without it explicitly remain unrecorded,
not retroactively attributed to today's executable. External ROS shared
libraries are not snapshotted. All eight regression runtime identities match
their snapshots, reports and archive provenance.

All eight archives were published byte-identically under the new batch name.
`lazy_scene_delivery_audit.v1.json` reloaded 331 default-library archives
(230 left / 101 right), seven stages each, zero errors. These include duplicates
across runs. Stored summaries for the eight new passes report max TCP error
0.742 mm, tangent 4.989551 degrees, tilt 4.898160 degrees, twist 9.864968 degrees,
420,175 dense arm samples and minimum margin beyond the existing 0.02 rad guard
of 0.001163074 rad. This is a stored-summary audit and fresh archive reload,
not an independent recomputation of every FK sample. No new RViz visual claim
is made for this startup-only change.

Current targeted verification: 217 planner/archive tests plus 23 evidence tests
passed (240 unique selected tests), alongside the eight native constructor
tests. The original evaluation/inventory, derived evaluation, full-run summary
and default planning YAML hashes remain unchanged.
Final cleanup found no remaining replay, benchmark, native startup fixture,
MoveIt, state publisher or RViz processes; domain locks 93-99 were all available.

## Constrained Contact Diagnostics

The ordinary bounded two-hard-case diagnostic batch
`contact_rejection_probe_parallel2_v1` completed with two failures, not passes.
New diagnostic snapshots preserve the actual previous and seed states,
waypoint/target/reference and rejected candidate joint values. Focused tests
verify copied-state ownership. Solver acceptance was unchanged in that batch.

An opt-in `constrained` backend now generates contact candidates with direct
task inequalities and analytic Jacobians. It preserves preinsert bounded IK,
all final validators, the physical pivot/TCP/posture, hard limits, defaults and
hardware prohibition. It is not a complete-path solver. The standalone 398
rejected-state local successes did not transfer to the two hard complete cases.
See `D_RING_METHOD_RESEARCH.md` for the constrained objective, derivative checks
and retained endpoint/continuation negatives.

`constrained_contact_parallel6_v1` completed 8/10 direction tasks on domains
94-99, batch wall 138.803 s. The eight controls passed in 32.683-75.691 s; both
hard right sweeps still failed (112.049/82.528 s). Each pass includes all seven
stages, retiming/dense validation, export and verified reload. This is a
separate targeted result and is not merged into any complete-cohort score.

Dense validation timing now creates a record before validation and finalizes
it on success, failure or cooperative interruption. Failed geometry/collision
and task-deadline tests preserve their original exceptions and record elapsed
time. Progress counts are last reported checkpoints, not exact work counters;
a forcibly killed process still cannot serialize its pending timing. The ten
case batch contains both passed and failed dense-validation durations, instead
of only durations for accepted validations.

`all_replayable_constrained_parallel6_v1` then ran the same complete frozen
111-direction / 99-input-ready-pair cohort on six workers, domains 94-99.
It produced 97 complete archive/reload passes and 14 failures in 1,002.722 s
batch wall time. Successful cases took 27.432-75.854 s (median 41.794 s).
The pair numerator was 85/337, with the 238 blocked pairs still retained.
All 111 code and native-runtime integrity checks passed. Comparison artifact:
`ompl_constrained_full_comparison.v1.json`; summary SHA256:
`b2660442645a73edb0fc8d9961e34a8c21aeae13331dba401a92bd74720144c3`.

Compared with the previous full cohort: 96 passes remained passes, both hard
right sweeps still failed, all eight timeouts became ordinary failures, four
passes regressed, and the already startup-repaired task passed. The regressions
are `20260911_170509_877908/plan_0041` left,
`20260911_193221_310116/plan_0077` left,
`20260911_193221_310116/plan_0088` left, and
`20260917_162759_350802/plan_0051` right. The right regression reached all contact
waypoints in one branch but still failed dense geometry/collision validation;
the left regressions had local IK/budget failures plus dense-geometry rejects.
Fewer timeouts is not higher success. This method is retained solely as an
explicit experimental option, not a default replacement. Its research archives
remain under the batch output and are not added to the default trajectory
library.

All 111 reports are present. They record 1,139 completed dense validations
totaling 61.952 s and 316 failed validations totaling 6.232 s. These are summed
worker durations, not batch wall or all IK time, and do not prove what
dominated earlier interrupted runs. Whole-case success/coverage accounting is
unchanged.

269 focused tests passed across planner/IK, smoothing/return routing, batch
validation, archive/model resources, saved ranges, runtime provenance and
mocked console/panel control modules. An additional broad screen-module run
exposed two unrelated RailTests failures: 14 px horizontal overflow and a
trace-start join accepted where the test expects rejection. Both reproduce
when every repository Python import is loaded from the pre-change diagnostic
code snapshot, with original module paths for asset resolution. Their sources
were not changed here; they remain unresolved, not silently counted as passes.
The final combined focused run is preserved in
`contact_constraint_probe_v1/focused_tests.log` (269 tests, 2.548 s).

`constrained_experiment_audit.v1.json` independently reloaded all 97 new
archives, seven stages each, and verified current code, code snapshots, native
runtime snapshots, reports and archived runtime provenance agree. The 316
failed dense validations comprise 158 TCP geometry, 123 joint margin and 35
collision rejects. These include rejected smoothing/candidate attempts in
otherwise passing cases, not 316 additional failed tasks.
`constrained_full_delivery_audit.v1.json` separately reloaded the unchanged
default library: 331 archives (230 left / 101 right), zero errors. The new
passes' stored validation summaries report maximum TCP error 2.988271 mm,
tangent 4.991230 degrees, tilt 4.970947 degrees, twist 9.890000 degrees,
1,473,693 dense arm samples, and minimum margin beyond the 0.02 rad guard
0.000094454 rad. This audit checks stored summaries and fresh archive loading,
not an independent recomputation of all timed FK samples.

`rviz_constrained_right_v1` exercised the new plan0020 right-sweep archive on
exclusive localhost domain 93, separately from benchmark domains 94-99. All
seven stage state/TF checks passed with 428 observed simulated joint messages,
zero smoke errors, robot-only foreground 4.919%, scene-added change 6.976%, and
inter-stage viewport change 3.485%. Approach, rotate and final screenshots were
visually checked for meshes, point cloud and full framing. Stage seeking is
not a joint-continuity measurement. No hardware commands were sent. Auxiliary
diagnostics/visualization shared the host with the full benchmark; timing
differences are not a clean single-factor performance comparison.
Final process checks found no remaining offline replay, benchmark, MoveIt,
state publisher, gallery, RViz or Xvfb processes; domain locks 93-99 were free.

## Candidate Diversity Investigation

The full existing 504-job preinsert grid was enumerated without ROS nodes for
both hard right sweeps. The unchanged bounded IK and scoped collision checker
accepted 63/65 states; 25 in each case were at least 0.25 rad in maximum joint
coordinate distance from every previously attempted complete candidate. The
second case had two previously undelivered orientation-18 states. Evidence:
`preinsert_grid_probe_v1/plan0052.v1.json` and `plan0080.v1.json`.
These are sampled valid initial states, not complete paths or a proof of
disconnected components. The initial fixture invocation stopped before any
search because it compared file-byte and normalized-JSON hashes; the corrected
fixture reuses the production archive-geometry record and checks capture state.

The explicit offline `--preinsert-policy diverse` preserves the first batch,
then interleaves existing grid jobs and balances previously delivered
orientation families plus sampled joint separation across batches. Original
IK, validation gates, candidate/time budgets, TCP, hand posture, physical
geometry, gains and default policy remain unchanged. Details and limitations
are recorded in `D_RING_METHOD_RESEARCH.md`.

`diverse_preinsert_parallel6_v1` finished 26 direction tasks from 20 contexts:
23 archive/reload passes, two failures and one timeout, wall 378.679 s.
Successful cases took 24.396-120.270 s. All 15 old passes in this selection
remained passes; seven old timeouts recovered. The separate startup fix also
remains effective but is not a new scheduling recovery. All 23 archives were
freshly reloaded in `preinsert_grid_probe_v1/targeted_audit.v1.json`.
Both hard right cases still failed. The new orientation-18 choices in
`plan_0080` collided during approach. `plan_0035` right remains a supervisor
timeout even though its report records exhaustion of all 16 complete candidates.
The subsequent `all_replayable_diverse_preinsert_parallel6_v1` completed the
entire frozen 111-direction / 99-input-ready-pair cohort on six workers,
domains 94-99: 108 passes (74 left / 34 right), two failures and one timeout.
Batch wall was 992.154 s; accepted case wall times were 24.493-126.130 s,
median 41.070 s. All 100 original bounded/OMPL passes repeated. Seven old
timeouts recovered in BOTH the targeted and full runs; the eighth, `plan_0035`
right, still timed out. The startup-repaired task passed too, but that is not
attributed to candidate diversity. Complete required-direction coverage is
96/337 (28.49%); blocked pairs remain in the denominator. Summary SHA256:
`4c4fa15db57792874491e42ab7cc92559d206c4d9b98cf7ffc0f27850bbb90c7`.

`preinsert_grid_probe_v1/full_audit.v1.json` freshly reloaded all 108 archives,
verified all 111 per-case source/native integrity proofs, checked current code
against frozen snapshots and compared available first-batch schedules with the
original full run. There are 110 available reports: the full-run `plan_0035`
right process was killed before report serialization, unlike the targeted
run's retained failure report. Its missing partial timings are not invented.
The 110 available reports contain 1,639 passed dense validations (202.345 s)
and 266 failed ones (26.665 s): 205 TCP geometry, 56 collision and five joint
margin rejects. These include rejected attempts inside passing cases, not
additional failed tasks, and summed worker time is not batch wall time.

All 108 archives were published byte-identically under
`data/offline_trajectory_library/all_replayable_diverse_preinsert_parallel6_v1/`.
`diverse_full_delivery_audit.v1.json` reloaded the whole default library:
439 archives (304 left / 135 right), zero errors. These include repeated task
contexts from independent runs, not 439 unique successes. Stored summaries for
the 108 new passes report maximum TCP error 2.208934 mm, tangent 4.999384 deg,
tilt 4.987606 deg, twist 9.916282 deg, 5,408,443 dense arm samples and minimum
margin beyond the existing 0.02 rad guard of 0.000965069 rad. This is a stored
validation-summary audit plus fresh loading, not an independent recomputation
of every timed FK sample. Publication did not grant hardware execution rights.

293 selected tests passed, including a final logged run in
`preinsert_grid_probe_v1/focused_tests.log`. Another 80 grid/prefix checks
against the frozen pre-change implementation confirmed the legacy sequence
and complete finite grid are preserved. `diverse_recovered_reuse_parallel3_v1` passed all
nine independent saved-range speed/acceleration checks on three newly recovered
left-sweep archives, batch wall 157.426 s; maximum normalized polynomial
difference 3.87e-9, immutable source geometry and hand timing preserved.
`rviz_diverse_recovered_left_v1` failed the 80 s readiness/stage smoke deadline
and is retained. The unchanged-code v2 repeat passed all seven stage state/TF
checks with 777 simulated joint messages, robot-only foreground 5.609%,
scene-added change 6.016% and inter-stage viewport change 3.849%. Approach,
rotate and return screenshots show the meshes, cloud and full robot framing.
Stage seeking is not a joint-continuity test. No real panel or hardware node
was started. These auxiliary checks shared the host with the full benchmark.
Final checks found no remaining replay/benchmark, MoveIt, state publisher,
gallery, RViz or Xvfb processes. Domain locks 93-99 were all free. Default YAML,
original inventory/evaluation and the derived-v2 manifest remain unchanged;
native runtime identity remains the previously verified startup-fixed build.

## Reverse Contact Diagnostic

`reverse_contact_probe_v1` adds diagnostic-only code; production planner,
configuration, native runtime, historical inputs and benchmark counts did not
change. The two hard right sweeps were evaluated in concurrent independent
processes without ROS nodes or hardware connections. Source report hashes match
`constrained_contact_parallel6_v1/0005/replay/report.json` (plan0052) and
`0006/replay/report.json` (plan0080), not the earlier rejection-probe batch.

Rechecking the old isolated endpoints against the exact production
`StraightIndexPolicy.tangent_reference` accepted only 3/9 and 4/11 states.
The other thirteen exceeded current tilt gates: the old endpoint diagnostic
rotated a recorded preinsert quaternion without production's tangent alignment.
In particular, the three previously reported orientation-16 endpoints did not
survive this recheck. Those old outputs remain as evidence of that diagnostic,
not production-compatible endpoint feasibility. The seven revalidated states
also failed reverse continuation, at most 66/120 and 58/120 rotate waypoints.

The new `endpoint_seed_probe.py` repeats the finite 24-orientation, three-seed
endpoint search using the production reference. It found 13/72 and 14/72
isolated states in 11.984/12.245 s, with existing scoped collision checks.
Only predecessor joint/orientation gates are omitted for an isolated endpoint;
they are fully restored during reverse continuation. All 27 new endpoints
passed that initial revalidation. However, none completed reverse rotation:
maximum progress was 98/120 (plan0052) and 91/120 (plan0080), in 13.267/12.577 s
per case. Rejections comprise 17 local IK, seven existing arm/body collision,
and three sampled-edge tangent failures. The latter reuse the validator's
`RETIMED_TCP_BOUNDS` code but are untimed linear-edge checks, not retimed paths.
No attempt reached reverse insertion, approach, withdrawal, return or export.
No whole-task pass, global infeasibility proof, or timing credit is claimed.

Five diagnostic tests and seven existing contact-IK tests passed. All input and
production-code hashes were unchanged across the probes. Negative outputs,
pointwise traces, source hashes and logs remain in the diagnostic directory.
Best complete evidence remains 108/111 directions, 96/99 input-ready pairs,
and 96/337 retained-pair conservative coverage, with 238 pairs still blocked.

## Reproduction

```bash
cd /home/syf/g1_upper_body_ws
source /home/syf/unitree_ws/env/setup_unitree_humble_g1.sh
source install/setup.bash
cmake --build build/g1_upper_body_moveit_config --target single_arm_planner_server -j2
bash scripts/build_task_ompl_route.sh
/usr/bin/python3 scripts/task_history_benchmark.py \
  --manifest data/offline_history/20260917/recorded_ready_recovery_v1/evaluation.v2.json \
  --output data/offline_history/20260917/NEW_BATCH \
  --workers 6 --domain 94 --budget 180 --export-archive \
  --ik-backend bounded --joint-route-backend ompl --preinsert-policy diverse

bash scripts/run_task_offline_gallery.sh --ros-domain-id 93 \
  --archives data/offline_trajectory_library/all_replayable_diverse_preinsert_parallel6_v1/88fb41f3dc78ccbc8739 \
  --output data/offline_history/20260917/NEW_GALLERY
```

For the previous bounded/OMPL candidate-order control, omit `--preinsert-policy diverse`.
For the bounded straight-return control, also omit `--joint-route-backend ompl`.
For the original MoveIt/straight-return control, omit all three experiment options.
For the explicit task-constrained contact experiment, replace
`--ik-backend bounded` with `--ik-backend constrained` and omit the preinsert
policy option; this is not the default and the combined ablation was not tested.
Solver experiments remain opt-in. Do not edit any `scripts/*.py`, `*.sh`, `*.cpp`
or rebuild a `*.so` during a benchmark. Do not edit/rebuild the native runtime,
replay launch or DDS profile either: every case audits the frozen hashes.
Do not use a domain held by another worker or real node.
The private Xvfb setup for headless screenshots is under
`/tmp/g1-offline-xvfb-syf`; no system package installation was performed.

At the earlier bounded-v2 checkpoint, 251 focused tests plus 11 evidence/ready
recovery tests passed (262 total). After the OMPL full run, 253 focused tests
and 16 evidence/cohort tests passed (269 total in those selected modules). They
cover FK/Jacobian, scoped planner, incremental search, archive/publishing,
fake-node panel, saved ranges, task contract and offline playback. Tests do not
establish the target success rate. Research decisions and pinned primary
sources are in `D_RING_METHOD_RESEARCH.md`.

Final cleanup found no remaining offline replay, benchmark, MoveIt, state
publisher or RViz processes. All exclusive domain locks 93-99 were available.
At that checkpoint every script matched the bounded-v2 snapshot. The subsequent
opt-in continuation and OMPL experiments above have their own frozen snapshots;
the original evaluation/inventory and default planning YAML hashes remain unchanged.

## Remaining Work

1. Preserve the new bounded/OMPL/diverse full cohort as the best recorded
   complete result: 108/111 directions and 96/337 retained pairs. Its two hard
   right-sweep failures and `20260912_155322_095139/plan_0035` right timeout
   remain unresolved. Improved initial-candidate coverage did not produce a
   complete path for them. Direct `constrained` contact IK had introduced four
   regressions and remains an opt-in negative experiment. Investigate
   multi-waypoint/branch propagation and the rejected approach states without
   raising budgets or relaxing gates. Live defaults remain unchanged.
2. The unused scene-interface startup dependency is fixed and separately
   regression-tested. Investigate the remaining early DDS reply-discovery risk
   without conflating it with geometry or replacing the original full-run failure.
3. Profile rejected IK work and branch diversity. Failed/cooperatively
   interrupted dense-validation timing is now recorded, but hard process kills
   still cannot serialize pending intervals. A final log line in dense
   validation does not establish that validation dominated earlier deadlines.
4. Repeat the complete frozen 111-task cohort after any further justified solver
   change. Keep common-input comparisons and added task evidence separate, and
   retain every negative development/full-run result.
5. Resolve historical applicability, direction, calibration/ready/home and
   physical scene grouping for the 238 blocked pairs. The 83 raw-3D targetless
   records have now been audited: they record prepare/recover, not a sweep.
   Geometry-only conversion cannot fill that task definition. Camera receipts
   still lack measured hand state and task intent. Do not infer from neighboring
   or current state. Even perfect success on all 99 input-ready pairs would cover
   only 29.38% of the retained 337 pairs; the all-history goal remains unmet.
   The fourteen unknown-task contexts were also audited without recovery; only
   two are otherwise input-complete. Do not treat prepare/center-check records
   as sweeps or infer a full task from one failed stage's direction.
