# Waist Limits And Automatic Preparation Recovery

Date: 2026-09-28. Runtime: `action_runtime_v3`.

## Motion Limits

The selected five trajectories are unchanged. Their planning waist limit is
0.35 rad/s. The push curve has a sampled peak of 0.3476449302 rad/s, a continuous
Hermite peak of 0.3476578556 rad/s, and a conservative velocity enclosure of
0.3478074464 rad/s. All are below 0.35.

Previously, the gateway reduced 0.35 to `0.006 * 50 * 0.95 = 0.285`.
The Jetson configuration now updates the waist at 100 Hz, matching the existing
100 Hz arm packet rate. Its effective waist limit is 0.35. The 0.006 rad command
step limit, acceleration, tracking, torque, temperature, and ownership guards
are unchanged. WSL defaults remain at 50 Hz. These are configured command bounds,
not independently established motor performance limits.

## Recovery Contract

- Startup remains passive. R2+A alone never acquires output; explicit initial
  initialization is still required after enable/startup or an operator stop.
- A failed task remains FAILED. Recovery does not replay that task or turn its
  callback into success. The next request needs a new taskId.
- A confirmed trajectory rejection or endpoint-settling failure can schedule
  one automatic recovery attempt. Partial/uncertain contact execution, finger
  failures, initialization failures and unsupported recording layouts do not.
- Recovery requires confirmed stopped output still owned by Jetson, fresh
  robot/controller/discovery feedback, valid enabled state, no monitored fault,
  and actual joints within 0.05 rad of a known recorded checkpoint. The other
  arm must also remain near its preparation checkpoint.
- Gateway STOP settling is allowed up to 15 seconds, followed by 0.5 seconds of
  continuously valid stationary evidence. Recovery resumes supported hold;
  it never sends ARM or clears a disabled/silent safety latch.
- Near preparation: restore preparation. At a pre-contact approach endpoint:
  use the recorded release-hand posture and reverse only that approach. At a
  completed contact endpoint: use the recorded release-hand posture and exit.
  Do not reverse a contact operation. All escape routes end at preparation.
- Recovery is attempted once. A new STOP, stale feedback, lost ownership,
  unsafe checkpoint or recovery failure blocks it. No infinite recovery loop.
- After the failed task terminates, a new request can return ACCEPTED while
  recovery is pending/running. The existing task manager admits one task at a
  time; further concurrent requests remain busy. The new task reports preparation
  recovery as a RUNNING stage, and dispatches its action only after recovery.
- If recovery fails, the waiting new task fails without starting its action.
- Restart does not resume recovery, replay failed tasks, or erase task history.

`GET /health` now includes `automaticRecovery.status`:
`idle`, `pending`, `running`, `ready`, or `blocked`, plus a diagnostic message.
During pending/running, `initialized` is false even though new-task admission is
allowed. Read the recovery status instead of interpreting this as a request to
cycle L2+B/R2+A. Hardware motion remains unverified until on-site acceptance.

## Incident Evidence

- WOW-AIR-001: start alignment and hand grip completed; the continuous push
  goal was rejected for `declared velocity exceeds limit: waist_yaw_joint`.
- WOW-GROUND-001: rejected at API admission with error 2003 because the previous
  failure still required explicit recovery. No pull trajectory was submitted.

## Testing After Deployment

Keep WSL hardware output stopped. Enable the robot, use R2+A, then submit an
initialization request and wait for SUCCEEDED. Use fresh task IDs for push/pull.
Do not deliberately obstruct the robot to test failures. Fault-injected recovery
and HTTP tests are offline only; perform physical recovery acceptance with an
operator, a clear recorded escape path and an immediate stopping method.

Public request port remains 8090; read-only observation remains 8092. Cloud
mapping and callbacks are unchanged. GEAR_OPERATION remains unmapped; the two
independent gear action endpoints are unchanged.

Deployment artifacts and receipts are in
`incoming/recovery_20260928` on Jetson and
`data/rail_deployment/recovery_20260928` on WSL. An original-file backup is
created under `data/deployment_backups/recovery-waist-*` before activation.
