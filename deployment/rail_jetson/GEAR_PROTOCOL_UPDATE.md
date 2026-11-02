# Gear protocol update, 2026-09-28

## Status

Deployed to Jetson on 2026-09-28 after the user powered it on. Activation receipt:
`data/rail_deployment/gear_protocol_20260928/activation_receipt.json`.
All three services are active; API reports `gearOperationConfigured=true` and the
expected library hash. All four recipes loaded successfully under Jetson Python.
No initialization, recovery or motion request was sent during deployment.
Post-install state: uninitialized, no active task, DETACHED; the existing recovery
latch is waiting for a new R2+A. Physical execution and callbacks remain untested
for this update. The package verification.json records the earlier offline phase.

Package: `data/rail_deployment/gear_protocol_20260928/`.
The six modified runtime modules derive from the last deployed recovery update,
not from newer WSL-only features. The installer preserves the selected release,
native binaries/configuration, task databases, callbacks and credentials.
No C++ rebuild is needed.

## Protocol semantics

`GEAR_OPERATION` parameters have exactly three fields:

- `count`: integer 1..2147483647. Number of cycles, excluding preliminary extension.
- `preAction`: `NONE` starts at LOWER; `EXTEND_DOWNLOCK` starts at UPPER and first extends to LOWER.
- `cycleMode`: `FULL_CYCLE` retracts then extends each cycle; `FINAL_RETRACT_ONLY` omits the last extension.

UPPER means retracted; LOWER means extended. All modes start with preparation,
one recorded approach and grip, and end with opening the hand and recorded exit.
These names do not imply that physical lock sensors have been verified.

For count 1:

| preAction | cycleMode | Contact path |
| --- | --- | --- |
| NONE | FULL_CYCLE | LOWER -> UPPER -> LOWER |
| NONE | FINAL_RETRACT_ONLY | LOWER -> UPPER |
| EXTEND_DOWNLOCK | FULL_CYCLE | UPPER -> LOWER -> UPPER -> LOWER |
| EXTEND_DOWNLOCK | FINAL_RETRACT_ONLY | UPPER -> LOWER -> UPPER |

The two previously approved three-leg combinations map to:

```json
{"taskId":"GEAR-LOWER-001","taskType":"GEAR_OPERATION","callbackUrl":"http://192.168.0.249:8091/eln/api/v1/robot-callbacks/task-status","parameters":{"count":2,"preAction":"NONE","cycleMode":"FINAL_RETRACT_ONLY"}}
```

```json
{"taskId":"GEAR-UPPER-001","taskType":"GEAR_OPERATION","callbackUrl":"http://192.168.0.249:8091/eln/api/v1/robot-callbacks/task-status","parameters":{"count":1,"preAction":"EXTEND_DOWNLOCK","cycleMode":"FULL_CYCLE"}}
```

Use the existing public 8090 task submission interface and existing authentication.
Use a fresh taskId for each new physical trial. Do not send these examples until
deployment is verified and a coordinated initialized physical test is intended.

## Trajectory evidence

The sidecar contains eight immutable optimized units from the approved snapshot.
Both existing three-leg compositions reconstruct exactly, including position,
velocity, acceleration, timestamps and hand commands. Source plan hashes, snapshot
hash and model hash bind the sidecar to the selected actions.

Units were checked with the existing continuous-polynomial derivative validator.
No trajectory speed limit or jerk policy was relaxed. This is protocol composition,
not a new speed optimization. No online time parameterization occurs.

Counts are generated lazily in blocks of at most eight cycles. Within a block,
the same C2 stitching as the offline preview is used. A controller goal boundary
and settling check can occur between blocks; arbitrary repeats are not claimed
to have nonzero velocity at every junction. Only one approach/grip/exit is used.
Reported nominal duration excludes live initial alignment/controller settling
and includes all three hand waits. The two original examples are approximately
59.71 s and 74.45 s respectively, before those extra live costs.

Failures remain FAILED, without replay. Confirmed gear checkpoints use the existing
automatic recovery gates and recorded exits. Unknown mid-contact failures remain
blocked. A new request can wait for active recovery under the existing API policy.
Motion completion does not prove external uplock/downlock or WOW signals.

## Offline verification

Completed on 2026-09-28: local suite 58/58 passed; staged runtime suite
26/26 passed. All eight deployment Python files passed Python 3.8 syntax parsing.
The staged suite includes a simulated gear rejection entering automatic recovery
without replaying the failed task. These totals overlap; they are not 84 unique tests.

From the workspace root, local regressions:

```bash
env PYTHONPATH=deployment/rail_jetson:scripts OPENBLAS_NUM_THREADS=1 python3 -m unittest test_gear_protocol test_automatic_recovery test_native_worker test_cloud_executor test_task_waist_rate test_task_waist_envelope -q
```

Test the actual staged Jetson runtime independently:

```bash
env PYTHONPATH=data/rail_deployment/gear_protocol_20260928/runtime:deployment/rail_jetson:scripts OPENBLAS_NUM_THREADS=1 python3 -m unittest test_gear_protocol test_automatic_recovery -q
```

HTTP tests against the real communication package and physical execution remain
pending power-on. Offline transports validate control flow, not actuator behavior.

## Next power-on deployment

First inspect services, task queue, controller ownership and pending requests.
Require idle, uninitialized DETACHED state, no automatic recovery, and no WSL
controller publishing. Pause upstream submissions for the deployment window.
Do not initialize or acquire output simply to install this update.

Transfer the package to Jetson and set `R` and `S` to the installation root and
uploaded staging directory. All commands below are single-line commands.

```bash
R=/home/unitree/rh56dfx/workspace/execution/action_runtime_v3
```

```bash
S="$R/data/updates/gear_protocol_20260928"
```

```bash
python3 "$S/install_gear_update.py" --root "$R" --staging "$S" --check-only
```

```bash
sudo -v
```

```bash
python3 "$S/install_gear_update.py" --root "$R" --staging "$S"
```

The installer refuses changed baseline hashes, an unexpected selected release or
preexisting sidecar. It backs up changed files and starts the executor passively.
Verify health `gearOperationConfigured=true`, matching API/native library hash,
`initialized=false`, no active task and DETACHED output. No task is sent by the
installer. If activation cannot be verified, inspect the printed backup path and
service state before rollback; never overwrite a controller claimed by someone else.

Then test public HTTP admission/callbacks offline on Jetson with a simulated native
transport before arranging real initialization and one example of each gear mode.
Do not use production task submission as a supposed harmless connectivity check.
