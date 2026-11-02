# Headless ELN Integration Acceptance

Status: 2026-09-22. This is an OFFLINE execution integration, not a commissioned
hardware service. No controller, HandClient, ROS node or boot service is started.
No browser is required for the HTTP acceptance below.

## Architecture

Reuse the existing Jetson `communication/eln_http_demo/robot_gateway` HTTP routes,
TaskManager, SQLite task store and callback outbox. Inject `RecordedTaskExecutor`
instead of the simulator's timed business stages. Its `SequenceRuntime` invokes
the headless sequence executor with an explicitly injected transport.

The current `cloud_service.py` only wires `OfflineTransport`. There is no real
hardware flag. Setting inherited ROBOT_BACKEND cannot enable robot output.
The Web version simulator and this API have separate databases and ownership
states; simulated Web activation or recovery cannot authorize this executor.

The HTTP service pins an immutable full release at process startup. Each task
records that version and its source-plan hashes before dispatch. Uploading a
module patch does not activate it. The audit database is exclusively locked to
prevent a second instance from marking a live task as interrupted.

## Task Semantics

- AIR/GROUND use the release's configurable push/pull mapping.
- NWS progress increments only after both complete left and right actions.
- Success means all requested motion stages completed. The message explicitly
  says external WOW/lock signals were NOT verified.
- Callback failure retries only the callback. Task IDs cannot execute twice.
- Interrupted or uncertain execution latches recovery. Submitting initialize
  does not implicitly clear it. There is no commissioned recovery endpoint yet.
- Restart marks incomplete requests failed and never replays them. Initialization
  is required again. A restart is not a substitute for physical stop confirmation.
- Large release validation runs outside the HTTP event loop; action duration is
  not limited by a browser, callback or HTTP response timeout.

## Four Independent Gear Segments

The four bindings are `gear_above_to_lower`, `gear_lower_to_upper`,
`gear_upper_to_lower`, and `gear_above_to_upper`. One request executes one saved
program. No automatic cycle, next segment, or additional return is appended.
The recorded program's own local joins and hand timing remain part of that segment.

The upstream cloud selection format is **not decided**. Do not treat the old
GEAR_OPERATION count/preAction/cycleMode as permission to combine segments.
It is rejected before task admission in this acceptance app.

For internal offline testing only:

```text
POST /internal/v1/gear-segments/{segment}/tasks
{"taskId":"unique-id","callbackUrl":"http://callback-host/result"}
```

The segment is one of `above_to_lower`, `lower_to_upper`, `upper_to_lower`,
`above_to_upper`. The internal event type is GEAR_SEGMENT, total/progress is 1.
This type is NOT accepted by the public `/api/v1/tasks` contract. Count and
combination fields are rejected. Final cloud field names can later map onto
these internal commands without changing the recorded trajectories.

## Run Acceptance Without a Browser

The isolated Jetson staging root is:

```text
/home/unitree/rh56dfx/workspace/execution/action_cloud_offline_20260922
```

From WSL, run:

```bash
ssh -o BatchMode=yes unitree@192.168.0.249 '
  /home/unitree/rh56dfx/workspace/communication/eln_http_demo/.venv/bin/python \
  /home/unitree/rh56dfx/workspace/execution/action_cloud_offline_20260922/deployment/rail_jetson/check_cloud_http.py \
  --gateway-root /home/unitree/rh56dfx/workspace/communication/eln_http_demo \
  --release /home/unitree/rh56dfx/workspace/execution/action_runtime_v2/data/action_releases/baseline-20260921-v2/bundle.json \
  --output /home/unitree/rh56dfx/workspace/execution/action_cloud_offline_20260922/data/manual_http_acceptance'
```

This launches a temporary loopback API and callback listener on Jetson, tests
initialize, push, pull, NWS, duplicate requests, callback retries, exact frozen
trajectory hashes/dispatch counts, cancellation and restart recovery, then
stops only its own test processes. Expected result: passed=true, simulation=true,
hardware_commands_sent=0. It does not stop the current WSL control panel.

`check_cloud_segments.py` separately exercises the four internal segment routes,
authentication, no automatic follow-on, and rejection of unknown/count requests.
Its synthetic fixture is only for control-flow verification, not physical motion.

The historical September 21 baseline is an acceptance fixture, not an implicit
selection of the user's current retaught production recordings.

## Verification Boundaries

- HTTP and callback integration: exercised on Jetson using its existing gateway
  Python environment; no edits to the existing cloud project or global packages.
- Recorded payloads: hashes and dispatch count compared against the selected
  release. Synthetic offline joins are NOT evidence of native planning parity.
- Physical hardware reads and motion tests in this round: NONE.
- Remaining: native ROS/hand transport, real output ownership/WSL handoff,
  commissioned recovery, dual-arm preparation and supervised hardware acceptance.
  Foxy Python and the HTTP virtualenv differ; their eventual transport boundary
  must not depend on importing Foxy bindings into the HTTP Python environment.

For WSL planning/Push and version preview testing see `PUSH_TESTING.md`.
