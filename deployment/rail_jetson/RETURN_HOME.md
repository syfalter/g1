# Return to R2+A upper-body home

## Deployment status (2026-09-24)

The candidate passed Jetson localhost-only DDS domain 174 acceptance, including
HTTP relay, task completion callbacks, measured home endpoint and retained hold.
No physical return-home motion was tested. The production executor has NOT been
switched to this version. The candidate directory is not the active service.
The final offline package includes a subsequent cancellation-status correction.

Activation must wait until the robot is available again. Inspect real health,
confirm no active task and a detached, silent Jetson controller, and coordinate
with whoever is using the robot. Back up the active runtime, deploy the final
package and restart only `eln_executor.service`; do not restart `hand_service`
or another operator's controller. This is a Python runtime change; no C++ build
is required. Do not restart an armed controller to perform this update.

After activation, GET `/openapi.json` through port 8090 must include
`/api/v1/robot/return-home`. Verify passive startup before explicitly initializing
for the supervised physical test. Existing recordings/configuration/API keys and
task journals must not be replaced. Rerun isolated acceptance on the final
package before activation. No initialization or home request is part of deploy.

## Contract

POST `/api/v1/robot/return-home` on the real relay at port 8090 (or native API
8795). Headers: `Content-Type: application/json`, `X-API-Key: <existing key>`.

```json
{"taskId":"home-20260924-001","callbackUrl":"http://YOUR_SERVER_IP:18080/robot-callback","contactClear":true}
```

`contactClear=true` confirms unloaded hands and a clear return path. The local
joint-space planner checks joint limits, not scene collisions. Do not return
directly while grasping a fixture or inside an obstructed workspace.

The robot must already be initialized under Jetson control, idle and free of
safety latches. This request never acquires WSL control, clears a fault or
initializes the robot implicitly. Busy/uninitialized requests are rejected, not
queued. Each new attempt needs a unique taskId. Do not retry an uncertain POST
with a new ID: query the original ID first.

Accepted response:

```json
{"code":0,"message":"accepted","taskId":"home-20260924-001","status":"ACCEPTED"}
```

Callbacks use the existing event format with `taskType: ROBOT_RETURN_HOME`,
statuses ACCEPTED/RUNNING/SUCCEEDED/FAILED/CANCELLED, and progress 0 then 1.
The callback receiver must return HTTP success and `{"code":0}`. Add the new
taskType to the upstream enum. Query with POST `/api/v1/tasks/{taskId}/query`
and `{}`; cancel with POST `/api/v1/tasks/{taskId}/cancel` and
`{"reason":"operator cancelled"}`. Both require the API key.

## Motion semantics

Both arms and waist move from measured stationary joint positions to the
operator-confirmed R2+A pose captured at 2026-09-23 19:31:44 +08:00.
The 15 joint values and provenance are in `home_pose.py`, also pinned in the
execution audit. Velocity and acceleration scaling are 0.7 and 0.7. Existing
controller protections, cancellation and measured endpoint checks remain.
No finger commands or leg commands are sent. Supporting output remains active.

**SUCCEEDED means upper-body pose reached, not permission to cut power.**
Balance, body support and the manufacturer's shutdown procedure remain external.
This does not replace the separate supported output-release endpoint. It does
not implement automatic recovery after FAILED. Failed/cancelled tasks retain
the existing recovery requirements.

## LAN test (after deployment is activated)

These commands cause physical motion. Clear the workspace first. Set
ROBOT_API_KEY to the existing key and replace the callback address with an
upstream HTTP endpoint reachable from Jetson.

```bash
curl --noproxy '*' -X POST http://192.168.123.164:8090/api/v1/robot/return-home -H "X-API-Key: $ROBOT_API_KEY" -H 'Content-Type: application/json' -d '{"taskId":"home-20260924-001","callbackUrl":"http://YOUR_SERVER_IP:18080/robot-callback","contactClear":true}'
```

```bash
curl --noproxy '*' -X POST http://192.168.123.164:8090/api/v1/tasks/home-20260924-001/query -H "X-API-Key: $ROBOT_API_KEY" -H 'Content-Type: application/json' -d '{}'
```

Alternatively, the CLI starts its own callback receiver. Replace YOUR_PC_LAN_IP;
allow incoming TCP 18080 on the PC. No WSL controller or Web UI is needed:

```bash
python3 deployment/rail_jetson/robot_request.py return-home --base http://192.168.123.164:8090 --callback-host YOUR_PC_LAN_IP --contact-clear
```
