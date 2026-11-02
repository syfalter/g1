# Jetson standalone test

## Real relay and controller handoff

`http://192.168.123.164:8090/docs` is now the REAL request relay, not the old
simulator. It forwards once to the production API on 8795, preserving API keys,
task IDs, request bodies, HTTP/business errors and callback URLs. Callbacks go
directly from the executor to the supplied PC/cloud address. No automatic retry
is performed on an uncertain request. Query the original task ID before retrying.
The same ports are available at Wi-Fi address `192.168.0.249`.
The existing `eln_interface.service` owns 8090 and reads the explicit
`config/interface_relay.json` configuration. Its entry-point source is mirrored
in `deployment/rail_jetson/eln_interface_entrypoint.py`; the installed copy is
`communication/eln_http_demo/deployment/interface_service.py`. No system unit or
system library was modified. If 8795 is unavailable, the relay returns 502;
it never pretends a real task succeeded using the simulator.

The HTTP services start with `native.phase=DETACHED`, `controllerRunning=false`:
there is no Jetson gateway, action server or motor-command publisher. Read-only
robot subscriptions remain. Only an explicit initialize request starts the
gateway, verifies that WSL relinquished control, waits for fresh stationary
feedback/torque history, and then ARMs and moves to preparation.

L2+B silences output immediately through the native gateway. Once task stopping
finishes and silent output is confirmed, the worker exits its own gateway and
returns to DETACHED automatically. It never kills a WSL process. After confirming
`controllerRunning=false`, press R2+A to return the robot to motion control and
start WSL normally. R2+A only clears task admission; it never launches a gateway,
ARMs or sends a trajectory. Stop WSL before sending a new Jetson initialize.
Neither a service restart nor an uncertain timeout replays a task.

The docs page is Swagger request/query tooling, not a visualization dashboard.
`http://192.168.123.164:8090/health` returns actual controller state. The old
8791 preview remains simulation-only. SSH is only maintenance/key setup.

Example single-line Bash requests on another PC, after securely setting KEY and
BASE to the API key and reachable Jetson base address. Only send the true recovery
confirmations after inspecting, clearing contact and supporting the robot:

```bash
BASE=http://192.168.0.249:8090
read -rsp 'API key: ' KEY
curl --noproxy '*' -sS "$BASE/health"
curl --noproxy '*' -sS -X POST "$BASE/api/v1/robot/recover" -H "X-API-Key: $KEY" -H 'Content-Type: application/json' -d '{"contactClear":true,"supportReady":true}'
```

Wait for `code=0`, `requiresInitialization=true`, then submit initialization with
a NEW task ID and an actual reachable callback URL. Do not repeat old uncertain
motion requests. The LAN helper documented below hosts a callback receiver for
PC testing and sends the same HTTP requests.

## Startup repair and LAN testing (2026-09-23)

The WSL execution chain already uses the onboard `g1_arm_example` service.
Keep it running. A registered but idle onboard `/arm_sdk` writer is now matched
by DDS participant properties (`g1_arm_example`, host `Unitree`) and its matching
command reader, then pinned by GUID for this session. This is trusted-LAN
identity checking, NOT cryptographic authentication. Other writers remain
blocked. Onboard command activity, observer failure, or changed identity blocks
admission and stops an owned active task; identities are not silently relearned.

Foxy's bundled Cyclone 0.7 crashed in discovery on the robot network. The
launcher now loads the existing Unitree SDK DDS libraries for its process tree
only; no system or WSL libraries were replaced. Runtime authorization is written
to a separate file because Foxy's node-specific parameter file overrode the CLI
flags. Startup remains DETACHED and sends zero motor commands. HTTP readiness is
verified before printing READY. The DDS observer is read-only and can be rebuilt
with `bash deployment/rail_jetson/build_native_helpers.sh` on Jetson.

The following are separate single-line commands, with no line continuation.
Do NOT start another copy if status already works. On Jetson:

```bash
cd /home/unitree/rh56dfx/workspace/execution/action_runtime_v3
nohup bash deployment/rail_jetson/run_standalone.sh --hardware --host 0.0.0.0 >>data/standalone.log 2>&1 </dev/null &
python3 deployment/rail_jetson/robot_request.py status
```

Wait for status to show `phase=DETACHED`, `controllerRunning=false` and
`connected=true`. After L2+B, press R2+A and wait for `requiresRecovery=false`.
No WSL controller may be running when you initialize. Battery, clear workspace, stable support and
physical stop must be checked before the following motion commands. Initialize
moves BOTH arms and waist; do not run the action until initialization SUCCEEDED:

```bash
python3 deployment/rail_jetson/robot_request.py initialize
python3 deployment/rail_jetson/robot_request.py dring_left
```

For another LAN PC, no ROS/WSL installation is needed, only Python 3 and SSH/scp.
Use `192.168.0.249` on the Wi-Fi LAN; use `192.168.123.164` on the robot wired
subnet. Check reachability with the status command first. Download the standalone
client (it has no repository imports):

```bash
scp unitree@192.168.0.249:/home/unitree/rh56dfx/workspace/execution/action_runtime_v3/deployment/rail_jetson/robot_request.py .
python3 robot_request.py status --base http://192.168.0.249:8795
```

Use `python` instead of `python3` on Windows when needed. Obtain the dedicated
key privately on Jetson with `cat config/api.key` from the deployment directory.
The client prompts for that key without echo. Do not post it into chat or embed
it in command history. `ROBOT_API_KEY` or `--key-file` are also supported.

For example, if the other PC's actual LAN address is `192.168.0.20`, these cause
real motion (replace that example address with the PC's address, NOT Jetson's):

```bash
python3 robot_request.py initialize --base http://192.168.0.249:8795 --callback-host 192.168.0.20
python3 robot_request.py dring_left --base http://192.168.0.249:8795 --callback-host 192.168.0.20
```

The helper hosts callbacks on PC TCP 18080 and returns `{"code":0}`. Permit
Jetson to reach that port in the PC firewall. Use the native Windows/Linux
terminal rather than a NAT-only WSL callback address. It prints ACCEPTED,
RUNNING and terminal events and also polls durable status. A callback delivery
failure does not mean motion failed: never repeat a task just to obtain callbacks.
Run one helper task at a time. The task ID is printed before submission; if the
connection fails after submission, query that ID instead of blindly resubmitting.

Cloud protocol (port 8795): `POST /api/v1/actions/dring_left/tasks`, header
`X-API-Key`, JSON `{"taskId":"unique-id","callbackUrl":"http://cloud/result"}`.
Use `/api/v1/robot/initialize` first with the same body shape and a different ID.
Every new task gets a unique ID. Query with `POST /api/v1/tasks/{taskId}/query`.
The 8090 mock gateway is separate; it does not execute these trajectories.

STOP from any LAN PC (also available without a callback listener):

```bash
python3 robot_request.py stop --base http://192.168.0.249:8795
```

After physically inspecting, clearing contact, and supporting the arms:

```bash
python3 robot_request.py recover --base http://192.168.0.249:8795 --contact-clear --support-ready
```

Then explicitly initialize again. For returning control to WSL, unload/support
the arms and use `release` with those confirmations, then stop the Jetson stack;
do not treat STOP as release. Automatic negotiated handoff remains separate.

## Scope and prerequisites

The production endpoint is port 8795, not the simulator Web or offline gateway.
No WSL controller, RViz, browser or SSH session is required during execution.
SSH is only used below to launch and submit a test request. No boot service or
automatic startup motion has been installed.

Before any real test: adequate battery, stable supported robot, clear workspace,
physical stop available, and ALL WSL/other arm controllers stopped. Initialization
will ARM and move BOTH arms plus waist at the frozen 0.7/0.7 scales. Closing a
browser alone does not stop its controller. Do not start WSL Execute concurrently.

## Launch on Jetson

```bash
ssh unitree@192.168.0.249
R=/home/unitree/rh56dfx/workspace/execution/action_runtime_v3
nohup bash "$R/deployment/rail_jetson/run_standalone.sh" --hardware --host 0.0.0.0 >"$R/data/standalone.log" 2>&1 </dev/null &
python3 "$R/deployment/rail_jetson/robot_request.py" status
```

Allow startup several seconds before status. Starting services alone does not
ARM or move the robot. If startup fails, inspect `data/standalone.log` and
`data/service_logs/{gateway,worker,api}.log`. Never launch a second controller to
work around an ownership rejection. Restrict port 8795 to a trusted network;
external cloud access needs a secured tunnel/proxy, not public plaintext HTTP.

Only after the prerequisites above are satisfied, these commands cause motion:

```bash
python3 "$R/deployment/rail_jetson/robot_request.py" initialize
python3 "$R/deployment/rail_jetson/robot_request.py" dring_left
```

The helper prints progress callbacks and waits for the terminal task result.
Ctrl-C requests cancellation, not power removal or output release.

## Cloud requests

Use `X-API-Key` from `config/api.key`, never publish that key. Send JSON:

```text
POST /api/v1/robot/initialize
{"taskId":"unique-init-id","callbackUrl":"http://reachable-cloud/result"}

POST /api/v1/actions/dring_left/tasks
{"taskId":"unique-action-id","callbackUrl":"http://reachable-cloud/result"}
```

Wait for initialization SUCCEEDED before a motion task. Task admission is not
completion. The existing callback protocol returns progress and terminal status;
`POST /api/v1/tasks/{taskId}/query` with `{}` queries the durable result.
Use unique task IDs; retries must not create duplicate motion. Completion means
the arm/hand sequence completed, not confirmation from an external WOW sensor.

Action names: `dring_left`, `dring_right`, `push`, `pull`,
`gear_above_to_lower`, `gear_lower_to_upper`, `gear_upper_to_lower`,
`gear_above_to_upper`. Gear requests execute exactly one segment.

## Recordings and updates

Currently selected: the user's latest `dring_left`, slot `left_3`, planned,
full-cycle, arm-only. Previous WSL execution evidence is NOT required. The
recording's arm-only setting is preserved; this does not command finger motion.
Other interfaces are implemented but reject requests until a recording is
explicitly selected. Historical recordings are not silently substituted.

Push the desired modules from WSL. With the standalone supervisor stopped and
output safely released, select one compatible package per action:

```bash
python3 "$R/deployment/rail_jetson/select_production.py" --root "$R" --bundle /absolute/path/to/left/bundle.json --bundle /absolute/path/to/another/module/bundle.json
```

Supply ALL desired modules on each selection; this replaces the selected set.
The selection command only writes configuration, never starts motion. Restart
the stack and explicitly initialize. Data-only updates require no C++ rebuild.
Web preview activation is separate and does not select this production runtime.

## Stop, recovery and output release

Authenticated `POST /api/v1/robot/stop` with `{}` requests STOP and retains arm
support. For an emergency, use the physical stop, not HTTP alone.

After L2+B, inspect the robot and clear contact, then press R2+A to return to
motion control. Jetson has exited its silent gateway and detects this NEW press
to restore task admission without starting a new gateway. No
extra Web or HTTP confirmation is needed. This does not ARM, move to preparation,
restart processes, or replay an interrupted task. Send initialize next, followed
by a NEW task ID. A chord held at startup is not a new confirmation; release and
press again after the service is ready. A recovery press expires after 30 seconds
if the robot cannot settle; check /health for the reason and press again.

For a supported HTTP STOP (output still armed), manual
`POST /api/v1/robot/recover` with `{"contactClear":true,"supportReady":true}`
remains available. These confirmations must reflect actual conditions. The
R2+A automatic path never restores an armed supported hold.

For WSL debugging, unload/support the arms before using L2+B. Wait until health
shows DETACHED and controllerRunning=false, then return to motion control using
R2+A and start WSL. No SSH shutdown or Web confirmation is needed. An HTTP
`release` remains an alternative controlled unloading procedure but does not by
itself exit the controller. Killing the API is NOT output release; the gateway
may intentionally remain alive holding the robot if output state is uncertain.

## Verification boundary

ARM64 native gateway and hand client compiled. Eight independent synthetic tasks
passed the actual native gateway against a software plant on localhost DDS174.
The selected original left recording also passed unchanged replay, cancel,
recovery, API-loss watchdog, restart reinitialization and output-release tests.
No domain-0 motion or physical finger operation was tested. First physical
commissioning must be supervised with adequate battery.
