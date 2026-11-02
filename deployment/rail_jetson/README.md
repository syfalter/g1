# Jetson Recorded Actions

## Native standalone execution (2026-09-22)

The native ROS transport, ARM64 gateway, headless production API and launcher
are now deployed separately in `execution/action_runtime_v3` on Jetson.
See [STANDALONE_TESTING.md](STANDALONE_TESTING.md) for real startup and requests.
All eight independent action interfaces passed isolated native integration;
the selected original left D-ring recording also passed unchanged replay,
cancel/recovery and API-loss watchdog tests. No physical motion was tested.
Only explicitly selected recordings are executable. Boot motion is NOT enabled.

## Push update (2026-09-22)

The WSL recording panels now have a Push control for their current PLAN_READY
snapshot. `releases.py` handles immutable full releases and module-only patches;
`upload_release.py` transfers and verifies them without activating anything.
The Web preview supports version inspection, simulated activation and rollback,
including D-ring slot selection provenance and landing-gear segment packages.
See [PUSH_TESTING.md](PUSH_TESTING.md) for the test procedure and exact scope.

The existing ELN HTTP gateway now has a separately staged headless recorded-task
adapter, verified on Jetson with an offline transport and real loopback HTTP
requests/callbacks. See [HEADLESS_TESTING.md](HEADLESS_TESTING.md). Gear tasks remain
four independent segments; no combined gear business flow or final upstream
segment schema is assumed.

The original WSL Execute path is unchanged. `executor.py` has a local join
planner and transport-independent sequence engine; `ros_transport.py` now
provides the native backend. Physical commissioning, coordinated WSL grants
and automatic boot preparation remain incomplete. The version Web is not real
telemetry. The older preview and offline HTTP entry points remain simulated.
The browser does NOT release the currently running Jetson or WSL controllers.
Do not expose this preview as the production ELN endpoint.

## Agreed behavior (2026-09-21)

- Configurable initial mapping: AIR -> push; GROUND -> pull.
- No per-task manual confirmation or trajectory refitting. Preserve hardware
  faults, STOP, fresh hardware feedback, single-owner authority and bounded joins.
- Robot arm/hand sequence completion is success, not an external WOW assertion.
  Production protocol documentation must explicitly record this deviation.
- Boot preparation: simultaneous 14 arm joints plus one shared waist yaw target;
  velocity/acceleration scales 0.7/0.7, waist -0.015293545089662075 rad.
- Blocked task: confirmed stop -> FAILED/CANCELLED -> inspect -> reset -> explicit
  preparation -> a NEW task ID. Never replay an interrupted contact segment.
- Full robot boot may automatically initialize once; process restart may not.
- WSL debugging and Jetson automation are mutually exclusive control owners.
  Remote UI loss never implies output release or permission to take over.

## Frozen baseline

`bundle.py` exports the successful push at 16:22:36 and cached pull at 17:08:42
(Asia/Shanghai, 2026-09-21). The six recorded curves, hand targets, complete
reference joins, timing, planning parameters and execution evidence are retained.
The effective waist velocity limit is 0.285 rad/s, not the YAML request 0.35.
Frozen baseline joins are reference examples, not valid starts for arbitrary
future robot postures. Production joins must use the existing local planner.

The bundle integrity is checked at load time. No fitting or retiming occurs in
the preview. In production, expensive consistency checks belong at deployment
and load time; per-command hardware feedback protection remains required.

## Offline preview

From `/home/syf/g1_upper_body_ws`:

```bash
python3 deployment/rail_jetson/server.py \
  --bundle data/rail_deployment/20260921_offline_v1/bundle.json \
  --database data/rail_deployment/preview.sqlite3 --port 8790
```

Visit `http://127.0.0.1:8790`. All commands are simulated, at 20x display speed.
The graph shows the real frozen joint arrays; task progress does not represent
hardware telemetry. No cloud callbacks are sent. Task IDs, mapping and ownership
state persist in SQLite. A restart enters unknown ownership / unconfirmed stop.
For a new DB only, `--simulate-boot` demonstrates boot initialization.

Tests:

```bash
python3 -m unittest discover -s deployment/rail_jetson -p 'test_*.py' -v
```

## Required production handoff

1. Reject new ELN requests; finish or stop the existing task and await action
   termination AND fresh stationary hold. Failure retains the occupied slot.
2. Confirm physical support/unloading before releasing arm support. Holding a
   task STOP is NOT equivalent to stopping LowCmd publication or disarming.
3. Jetson gateway acknowledges output release. Issue a generation-bound one-use
   grant for WSL. WSL launch must require that grant before enabling its gateway.
4. While WSL owns output, Jetson remains observation-only. Missing heartbeat or
   network connection MUST NOT trigger automatic Jetson takeover.
5. WSL acknowledges actual output release before Jetson can acquire. Prepare
   from fresh measured joints after acquisition; never replay cached startup.

This simulator exercises the state contract and failure cases, but its simulated
release acknowledgements are not proof of hardware ownership. The existing WSL
launch script is unchanged and does NOT yet enforce these grants. The real
hand_service remains the sole serial owner throughout either control mode.

## Timeout and recovery policy

- Do not widen LowState watchdog, torque or motor fault limits for UI/network lag.
- Separate task-envelope timeout, segment duration, endpoint settling, hand RPC
  and hardware freshness. Derive task deadlines from the sum of frozen segment
  durations plus explicit finite budgets for joins, gain switching and settling.
- A missing web sample only affects displayed freshness; it is not a hardware
  STOP. A lost action result is not successful completion.
- Retry read-only status queries within a bounded grace interval. Never resend
  an uncertain movement or finger command because its acknowledgement timed out.
- Cancellation/timeout cannot release task ownership until stop is confirmed.
  Expose unconfirmed stop as a separate recoverable UI state, not READY.
- Recovery confirmation does not clear motor faults or automatically open hands.
  Production recovery requires fresh controller/hand evidence as well as the
  operator's contact/support inspection. The simulator does not acquire evidence.

## ARM64 parity scope

`interpolation_check.cpp` compiles the original gateway interpolation method
extracted verbatim into `interpolation.inc`. `parity.py` supplies the same
successful arm segments, including reference joins, sampled at <=10 ms spacing
and every knot. Both architectures output position, velocity and acceleration.
An offline numerical match does not certify closed-loop timing, friction,
contact force, gains, watchdogs, gain switching, or a physical deployment.

Completed comparison: 16 segments, 10,799 samples. Maximum absolute differences:
position 1.1102230246251565e-15 rad; velocity 2.846334279382745e-14 rad/s;
acceleration 3.637982276538665e-12 rad/s^2, within the 1e-10 threshold.
The report is `data/rail_deployment/20260921_offline_v1/parity_report.json`.

`dual_prepare.py` additionally reuses the existing joint-line and limit-retiming
helpers to create a 15-axis simultaneous reference preparation. From the selected
historical session home it takes 9.428550532 s and passes 1,920 sampled motion
limit checks. It is marked non-executable: current live gateway admission still
rejects dual-arm trajectories, and simultaneous-arm clearance is not validated.

## Native gateway build

The gateway and state bridge were successfully compiled in the robot's private
workspace at:

`/home/unitree/rh56dfx/workspace/execution/rail_migration_offline_v1/native_ws`

`stage_native.py` stages source-only packages. It preserves the actual WSL
control_msgs 4.9.0 definitions in a PRIVATE Foxy overlay rather than changing
system packages. Unitree message definitions are copied unchanged. Cross-distro
live interoperability still requires a separate test; compilation is not proof.

Two isolated source compatibility changes were needed: KDL's `Fixed` uses its
equivalent old name `None`; `foxy_compat.hpp` prevents Foxy's optional statistics
collector from treating the Unitree RPC header as a ROS timestamp header. Neither
changes LowState/service watchdogs, control gains, trajectories or limits.
The migration copy includes a headless CMake recipe with explicit URDF linkage.
The original WSL source, installed controller and launch scripts are unchanged.

The ARM64 ELF links with no unresolved `ldd` dependencies under this overlay.
`native_smoke.py` checks startup only on domain 174 / loopback, hardware output
disabled and no command publishers. It never sends an arm/motion request.
The smoke test passed with the original URDF and payload files; its process was
terminated after verification. The initial attempt without gravity models was
correctly rejected by the gateway's bilateral-support requirement, which was
not bypassed. Report: `data/rail_deployment/20260921_offline_v1/native_smoke_report.json`.

Final offline verification: 19 unittest cases passed. Playwright exercised task
initialization, injected failure, the complete recovery sequence and the complete
simulated Jetson/WSL handoff. Desktop 1440px, tablet 768px and mobile 390px were
checked, including nonblank trajectory canvas and page overflow assertions.
Screenshots: `data/rail_deployment/screenshots/`.

## Remaining deployment gates

- Existing gateway rejects 14/15-joint combined-arm trajectories. Simultaneous
  preparation needs an explicitly scoped normal-gain dual-arm extension plus
  validator, interpolation and both-arm hold tests. Not implemented here.
- Native robot: ARM64 Ubuntu 20.04 / Foxy. Gateway compilation now succeeds using
  the private overlay above, without Docker or system package installation.
  No host packages, OS, existing services or user groups have been changed.
  The task hand client and complete Python runtime still need packaging/testing.
- Add the production headless planner/executor and ELN adapter, real stop/output
  acknowledgements, WSL launcher grant enforcement, telemetry and callbacks.
- Persist Linux boot ID and ownership journal; distinguish first full boot from
  a service restart. Unit dependencies alone do not prove the robot is supported.
- Validate preparation paths for both arms, then dry-run the complete service
  before separately authorizing physical commissioning and boot enablement.
