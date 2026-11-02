# G1 Unitree read-only and shadow integration

This package contains a read-only state bridge, a native safety observer, a
shadow-only trajectory executor, and a guarded future `/arm_sdk` gateway. The
checked-in configuration creates no `/arm_sdk` publisher, and no component in
this package publishes `/lowcmd` or a Unitree motor-control request. The guarded
gateway does issue the read-only RobotState `ServiceList` query (API 1003) so it
can reject takeover when an onboard controller conflicts with arm SDK control.

Build with the Unitree message workspace sourced:

```bash
source ~/unitree_ws/env/setup_unitree_humble_g1.sh
cd ~/g1_upper_body_ws
colcon build --packages-select g1_unitree_control --symlink-install
source install/setup.bash
```

Run against the high-frequency real-robot state:

```bash
ros2 launch g1_unitree_control arm_state_bridge.launch.py
```

Outputs:

- `/g1/arm_joint_states`: 14 arm joints as `sensor_msgs/msg/JointState` with
  reliable output QoS.
- `/g1/arm_state_bridge/diagnostics`: bridge health and source counters.

The output is intentionally separate from `/joint_states`. After mapping and
sign validation, pass `output_topic:=/joint_states` only in a launch where no
other joint-state publisher is running.

## Native L2+B safety monitor

`g1_safety_monitor` is a strictly read-only observer. It has one `/lowstate`
subscription and no application publishers or control clients. (ROS 2 still
creates its standard `/rosout` and `/parameter_events` endpoints.) It decodes
the native controller key mask (`L2+B == 0x0220`) and records arm position,
velocity, estimated torque, motor mode, and robot mode before and after the
button chord:

```bash
ros2 run g1_unitree_control g1_safety_monitor
```

The robot must be physically supported before testing native damping behavior.

## Remote right-arm teaching controller

The direct-DDS remote teaching controller keeps the official high-level motion
service responsible for standing while it takes and releases only the arm SDK
weight.  It uses the fitted wrist-camera payload model even when image capture
is disabled:

```bash
/home/syf/g1_upper_body_ws/scripts/run_right_arm_remote_teach.sh eth0
```

- Hold `F1+X` for 0.8 s to start or stop the active right-arm hold controller.
- Hold physical `F3` (reported by this robot as SDK `F2`, DDS mask `0x0080`)
  to enter `DRAG` immediately from the current measured pose. There is no
  intermediate arming delay. Releasing it actively brakes and holds the arm.
- In `HOLD`/`BRAKE`, right-arm tracking error does not latch the controller:
  the reference remains at the last dragged pose and the arm returns there
  after the operator releases it. Torque, speed, state and motor protections
  remain active.
- Every new `F3` press anchors a fresh drag at the measured pose and resets
  virtual velocity, effort gates and transient compensation. The residual
  torque zero-bias is frozen after initial zeroing so HOLD recovery forces are
  not learned into later drag sessions. While `F3` remains held, it is itself
  the arm-level operator intent and all mechanically coupled axes synchronize
  their references. Per-joint residual-torque gates affect only the admittance
  velocity response and can no longer send the whole arm to `BRAKE/HOLD`.
  The independent 6 degree instantaneous guard remains latching.
- Press `F1+A` while admittance is active to append a timestamped joint snapshot.
- `L2+B` remains the native emergency chord and immediately latches DDS output
  silent; an emergency latch never automatically resumes.
- Normal `F1+X` exit restores normal gains, removes fitted feedforward, ramps the
  arm SDK weight to zero, destroys the publisher, and returns to read-only
  standby.  Another `F1+X` hold may start a new numbered session.

Each session writes a 50 Hz `*_sessionNNN.csv` and a corresponding
`*.captures.csv`.  Capture rows contain right-arm position, velocity, estimated
torque, command target, and temperature.  The reserved image columns report
`NOT_CONFIGURED` until a non-blocking camera capture sink is connected.

Before the first hardware run on a firmware version, verify that the onboard
motion service does not consume the proposed chords:

```bash
/home/syf/unitree_sdk2/build/bin/g1_remote_key_check eth0
```

This key checker has DDS subscribers only and never creates a command publisher.

## Shadow trajectory executor

`shadow_trajectory_executor` consumes real `/lowstate` and a one-arm
`trajectory_msgs/msg/JointTrajectory`, but only publishes a synthetic
`sensor_msgs/msg/JointState` on
`/g1/shadow_executor/commanded_joint_states`. Its diagnostic hardware ID is
`NO_HARDWARE_OUTPUT`.

The input is rejected unless it contains exactly the seven joints of one arm,
starts near the measured arm pose, respects guarded URDF position limits, and
stays below the configured velocity and acceleration limits. The executor
latches on L2+B, stale state, unexpected robot mode, or any arm motor mode that
does not match `expected_arm_motor_mode`. A latch never clears automatically;
`/g1/shadow_executor/reset` succeeds only after fresh state and normal modes
have returned.

The default limits and topics are in `config/shadow_executor.yaml`. This node
is a dry-run safety gate, not a hardware controller.

## Guarded `/arm_sdk` hardware gateway

`arm_sdk_hardware_gateway` implements the future Unitree high-level arm command
path while keeping hardware output structurally absent by default. It follows
the installed official `g1_arm_sdk_dds_example` contract:

- `unitree_hg/msg/LowCmd` on `/arm_sdk` at 100 Hz;
- G1 7-DOF arm motor indices 15 through 28;
- upper-body blend weight in `motor_cmd[29].q`;
- no CRC calculation on this high-level path (CRC is used by the separate
  `/lowcmd` low-level example).

Start the safe default:

```bash
source ~/unitree_ws/env/setup_unitree_humble_g1.sh
source ~/g1_upper_body_ws/install/setup.bash
ros2 launch g1_unitree_control arm_sdk_hardware_gateway.launch.py
```

With the checked-in configuration, the node does **not** create an `/arm_sdk`
publisher. It subscribes to `/lowstate`, `/g1/arm_gateway/trajectory`, and the
RobotState API response; it publishes command previews, diagnostics, and a
periodic read-only ServiceList request. Calling `/g1/arm_gateway/arm` is
rejected in this mode. Note that `ros2 topic list` may still show `/arm_sdk`
because the robot-side `g1_arm_example` service subscribes to it; use the
gateway node information and `hardware_output_available` diagnostic to verify
whether this process has a command publisher.

Hardware-capable startup requires both an explicit startup boolean and an
exact authorization phrase; it still starts unarmed, creates no `/arm_sdk`
publisher, and requires the arm service. The publisher is created only after
the final state/service/tau-anchor checks in the arm callback, matching the
previously hardware-tested remote-teach program. These startup-only gates are
intentionally not enabled by any launch file in the MoveIt package.

On G1, developer/debug mode is the intended SDK-control environment: it stops
the built-in motion service from competing for the joints. `mode_machine` and
the individual motor `mode` bytes are recorded for diagnostics, but are not
treated as a normal/debug authorization flag. Before arming, the gateway instead
requires a fresh RobotState ServiceList with `g1_arm_example` and `ai_sport`
running, while the conflicting test services (`auto_test_arm` and
`auto_test_low`) remain stopped. This is the official high-level `/arm_sdk`
path used while the main motion service keeps the lower body standing. On the
firmware validated here, service status `0` means running and `1` means stopped.

The gateway accepts exactly seven joints from one arm and checks:

- fresh, finite `/lowstate` and a fresh, conflict-free onboard ServiceList;
- released L2+B and a manually reset safety latch;
- measured upper-body velocity, estimated torque, motor temperature, drive
  status, and voltage;
- guarded URDF joint limits and trajectory start error;
- declared and derived segment velocity/acceleration;
- maximum per-cycle target change;
- warning, pause, and hard tracking-error thresholds.

For bumpless right-arm takeover, the hardware gateway keeps approximately two
seconds of 100 Hz `tau_est` history. Arming is rejected unless the upper body is
stationary and each anchor has enough samples, acceptable spread, and remains
inside its per-joint feedforward limit. Like the proven remote-teach path, the
first command uses weight 1, arm gains 60/1.5, the measured joint pose, and the
measured torque anchor. That anchor is held for two seconds before torque alone
blends over another two seconds to a dynamic KDL gravity model plus the fitted
payload in `right_arm_camera_fit_20260824_185416.yaml`. The model is
evaluated from measured joint positions and IMU roll/pitch/yaw on every control
cycle. Direct torque is multiplied by the official arm SDK blend weight, as in
the hardware-tested SDK probe. The left arm, which is held rather than moved by
right-arm trajectories, retains its measured static torque anchor. The gateway
also estimates feedforward-plus-PD torque, including the bounded waist integral
torque described below. Shoulder, elbow, and wrist-roll
joints use 20 Nm continuous and 25 Nm short-peak limits; wrist pitch/yaw use
4 Nm continuous and 5 Nm short-peak limits. The peak window is one second.

Tracking error slows trajectory time to 25 percent at the warning threshold
and pauses it at the pause threshold. Crossing the former hard tracking limit,
sustained moderate measured velocity, a per-cycle target jump, or a normal PD
command-limit violation now aborts the trajectory into damped recovery instead
of destroying support. Recovery keeps `/arm_sdk`, gravity feedforward, and the
measured pose active with gains 10/3 until velocity remains below 0.10 rad/s for
0.5 seconds. A three-second recovery timeout raises a persistent warning but
does not silence output. Persistent or peak arm torque stops the trajectory and
enters the same supported recovery instead of destroying `/arm_sdk`. Gravity
feedforward is saturated at the continuous joint limit and reports a warning;
only non-finite feedforward data is latched. The 85 C temperature threshold is
an alarm while armed so the arm can move to a lower-load pose, while a hot robot
cannot be newly armed. Strong recovery velocity, instantaneous velocity,
motor/service/state faults, and native L2+B still latch output silent. A
requested normal disarm holds the measured pose while ramping the official
blend weight down over two seconds.

Configuration is in `config/arm_sdk_hardware_gateway.yaml`. Current interactive
limits are 0.60 rad/s, 1.50 rad/s^2, and 0.006 rad per 10 ms cycle, so scaling
0.5 has the old scaling-1.0 ceiling. A quintic start blend aligns each accepted
trajectory from the previous held command to its first point; both held and
measured starts must pass the existing tolerance and joint-limit checks.
Waypoint segments use quintic
Hermite interpolation when MoveIt supplies velocity and acceleration. Motor 26 (right wrist
roll) uses gains 40/2.5 instead of the normal arm gains 60/1.5 to reduce the
measured wrist oscillation. Every accepted trajectory records synchronized
target, q, dq, tau_est, tracking error, and progress at the control rate under
`/home/syf/g1_upper_body_ws/logs/endpoint_motion`.
`LowState` does not expose motor current, so `tau_est` is used as the available
software overload proxy; the drive's native over-current protection remains
primary. This 29-DoF robot reports waist roll/pitch motors 13 and 14 inactive,
so only waist yaw motor 12 is included in the command and safety set. Normal
release ramps weight to zero and then destroys the `/arm_sdk` publisher;
latched safety exits destroy it immediately without publishing another frame.

Waist yaw defaults to Kp/Kd/Ki `60/4.5/20` with integral torque limited to
1.2 Nm. The integral state is stored directly in Nm and is added through the
drive's `tau` field. It waits for persistent error outside a 0.002 rad deadband,
uses full gain while holding or detecting low-speed stiction, and reduces gain
while following a moving curve. Measured breakaway, target reversal, excessive
tracking error, release, recovery, and task admittance freeze or bleed/reset
the state. The same integral term is included in command-overload estimation,
diagnostics, and endpoint CSV logs. Runtime Kp/Kd/Ki/limit changes are atomic
and allowed only in guarded idle phases.

Normal trajectory handovers preserve the bounded waist integral, including
arm-only trajectories. They do not re-anchor the command to measured position
or drop supporting integral torque in one tick. Cancellation, STOP, recovery,
release and safety exits still reset the integral immediately.

Tracking slowdown now uses a continuous quintic error-to-time-scale map:
full speed through half the warning error, 0.25 at the warning, and zero at
the original pause threshold. It never exceeds the old piecewise scale.
Feedback acceleration and CSV `*_command_ddq` include the estimated time-scale
derivative term; `*_target_ddq` remains the unscaled path acceleration. This
does not guarantee bounded jerk or acceleration under discontinuous feedback:
pause and safety stops still take priority over smoothing.

Action cancellation stops progression in the cancel callback and completes
the goal only once rclcpp_action has entered CANCELING. Direct gateway cancel
or disarm aborts an executing goal rather than making an invalid CANCELED
transition. Focused offline coverage: `scripts/test_task_control_smoothness.py`
and `scripts/check_task_control_transitions.py` (localhost FastDDS domain 87).
# Right-arm static payload calibration

`right_arm_static_calibrator` is read-only: it subscribes to `/lowstate` and
does not create an arm command publisher. It fits the total mass and center of
mass of everything mounted after `right_wrist_yaw_link` from multiple static
poses.

Start it with the G1 CycloneDDS environment and the package setup sourced:

```bash
ros2 run g1_unitree_control right_arm_static_calibrator \
  --ros-args --params-file \
  /home/syf/g1_upper_body_ws/src/g1_unitree_control/config/right_arm_static_calibrator.yaml
```

After the robot has been stationary for at least 1.5 seconds, capture one pose:

```bash
ros2 service call /g1/right_arm_calibration/capture std_srvs/srv/Trigger '{}'
```

The node rejects moving, noisy, stale, or near-duplicate poses. Collect at
least 8 and preferably 10--12 diverse poses. Request/refine the fit with:

```bash
ros2 service call /g1/right_arm_calibration/fit std_srvs/srv/Trigger '{}'
```

Samples are appended to `calibration/right_arm_static_samples.csv`; fitted
payload parameters are written to `calibration/right_arm_payload_fit.yaml`.

## Guarded right-arm payload calibration run

The hardware-tested SDK2 safety probe is also reused by the multi-pose runner:

```bash
/home/syf/g1_upper_body_ws/scripts/run_right_arm_payload_calibration.sh eth0
```

Before `START`, the program is read-only and performs ServiceList,
SportModeState/FSM, LowState freshness, locked-waist, motor-mode, temperature,
velocity, torque and start-pose checks. It then keeps a 50 Hz `rt/arm_sdk`
hold even while waiting for keyboard input. Every one of the 12 screened poses
requires a separate ENTER. L2+B or any state/safety fault stops publishing
immediately; normal completion returns along the screened path and releases
weight over two seconds.

The run writes a timestamped CSV and identifies payload mass plus COM in
`right_wrist_yaw_link` with least squares. Typing `VALIDATE` after the fit
replays the same screened path using fitted KDL gravity feedforward and captures
repeat-pose checks at poses 1, 6 and 11.
