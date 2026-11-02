# G1 upper-body MoveIt planning

This package supports simulation, read-only real-state visualization, and an
explicitly enabled RViz-to-hardware path through the guarded arm SDK gateway.
The default launch remains read-only: trajectory execution and the hardware
gateway are both disabled unless requested with launch arguments.

Build:

```bash
source /opt/ros/humble/setup.bash
cd ~/g1_upper_body_ws
colcon build --packages-select g1_mode6_description g1_upper_body_moveit_config --symlink-install
source install/setup.bash
```

Start RViz and generate the verified left-arm example plan automatically:

```bash
ros2 launch g1_upper_body_moveit_config sim_moveit.launch.py
```

The terminal reports the selected mode, timed waypoint count, total duration,
peak joint velocity and peak joint acceleration. In the unobstructed case AUTO
should select `LINEAR`. RViz animates the published planned path and shows the
demo table in the PlanningScene.

To place a box across the direct TCP path and observe AUTO reject the linear
path and fall back to OMPL:

```bash
ros2 launch g1_upper_body_moveit_config sim_moveit.launch.py demo_obstacle:=true
```

For interactive-only use, disable the automatic request, select `left_arm` or
`right_arm` in the MotionPlanning panel, drag the orange goal marker, and press
Plan:

```bash
ros2 launch g1_upper_body_moveit_config sim_moveit.launch.py auto_plan:=false
```

In simulation, use Plan to inspect trajectories; hardware Execute is available
only in the real launch and only after all guarded gateway steps below.

The endpoint-pose API is available as both `/plan_single_arm` (compatibility)
and `/move_single_arm` using
`g1_upper_body_moveit_config/srv/PlanSingleArm`. Supported modes are `AUTO`,
`LINEAR`, and `OMPL`. `AUTO` tries a collision-checked Cartesian path first and
falls back to OMPL when the linear fraction is incomplete. Set `execute: false`
to plan and visualize only. Set `execute: true` to submit a successful
right-arm plan to the guarded FollowJointTrajectory gateway and wait for the
final result. Planning and execution status are reported separately.

The same API accepts either an absolute or relative TCP target. The optional
`tool_profile` request field selects a fixed `arm_tool0 -> TCP` transform from
`config/tool_profiles.yaml`; an empty value remains backward-compatible with
the identity `tool0` profile. Both Cartesian IK and OMPL goals are computed for
the selected TCP and internally converted back to the corresponding `tool0`
goal. LINEAR mode interpolates the selected TCP first (translation plus
quaternion slerp), then converts every sparse waypoint back to `tool0`; the
default maximum rotational spacing is 0.15 rad. The response exposes
`selected_tool_profile`, `resolved_tcp_target`, and
`resolved_tool0_target`; the older `resolved_target` field remains an alias of
`resolved_tcp_target`.

Profiles are usable only when both `enabled` and `calibrated` are true. The
configuration includes enabled identity profiles for `tool0` and the current
right-hand mount plus disabled examples for a dexterous-hand palm, fingertip,
and camera. The example dimensions are not measurements and are deliberately
rejected until their matrices are replaced and explicitly enabled. The
existing conservative hand collision geometry remains active independently of
which TCP is selected.

With `target_is_relative: true`, `target.header.frame_id` is the reference frame,
the position fields are XYZ displacement in metres along that frame's axes,
and the quaternion is a delta rotation of the selected TCP about those frame
axes. An all-zero relative quaternion is treated as identity. At request time
the target is frozen into MoveIt's fixed planning frame. Relative commands are
initially bounded to 0.20 m translation and 60 degrees rotation per request.

For example, plan a 3 cm upward right-arm motion in `torso_link` coordinates:

```bash
ros2 service call /plan_single_arm \
  g1_upper_body_moveit_config/srv/PlanSingleArm \
  "{arm: RIGHT, tool_profile: right_hand_mount, target: {header: {frame_id: torso_link}, pose: {position: {x: 0.0, y: 0.0, z: 0.03}, orientation: {w: 1.0}}}, target_is_relative: true, mode: AUTO, linear_step: 0.02, jump_threshold: 0.60, velocity_scaling: 0.20, acceleration_scaling: 0.20, execute: false}"
```

Use `right_tool0` instead of `torso_link` to express the displacement along
the current tool axes. Planning and display do not imply hardware execution.

## Real-robot read-only shadow launch

The real launch feeds all 27 modeled revolute joints from `/lowstate` into
MoveIt and copies successful, collision-checked plans to the shadow executor:

```bash
source ~/unitree_ws/env/setup_unitree_humble_g1.sh
source ~/g1_upper_body_ws/install/setup.bash
ros2 launch g1_upper_body_moveit_config real_shadow_moveit.launch.py
```

Use `use_rviz:=false` for headless validation. By default this launch keeps
`allow_trajectory_execution=false`, does not start the hardware gateway, and
does not create a Unitree command publisher. The shadow trajectory output is
display/diagnostics only. `enforce_tracking_error` defaults to false because
the dry-run target is expected to move while the physical arm stays still.

When `show_planned_overlay:=true` (the default), RViz displays the measured
robot and a semi-transparent planned-reference robot together. The measured
model follows `/joint_states`; the reference model follows
`/g1/arm_gateway/command_preview` overlaid on the measured full-body state and
is published under the `planned/` TF prefix.

For an approved hardware session, enable RViz execution and start the guarded
gateway explicitly:

```bash
ros2 launch g1_upper_body_moveit_config real_shadow_moveit.launch.py \
  enable_rviz_interactive_execution:=true \
  start_hardware_gateway:=true \
  hardware_output_enabled:=true \
  hardware_authorization:=I_UNDERSTAND_ARM_SDK_OUTPUT
```

This still starts unarmed and creates no `/arm_sdk` publisher. After checking
the live diagnostics and clearing the workspace, a separate call to
`/g1/arm_gateway/arm` is required. RViz Execute then uses
`/right_arm_controller/follow_joint_trajectory`; the gateway revalidates the
right-arm trajectory and applies all configured joint, velocity, acceleration,
tracking, motor-mode, temperature, torque, service-state, and state-timeout
guards. Every real arm takeover or trajectory must still receive an explicit
operator confirmation.

The `G1 Endpoint` panel captures the first complete right-arm `/joint_states`
sample after RViz starts. **Set Goal: Startup Pose** restores that fixed seven-
joint pose as the MoveIt goal while leaving the planning start at the current
measured state. The built-in MoveIt "same as start" meaning is unchanged.

The standard RViz `Plan` button uses the configured OMPL/RRTConnect pipeline.
The `G1 Endpoint` panel adds `AUTO Plan` and `AUTO Execute`: it reads the
current `EE:goal_right_tool0` interactive marker, tries collision-checked
Cartesian LINEAR first, and falls back to OMPL. `AUTO Plan` stores the exact
trajectory and `AUTO Execute` executes that saved trajectory. Moving the marker
after planning invalidates it. The panel's `AUTO Velocity Scaling` and
`AUTO Accel Scaling` controls are applied when `AUTO Plan` time-parameterizes
the LINEAR path or configures the OMPL fallback. Changing either value
invalidates the saved plan, so `AUTO Execute` always runs the exact trajectory
and timing shown by the latest successful plan. The RViz `right_arm` planning
group synchronizes all seven right-arm joints, but does not include the waist
or left arm.

LINEAR interpolates TCP translation on a straight line and orientation with
quaternion slerp. Time parameterization uses TOTG followed by Ruckig smoothing
with a 5 rad/s^3 joint jerk limit. The OMPL fallback uses the same Ruckig
post-processing adapter. The hardware gateway reconstructs timed segments with
quintic Hermite interpolation when position, velocity, and acceleration are
available, preserving acceleration continuity at waypoint boundaries.

RViz goal interaction enables collision-aware approximate IK and uses a 0.20 s
KDL search budget. This keeps the orange goal robot responsive while rotating
near the edge of the exact orientation workspace. Approximate IK affects only
the interactive goal preview and standard RViz joint-state goal selection;
`AUTO Plan` still validates the marker's exact pose and may reject a pose that
cannot be reached exactly. External-program communication is enabled so the
panel's startup-pose goal command reaches the MoveIt display.

The RViz absolute limits are now 0.60 rad/s and 1.50 rad/s^2. Scaling 0.5
therefore matches the previous scaling-1.0 ceiling. Use scaling 0.5 for the
first hardware retest.

## Unified real endpoint-motion server

For headless operation, start the state, planning, AUTO fallback,
tool-profile, execution, gravity-feedforward, and safety stack with one command:

```bash
/home/syf/g1_upper_body_ws/scripts/run_g1_endpoint_motion_server.sh eth0
```

Add `--rviz` only when visualization is wanted:

```bash
/home/syf/g1_upper_body_ws/scripts/run_g1_endpoint_motion_server.sh eth0 --rviz
```

The launcher asks for `ARM` exactly once, anchors the measured pose, waits for
the gravity blend, and then remains resident. Each endpoint request explicitly
chooses `execute: false` or `execute: true`, so a plan-only request cannot move
the robot. Ctrl+C requests a controlled gateway release before shutting down.

For example, after the server is active, plan and execute a relative 1 cm
motion in `torso_link` coordinates without RViz:

```bash
ros2 service call /move_single_arm \
  g1_upper_body_moveit_config/srv/PlanSingleArm \
  "{arm: RIGHT, tool_profile: tool0, target: {header: {frame_id: torso_link}, pose: {position: {z: 0.01}, orientation: {w: 1.0}}}, target_is_relative: true, mode: AUTO, linear_step: 0.02, jump_threshold: 0.60, velocity_scaling: 0.20, acceleration_scaling: 0.20, execute: true}"
```

Hardware execution defaults to at most 60% of the MoveIt velocity limits and
100% of the acceleration limits, corresponding to the current gateway limits.
Requests above those execution scalings are rejected rather than silently
clamped.

After native L2+B damping, the shadow executor remains latched while the arm
motor modes differ from the configured normal mode. Do not change
`expected_arm_motor_mode` merely to bypass this latch. Restore the robot's
normal native mode first, verify the physical area is safe, and then explicitly
call `/g1/shadow_executor/reset` before another shadow run.
