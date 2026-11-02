# G1 KDL IK v1

This package solves either G1 arm from a target `Pose` in `torso_link` using
MoveIt's KDL plugin. It only computes joint positions; it does not publish to
`/arm_sdk` and does not perform collision checking or trajectory generation.

Build and run the offline FK-to-IK check:

```bash
source /opt/ros/humble/setup.bash
cd ~/g1_upper_body_ws
colcon build --packages-select g1_kdl_ik
source install/setup.bash
ros2 launch g1_kdl_ik kdl_ik.launch.py run_self_test:=true
```

The server exposes the standard MoveIt service `/compute_ik`. Valid groups are
`left_arm` and `right_arm`; the target frame is `torso_link`. The request seed
should contain the seven current arm joint positions so the redundant 7-DOF
solution stays close to the robot's current posture.

## Smooth RViz dual-arm demo

Start the integrated model, IK server, joint-state trajectory publisher, and RViz:

```bash
source /opt/ros/humble/setup.bash
source ~/g1_upper_body_ws/install/setup.bash
ros2 launch g1_kdl_ik visualize_ik.launch.py
```

In another terminal, call `/solve_dual_arm_ik`. Both poses are in `torso_link`.
Set either solve flag to false to leave that arm where it is. The server applies
quintic interpolation at 50 Hz. `duration` is a requested minimum; it is
automatically increased when needed to obey `max_joint_speed` from the launch.

The following target pair is known to be reachable and is useful as a first test:

```bash
source /opt/ros/humble/setup.bash
source ~/g1_upper_body_ws/install/setup.bash

ros2 service call /solve_dual_arm_ik g1_kdl_ik/srv/SolveDualArmIK \
"{left_target: {position: {x: 0.288462768, y: 0.239756091, z: 0.059391637}, orientation: {x: 0.145712255, y: 0.087035953, z: 0.118936094, w: 0.978287732}}, right_target: {position: {x: 0.289667479, y: -0.180203919, z: 0.043871191}, orientation: {x: -0.103537391, y: 0.115411312, z: 0.103497625, w: 0.982470600}}, solve_left: true, solve_right: true, duration: 3.0}"
```

This is a visualization trajectory, not collision-aware motion planning and not
a command to the physical robot.

## Reachability map and exact-pose query

Start the robot, both reachability point clouds, the query service, and RViz:

```bash
source /opt/ros/humble/setup.bash
source ~/g1_upper_body_ws/install/setup.bash
ros2 launch g1_kdl_ik reachability.launch.py
```

The map topics are `/reachability/left` and `/reachability/right`. Query a pose
in `torso_link` with:

```bash
ros2 service call /query_reachability g1_kdl_ik/srv/QueryReachability \
"{arm: left, target: {position: {x: 0.288462768, y: 0.239756091, z: 0.059391637}, orientation: {x: 0.145712255, y: 0.087035953, z: 0.118936094, w: 0.978287732}}, search_radius: 0.02, ik_timeout: 0.1}"
```

`voxel_found` means the position map supplied a nearby seed. `ik_success` means
KDL subsequently solved the complete target position and orientation. A found
voxel alone is not proof that the requested orientation is reachable.

For a one-time exhaustive FK consistency check of every stored seed, launch
headless with `validation_stride:=1`. Normal startup checks every 101st record:

```bash
ros2 launch g1_kdl_ik reachability.launch.py \
  use_rviz:=false validation_stride:=1
```

## Capability map v1: automatic global envelope and orientation richness

The v1 capability map is separate from the legacy 5 mm position map. It uses
10 mm position voxels and records 42 approximately uniform directions for the
`tool0` +X axis, with 8 roll bins per direction. It also runs support-direction
optimization to find the global workspace envelope without a hand-written
Cartesian bounding box. Collision filtering is not enabled in this first
visual iteration.

Generate the map once:

```bash
source /opt/ros/humble/setup.bash
source ~/g1_upper_body_ws/install/setup.bash
ros2 launch g1_kdl_ik generate_capability_v1.launch.py
```

Then inspect it with the joint sliders and RViz:

```bash
ros2 launch g1_kdl_ik capability_v1.launch.py
```

The translucent clouds are colored by the number of distinct orientation bins
observed in each position voxel. The larger bright spheres are workspace
boundary points found by directed optimization. Topics are:

- `/capability/left/score`, `/capability/right/score`
- `/capability/left/boundary`, `/capability/right/boundary`

### Query a TCP pose

`tool0` is the rigid mounting frame and `TCP` is the configurable task point.
The default transform is identity. For example, a TCP 10 cm along the left
`tool0` +X axis is configured at launch time with:

```bash
ros2 launch g1_kdl_ik capability_v1.launch.py left_tcp_x:=0.10
```

Query a desired TCP pose in `torso_link`:

```bash
ros2 service call /query_capability g1_kdl_ik/srv/QueryCapability \
"{arm: left, target_tcp: {position: {x: 0.288462768, y: 0.239756091, z: 0.059391637}, orientation: {x: 0.145712255, y: 0.087035953, z: 0.118936094, w: 0.978287732}}, search_radius: 0.03, direction_tolerance: 0.45, roll_tolerance: 0.45, ik_timeout: 0.05, max_seed_attempts: 8}"
```

The service transforms the requested TCP pose back to `tool0`, checks nearby
position/orientation bins, tries several stored seeds with exact KDL IK, and
reports the achieved TCP error. RViz shows translucent axes for the requested
TCP and opaque axes for the achieved TCP. With the default
`use_joint_gui:=false`, a successful query also starts a smooth, visualization-
only joint trajectory. It does not publish to `/arm_sdk` or command the robot.

For manual joint-slider inspection instead, launch with `use_joint_gui:=true`;
solution animation is disabled in that mode to avoid two `/joint_states`
publishers fighting each other.

### RViz interactive TCP targets

On startup, yellow six-DOF targets are placed at the current left and right TCP
poses. Select RViz's **Interact** tool, drag the arrows/rings to change position
or orientation, then right-click the yellow sphere and choose:

- `Query and solve` to run the capability lookup, exact KDL IK, and RViz motion.
- `Reset to current TCP` to place the target back on the displayed hand.

Dragging only edits the target and never commands a robot. A successful query
shows a larger translucent target frame/yellow origin and a shorter opaque
achieved frame/green origin. A failed query leaves the hand still and changes
the target origin to red. The terminal service and interactive targets use the
same query implementation.

The full capability and optimized-boundary clouds are disabled in the normal
interactive view. They remain available as diagnostic layers with:

```bash
ros2 launch g1_kdl_ik capability_v1.launch.py publish_debug_clouds:=true
```

Then manually enable the four disabled capability displays in RViz.
