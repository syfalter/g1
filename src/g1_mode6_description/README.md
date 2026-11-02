# G1 mode 6 visualization

This package visualizes the official Unitree
`g1_29dof_lock_waist_rev_1_0` model without requiring a powered robot.

Build:

```bash
source /opt/ros/humble/setup.bash
cd ~/g1_upper_body_ws
colcon build --symlink-install
source install/setup.bash
```

Run with RViz and joint sliders:

```bash
ros2 launch g1_mode6_description display.launch.py
```

The fixed RViz frame is `pelvis`. The initial end-effector frames
`left_tool0` and `right_tool0` are coincident with `left_rubber_hand` and
`right_rubber_hand`; their real TCP offsets can be calibrated later.

For a headless smoke test:

```bash
ros2 launch g1_mode6_description display.launch.py use_gui:=false use_rviz:=false
```
