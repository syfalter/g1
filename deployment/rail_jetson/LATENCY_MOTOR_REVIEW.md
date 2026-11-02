# Startup Latency And Motor Review, 2026-09-28

## Activation Status

Latency patch is implemented, locally tested and uploaded to Jetson staging, NOT
activated. The robot was initialized in ARMED_HOLD under Jetson ownership during
inspection. No motion/recovery request or service restart was sent. Activation
requires operator-confirmed safe disable and DETACHED output.

Staging: `data/rail_deployment/latency_motor_20260928/`.

## Startup Changes

Structural release validation is cached after startup/first successful validation.
Every mutable request source still gets a private deep copy and a full content hash
check. Tampering is rejected; changed releases get full validation. The cache holds
one verified hash, not mutable trusted trajectory arrays.

The first `align to pinned task start` segment may be skipped only after the existing
stationary/authority check and when ALL target joints are within 0.005 rad, with finite
values and no missing joints. Other joins are unchanged. Subsequent stage switches,
fresh recorded-start admission and gateway safety checks still run. Out-of-tolerance
states still use the original local join planner.

No recorded points, timing, speed limits, gains or hand ordering changed.

Jetson CPU-only benchmark (three repetitions):

- Previous deep-copy plus full validation: 2.674, 2.795, 2.791 s.
- Cached structure plus deep-copy/content hash: 1.443, 1.607, 1.469 s.

This measures software validation only, not end-to-end task latency. The roughly
1.25 s mean reduction and conditional skipped join do not imply instant motion.

Observed old pipeline:
- Gear task `ELN-20260928-164059-d3fa6bd16f`: accepted 16:40:59.800, recorded approach
  accepted by gateway 16:41:09.863, approximately 10.06 s. Includes initial alignment
  and independent open-hand stage; it is not a 10 s pause inside the recording.
- D-ring task `TASK-20260928-000004`: accepted 17:08:04.826, recording accepted
  17:08:12.256, approximately 7.43 s. No separate initial hand step in its program.

## Latest Completed Motion

The latest completed task at inspection was D-ring pair `TASK-20260928-000004`,
SUCCEEDED at 17:09:27.817 local time. Analysed CSV:
`logs/endpoint_motion_20260928_170812_003.csv` under staging.

- 8462 samples including post-motion hold observation; recording duration 74.605 s.
- Largest estimated joint torque: 7.3125 Nm, left shoulder pitch.
- Largest logged joint position tracking error: 0.009426 rad, passive right wrist yaw.
- Left shoulder pitch peak measured speed: 0.619728 rad/s; its command peak 0.323811.
- Left wrist yaw peak measured speed: 0.613592 rad/s; its command peak 0.398048.
- Longest single-joint sampled interval above 0.5 rad/s: approximately 70 ms.
- Maximum sampled control gap 23.787 ms, feedback age 32.920 ms; time scale stayed 1.
- Current health snapshot: highest motor temperature 59 C, configured alarm 85 C;
  no active motor fault, overload or temperature alarm. CSV does NOT contain temperature.
- Some hold integrators are saturated at their configured compensation bounds; this
  is not a motor over-torque alarm, nor evidence that audible vibration is harmless.

These logs contain neither audio nor measured contact/tool forces. Short velocity
overshoot exists, but its acoustic cause is unconfirmed. Do not infer mechanical
safety solely from absence of alarms. This patch does not increase speed or gains.

## Verification

Local regression 86/86; staged-runtime suite 56/56. Includes hash tampering,
changed-release revalidation, private-copy isolation, alignment boundary checks,
gear hand ordering, automatic recovery and task execution regressions. Suites overlap.
Jetson benchmark used only JSON and CPU; no ROS node or hardware transport was created.
