# Gear Hand Update, 2026-09-28

Installed on Jetson, services active, controller DETACHED. No initialization or
physical task was requested during deployment. Receipt:
`data/rail_deployment/gear_hand_policy_20260928/activation_receipt.json`.

## Failure

Task `ELN-20260928-164059-d3fa6bd16f` reached LOWER successfully, then failed during
grip with `Hand posture did not settle; arm must not proceed`. No contact stroke
ran. The optimized exporter omitted WSL landing gear's `hand_target_required=False`;
native execution therefore required all normalized finger errors <=0.03 for three
fresh feedback samples within 15 seconds. Historical failure evidence does not
identify the individual finger error. The user's physical observation was that
the thumb closes over the four fingers, requiring a phased command.

## Installed Behavior

- Grip: four fingers ramp during 0..0.5 s; both thumb channels stay at initial
  values, then ramp during 0.5..1.0 s.
- Open: both thumb channels ramp during 0..0.5 s, then four fingers during 0.5..1.0 s.
- All gear opens, including initial open and recovery exit open, use this order.
- No physical target-position equality requirement. Command acknowledgement,
  advancing healthy telemetry, STOP and hardware-fault checks remain mandatory.
- Three fresh final-command feedback samples precede hold and the next arm step.
  This verifies command completion, NOT grasp, contact or physical finger arrival.
- Both protocol start directions and legacy approved full gear actions are covered.
- Recorded arm arrays, speeds, selected release, other task hand policies and the
  protocol motion-library data hash are unchanged. Future local exports carry the
  completion and ordering metadata explicitly.

## Startup Timing Evidence

Local timestamps from the failed task on September 28:

- 16:40:59.799: ACCEPTED.
- 16:41:02.401: alignment stage announced.
- 16:41:04.126: gateway accepted a 1 s alignment plus 0.118 s start blend.
- 16:41:05.494: open-hand stage announced.
- 16:41:08.550: approach announced.
- 16:41:09.863: gateway accepted the 9.714 s entry_lower recording.
- 16:41:20.160: grip stage announced.
- 16:41:36.668: FAILED.

The approximately 10 s startup latency is before the recorded approach. The first
stored entry_lower point displaced over 0.001 rad occurs at 0.186 s (entry_upper:
0.223 s); no multi-second initial static interval was found. Optimization of
pre-dispatch validation, redundant alignment and phase overhead remains separate
work; this update does not claim to eliminate that latency.

## Verification

Local regression: 96 passed. Staged Jetson runtime: 52 passed. Jetson Python 3.8
mock hand tests: 24 passed before installation and again against installed code.
Suites overlap. Actual finger sequencing remains to be physically tested.

After operator confirms safe clearance: R2+A, explicit initialization, wait for
SUCCEEDED, then submit a new taskId. Failed historical tasks are not replayed.
