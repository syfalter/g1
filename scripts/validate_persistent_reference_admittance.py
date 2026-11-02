#!/usr/bin/env python3
"""Offline replay checks for the persistent-reference right-arm admittance."""

import csv
import math
import sys


JOINTS = tuple(range(22, 29))
KP = (2.0, 2.0, 2.0, 2.0, 2.0, 1.5, 1.5)
TAU_LIMIT = (12.0, 12.0, 12.0, 12.0, 12.0, 4.0, 4.0)
MAX_ERROR = tuple(math.radians(x) for x in (1.5, 1.5, 1.5, 2.0, 2.0, 2.0, 2.0))
LOWER = (-3.0892, -2.2515, -2.618, -1.0472, -1.972222054,
         -1.614429558, -1.614429558)
UPPER = (2.6704, 1.5882, 2.618, 2.0944, 1.972222054,
         1.614429558, 1.614429558)
LIMIT_MARGIN = math.radians(5.0)
DT = 0.02


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def quintic(value):
    value = clamp(value, 0.0, 1.0)
    return value * value * value * (10.0 + value * (-15.0 + 6.0 * value))


def load_rows(path):
    with open(path, newline="") as stream:
        rows = [row for row in csv.DictReader(stream)
                if row.get("j28_winding") not in (None, "")]
    if not rows:
        raise RuntimeError("no complete rows in replay log")
    origin = int(rows[0]["stamp_ns"])
    for row in rows:
        row["relative_time"] = (int(row["stamp_ns"]) - origin) / 1e9
    return rows


def replay(rows, initial_reference=None):
    q_ref = ([float(rows[0][f"j{joint}_q"]) for joint in JOINTS]
             if initial_reference is None else list(initial_reference))
    peak_error = [0.0] * 7
    peak_total = [0.0] * 7
    peak_command_velocity = [0.0] * 7
    raw_follow_remaining = [0.0] * 7
    hold_blend = [0.0] * 7
    hold_integral = [0.0] * 7
    final_kp = list(KP)
    final_total = [0.0] * 7
    final_tau_est = [0.0] * 7
    for row in rows:
        for local, joint in enumerate(JOINTS):
            measured = float(row[f"j{joint}_q"])
            dq = float(row[f"j{joint}_dq"])
            filtered_dq = float(row[f"j{joint}_filtered_dq"])
            speed = float(row[f"j{joint}_speed_envelope"])
            virtual_dq = float(row[f"j{joint}_virtual_dq"])
            passthrough = float(row.get(f"j{joint}_passthrough", 0.0))
            kd = float(row[f"j{joint}_kd"])
            tau_ff = float(row[f"j{joint}_tau_ff"])

            motion_blend = quintic((abs(filtered_dq) - 0.28) / (0.45 - 0.28))
            if local < 5 and abs(dq) > 0.40:
                raw_follow_remaining[local] = 0.35
            else:
                raw_follow_remaining[local] = max(
                    0.0, raw_follow_remaining[local] - DT)
            raw_motion_blend = 1.0 if raw_follow_remaining[local] > 0.0 else 0.0
            brake_start = 0.70 if joint == 26 else 0.80
            brake_blend = quintic((speed - brake_start) / (1.20 - brake_start))
            reference_error = measured - q_ref[local]
            capture_dq = motion_blend * (1.0 - brake_blend) * clamp(
                0.80 * filtered_dq + 10.0 * reference_error, -0.40, 0.40)
            command_dq = virtual_dq + passthrough * filtered_dq + capture_dq
            command_dq = clamp(command_dq, -0.75, 0.75)

            effort_engaged = bool(int(float(row[f"j{joint}_engaged"])))
            reference_follow = (effort_engaged or motion_blend > 0.0 or
                                raw_motion_blend > 0.0)
            kp = KP[local]
            if local >= 5:
                hold_target = 0.0 if reference_follow else 1.0
                seconds = 0.25 if hold_target > hold_blend[local] else 0.10
                hold_blend[local] += clamp(
                    hold_target - hold_blend[local], -DT / seconds, DT / seconds)
                kp += hold_blend[local] * (2.5 - KP[local])
                if reference_follow:
                    release_alpha = DT / (0.15 + DT)
                    hold_integral[local] *= 1.0 - release_alpha
                elif hold_blend[local] >= 0.99 and abs(dq) < 0.15:
                    error = q_ref[local] - measured
                    integral_error = math.copysign(
                        max(abs(error) - math.radians(0.25), 0.0), error)
                    hold_integral[local] = clamp(
                        hold_integral[local] + 2.0 * integral_error * DT,
                        -0.18, 0.18)

            q_ref[local] += command_dq * DT
            if reference_follow:
                q_ref[local] = clamp(q_ref[local], measured - MAX_ERROR[local],
                                     measured + MAX_ERROR[local])
            q_ref[local] = clamp(q_ref[local], LOWER[local] + LIMIT_MARGIN,
                                 UPPER[local] - LIMIT_MARGIN)

            # Conservative bound: assume the complete 0.08 Nm breakaway assist
            # on J24/J26 even though it is normally faded before motion.
            breakaway = 0.08 if joint in (24, 26) else 0.0
            total = tau_ff + math.copysign(breakaway, command_dq or 1.0) + \
                hold_integral[local] + kp * (q_ref[local] - measured) + \
                kd * (command_dq - dq)
            peak_error[local] = max(peak_error[local], abs(q_ref[local] - measured))
            peak_total[local] = max(peak_total[local], abs(total))
            peak_command_velocity[local] = max(peak_command_velocity[local],
                                               abs(command_dq))
            final_kp[local] = kp
            final_total[local] = total
            final_tau_est[local] = float(row[f"j{joint}_tau_est"])
    details = {"hold_blend": hold_blend, "hold_integral": hold_integral,
               "kp": final_kp, "total": final_total,
               "tau_est": final_tau_est}
    return q_ref, peak_error, peak_total, peak_command_velocity, details


def release_check(rows, joint, start, end):
    selected = [row for row in rows if start <= row["relative_time"] < end]
    if not selected:
        raise RuntimeError(f"release interval {start}-{end}s has no rows")
    local = joint - 22
    q_ref, _, _, _, _ = replay(selected)
    measured_start = float(selected[0][f"j{joint}_q"])
    measured_end = float(selected[-1][f"j{joint}_q"])
    reference_drift = q_ref[local] - measured_start
    measured_drift = measured_end - measured_start
    restoring_error = q_ref[local] - measured_end
    return measured_drift, reference_drift, restoring_error


def drag_segments(rows, joint):
    """Return historical intervals containing positive drag evidence."""
    evidence = [bool(int(float(row[f"j{joint}_engaged"]))) or
                abs(float(row[f"j{joint}_filtered_dq"])) > 0.28
                for row in rows]
    segments = []
    start = last = None
    for index, active in enumerate(evidence):
        if active:
            if start is None:
                start = index
            last = index
        elif start is not None and index - last > 10:
            if last - start >= 10:
                segments.append((start, last))
            start = last = None
    if start is not None and last - start >= 10:
        segments.append((start, last))
    return segments


def main():
    if len(sys.argv) not in (2, 3, 4):
        print(f"Usage: {sys.argv[0]} REPLAY_CSV [FAULT_REGRESSION_CSV] "
              "[DISTAL_HOLD_CSV]",
              file=sys.stderr)
        return 2
    rows = load_rows(sys.argv[1])
    _, peak_error, peak_total, peak_command_velocity, _ = replay(rows)
    passed = True
    print(f"REPLAY rows={len(rows)} duration={rows[-1]['relative_time']:.2f}s")
    for local, joint in enumerate(JOINTS):
        torque_ok = peak_total[local] < TAU_LIMIT[local]
        velocity_ok = peak_command_velocity[local] <= 0.75 + 1e-9
        passed &= torque_ok and velocity_ok
        print(
            f"STRESS j{joint} open_loop_error_peak_deg="
            f"{math.degrees(peak_error[local]):.3f} total_peak={peak_total[local]:.3f}/"
            f"{TAU_LIMIT[local]:.1f} cmd_dq_peak={peak_command_velocity[local]:.3f} "
            f"safe={int(torque_ok and velocity_ok)}"
        )

    # The old controller generated the measured trajectory, so a 544 second
    # open-loop q_ref replay is not a valid tracking test.  Reset q_ref at each
    # independently observed drag episode and verify that the new controller
    # follows substantial commanded motion without reaching the 6 degree hard
    # guard or any torque/velocity limit.
    for local, joint in enumerate(JOINTS):
        ratios = []
        endpoint_errors = []
        hard_peak = 0.0
        drag_tau_peak = 0.0
        drag_velocity_peak = 0.0
        for start, end in drag_segments(rows, joint):
            selected = rows[start:end + 1]
            measured_start = float(selected[0][f"j{joint}_q"])
            measured_end = float(selected[-1][f"j{joint}_q"])
            measured_motion = measured_end - measured_start
            if abs(measured_motion) < math.radians(3.0):
                continue
            q_ref, errors, totals, velocities, _ = replay(selected)
            reference_motion = q_ref[local] - measured_start
            ratios.append(reference_motion / measured_motion)
            endpoint_errors.append(abs(q_ref[local] - measured_end))
            hard_peak = max(hard_peak, errors[local])
            drag_tau_peak = max(drag_tau_peak, totals[local])
            drag_velocity_peak = max(drag_velocity_peak, velocities[local])
        if not ratios:
            print(f"DRAG j{joint} no qualifying historical episodes safe=0")
            passed = False
            continue
        median_ratio = sorted(ratios)[len(ratios) // 2]
        endpoint_peak = max(endpoint_errors)
        follow_ok = 0.65 <= median_ratio <= 1.15
        endpoint_ok = endpoint_peak <= MAX_ERROR[local] + 1e-9
        hard_ok = hard_peak < math.radians(6.0)
        torque_ok = drag_tau_peak < TAU_LIMIT[local]
        velocity_ok = drag_velocity_peak <= 0.75 + 1e-9
        drag_ok = follow_ok and endpoint_ok and hard_ok and torque_ok and velocity_ok
        passed &= drag_ok
        print(
            f"DRAG j{joint} episodes={len(ratios)} median_follow="
            f"{median_ratio:.3f} endpoint_error_peak_deg="
            f"{math.degrees(endpoint_peak):.3f} transient_error_peak_deg="
            f"{math.degrees(hard_peak):.3f} total_peak={drag_tau_peak:.3f} "
            f"cmd_dq_peak={drag_velocity_peak:.3f} safe={int(drag_ok)}"
        )

    # Previously observed hands-off gravity-creep intervals from this log.
    intervals = ((27, 109.0, 110.0), (27, 117.0, 119.0),
                 (28, 230.0, 236.0), (27, 512.0, 517.0),
                 (28, 536.0, 537.0))
    for joint, start, end in intervals:
        measured, reference, restoring = release_check(rows, joint, start, end)
        reference_ok = abs(reference) <= math.radians(0.20)
        restoring_ok = abs(restoring) >= math.radians(0.50)
        passed &= reference_ok and restoring_ok
        print(
            f"RELEASE j{joint} {start:.0f}-{end:.0f}s measured_drift_deg="
            f"{math.degrees(measured):+.3f} q_ref_drift_deg="
            f"{math.degrees(reference):+.3f} restoring_error_deg="
            f"{math.degrees(restoring):+.3f} safe={int(reference_ok and restoring_ok)}"
        )
    if len(sys.argv) >= 3:
        fault_rows = load_rows(sys.argv[2])
        fault_start = fault_rows[-1]["relative_time"] - 0.40
        selected = [row for row in fault_rows
                    if row["relative_time"] >= fault_start]
        initial = [float(selected[0][f"j{joint}_target_q"])
                   for joint in JOINTS]
        _, errors, totals, velocities, _ = replay(selected, initial)
        local = 26 - 22
        regression_ok = (errors[local] <= MAX_ERROR[local] + 1e-9 and
                         totals[local] < TAU_LIMIT[local] and
                         velocities[local] <= 0.75 + 1e-9)
        passed &= regression_ok
        print(
            f"FAULT_REGRESSION j26 rows={len(selected)} error_peak_deg="
            f"{math.degrees(errors[local]):.3f}/"
            f"{math.degrees(MAX_ERROR[local]):.1f} total_peak="
            f"{totals[local]:.3f}/{TAU_LIMIT[local]:.1f} cmd_dq_peak="
            f"{velocities[local]:.3f} safe={int(regression_ok)}"
        )
    if len(sys.argv) == 4:
        hold_rows = load_rows(sys.argv[3])
        hold_cases = ((27, 253.44, 257.34),
                      (28, hold_rows[-1]["relative_time"] - 1.20,
                       hold_rows[-1]["relative_time"]))
        for joint, start, end in hold_cases:
            selected = [row for row in hold_rows
                        if start <= row["relative_time"] <= end]
            initial = [float(selected[0][f"j{axis}_target_q"])
                       for axis in JOINTS]
            q_ref, errors, totals, velocities, details = replay(selected, initial)
            local = joint - 22
            measured = float(selected[-1][f"j{joint}_q"])
            signed_error = q_ref[local] - measured
            correction = details["total"][local] - details["tau_est"][local]
            restoring_margin = correction * math.copysign(1.0, signed_error)
            hold_ok = (details["hold_blend"][local] >= 0.99 and
                       abs(details["kp"][local] - 2.5) < 1e-9 and
                       abs(details["hold_integral"][local]) <= 0.18 + 1e-9 and
                       restoring_margin >= 0.05 and
                       totals[local] < TAU_LIMIT[local] and
                       velocities[local] <= 0.75 + 1e-9)
            passed &= hold_ok
            print(
                f"DISTAL_HOLD j{joint} {start:.2f}-{end:.2f}s error_deg="
                f"{math.degrees(signed_error):+.3f} kp={details['kp'][local]:.2f} "
                f"integral={details['hold_integral'][local]:+.3f} "
                f"restoring_margin={restoring_margin:.3f} total_peak="
                f"{totals[local]:.3f}/{TAU_LIMIT[local]:.1f} safe={int(hold_ok)}"
            )
    print("RESULT", "PASS" if passed else "FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
