#!/usr/bin/env python3
"""Replay the F3 deadman and joint-limit guards against an admittance log."""

import csv
import math
import sys


JOINTS = tuple(range(22, 29))
LOWER = (-3.0892, -2.2515, -2.618, -1.0472, -1.972222054,
         -1.614429558, -1.614429558)
UPPER = (2.6704, 1.5882, 2.618, 2.0944, 1.972222054,
         1.614429558, 1.614429558)
MARGIN = math.radians(8.0)
MAX_ERROR = tuple(math.radians(x) for x in (1.5, 1.5, 1.5, 2.0, 2.0, 2.0, 2.0))
DEADBAND = (0.08, 0.06, 0.08, 0.06, 0.06, 0.05, 0.04)
VELOCITY_PER_TORQUE = (0.60, 0.80, 0.70, 0.80, 1.30, 1.00, 1.20)
MAX_VELOCITY = (0.20, 0.20, 0.20, 0.25, 0.25, 0.30, 0.30)
MAX_ACCELERATION = (0.60, 0.60, 0.60, 0.80, 1.20, 1.00, 1.00)
PASSTHROUGH = (0.65, 0.65, 0.65, 0.65, 0.35, 0.65, 0.65)
RELEASE_SECONDS = 0.06
PREDICTION_LATENCY = 0.12
BRAKE_ACCELERATION = 3.0
MAX_COMMAND_VELOCITY = 0.35


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def load_rows(path):
    with open(path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError("empty replay log")
    origin = int(rows[0]["stamp_ns"])
    for row in rows:
        row["time"] = (int(row["stamp_ns"]) - origin) / 1e9
    return rows


class DeadmanReplay:
    def __init__(self, row):
        self.state = "HOLD"
        self.state_since = row["time"]
        self.rearm_required = True
        self.released_since = row["time"]
        self.q_ref = [float(row[f"j{joint}_q"]) for joint in JOINTS]
        self.target_dq = [0.0] * 7
        self.engaged = [False] * 7
        self.intent_active = False
        self.guard = None
        self.last_time = row["time"]

    def transition(self, state, now, cause):
        if state == self.state:
            return
        self.state = state
        self.state_since = now
        if state == "BRAKE":
            self.rearm_required = True
            self.target_dq = [0.0] * 7
            self.engaged = [False] * 7
            self.intent_active = False
            if self.guard is None:
                self.guard = (now, cause)

    def step(self, row, deadman):
        now = row["time"]
        dt = clamp(now - self.last_time, 0.0, 0.05)
        self.last_time = now
        if not deadman:
            if now - self.released_since >= RELEASE_SECONDS:
                self.rearm_required = False
        else:
            self.released_since = now

        if self.state == "HOLD" and deadman and not self.rearm_required:
            self.q_ref = [float(row[f"j{joint}_q"]) for joint in JOINTS]
            self.target_dq = [0.0] * 7
            self.engaged = [False] * 7
            self.intent_active = True
            self.transition("DRAG", now, "F3 pressed")
        elif self.state == "DRAG" and not deadman:
            self.transition("BRAKE", now, "F3 released")

        if self.state == "DRAG":
            self.intent_active = deadman
            for local, joint in enumerate(JOINTS):
                q = float(row[f"j{joint}_q"])
                dq = float(row[f"j{joint}_dq"])
                stop = q + dq * PREDICTION_LATENCY + math.copysign(
                    dq * dq / (2.0 * BRAKE_ACCELERATION), dq)
                if stop <= LOWER[local] + MARGIN or stop >= UPPER[local] - MARGIN:
                    self.transition("BRAKE", now, f"j{joint} predicted limit")
                    break

        for local, joint in enumerate(JOINTS):
            filtered = float(row[f"j{joint}_filtered"])
            dq = float(row[f"j{joint}_dq"])
            if self.state != "DRAG":
                self.engaged[local] = False
            elif not self.engaged[local] and abs(filtered) > DEADBAND[local]:
                self.engaged[local] = True
            elif self.engaged[local] and abs(filtered) < DEADBAND[local] * 0.5:
                self.engaged[local] = False
            active = (math.copysign(max(abs(filtered) - DEADBAND[local] * 0.5, 0.0),
                                    filtered) if self.engaged[local] else 0.0)
            requested = (clamp(VELOCITY_PER_TORQUE[local] * active,
                               -MAX_VELOCITY[local], MAX_VELOCITY[local])
                         if self.state == "DRAG" else 0.0)
            acceleration = (BRAKE_ACCELERATION if self.state == "BRAKE"
                            else MAX_ACCELERATION[local])
            self.target_dq[local] += clamp(requested - self.target_dq[local],
                                           -acceleration * dt, acceleration * dt)
            command_dq = 0.0
            if self.state == "DRAG":
                command_dq = self.target_dq[local]
                if self.engaged[local]:
                    command_dq += PASSTHROUGH[local] * dq
                command_dq = clamp(command_dq, -MAX_COMMAND_VELOCITY,
                                   MAX_COMMAND_VELOCITY)
                self.q_ref[local] += command_dq * dt
                # One deliberately moved joint can mechanically couple several
                # arm axes. Keep every coupled reference synchronized during a
                # short arm-level intent window instead of dropping individual
                # axes merely because their residual-torque gates flicker off.
                if self.intent_active:
                    q = float(row[f"j{joint}_q"])
                    self.q_ref[local] = clamp(self.q_ref[local],
                                              q - MAX_ERROR[local],
                                              q + MAX_ERROR[local])
            self.q_ref[local] = clamp(self.q_ref[local],
                                      LOWER[local] + MARGIN,
                                      UPPER[local] - MARGIN)


def run(rows, deadman_for):
    replay = DeadmanReplay(rows[0])
    history = []
    for row in rows:
        replay.step(row, deadman_for(row["time"]))
        history.append(tuple(replay.q_ref))
    return replay, history


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} INCIDENT_CSV", file=sys.stderr)
        return 2
    rows = load_rows(sys.argv[1])
    incident = [row for row in rows if row["time"] >= rows[-1]["time"] - 2.5]

    hold, hold_history = run(incident, lambda _: False)
    hold_drift = max(abs(hold_history[-1][j] - hold_history[0][j]) for j in range(7))
    hold_ok = hold.state == "HOLD" and hold_drift < 1e-12
    print(f"NO_DEADMAN state={hold.state} q_ref_drift={hold_drift:.9f} safe={int(hold_ok)}")

    startup_held, startup_history = run(incident, lambda _: True)
    startup_drift = max(abs(startup_history[-1][j] - startup_history[0][j])
                        for j in range(7))
    startup_ok = startup_held.state == "HOLD" and startup_drift < 1e-12
    print(f"STARTUP_F3_HELD state={startup_held.state} q_ref_drift="
          f"{startup_drift:.9f} safe={int(startup_ok)}")

    press_at = incident[0]["time"] + 0.10
    release_at = incident[0]["time"] + 0.70
    released, released_history = run(
        incident, lambda now: press_at <= now < release_at)
    release_index = next(i for i, row in enumerate(incident) if row["time"] >= release_at)
    release_drift = max(abs(released_history[-1][j] - released_history[release_index][j])
                        for j in range(7))
    release_ok = released.state in ("BRAKE", "HOLD") and release_drift < 1e-12
    print(f"F3_RELEASE state={released.state} post_release_q_ref_drift="
          f"{release_drift:.9f} safe={int(release_ok)}")

    held, _ = run(incident, lambda now: now >= press_at)
    guard_ok = held.guard is not None
    if held.guard:
        guard_time, cause = held.guard
        guard_row = min(incident, key=lambda row: abs(row["time"] - guard_time))
        j26 = float(guard_row["j26_q"])
        before_safe_boundary = j26 > LOWER[4] + MARGIN
        guard_ok &= before_safe_boundary
        print(f"HELD_F3 guard={cause} j26_deg={math.degrees(j26):.2f} "
              f"safe_boundary_deg={math.degrees(LOWER[4] + MARGIN):.2f} "
              f"safe={int(guard_ok)}")
    else:
        print("HELD_F3 guard=NONE safe=0")

    passed = hold_ok and startup_ok and release_ok and guard_ok
    print("RESULT", "PASS" if passed else "FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
