"""Online Cartesian continuation policy; no time limits or robot output."""
from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class ContinuityPolicy:
    max_joint_step_rad: float = .04
    max_subdivision_depth: int = 6
    max_extra_points: int = 240
    slope_weight: float = 2.
    fast_slope_error_rad: float = .01

    def record(self):
        return asdict(self)


def slope_error(positions, candidate, previous_distance, next_distance):
    """Compare joint increments at equal Cartesian spacing, including refined steps."""
    if len(positions) < 2 or min(previous_distance, next_distance) <= 1e-9:
        return np.zeros(len(candidate))
    previous = np.asarray(positions[-1])-np.asarray(positions[-2])
    delta = np.asarray(candidate)-np.asarray(positions[-1])
    # Compare over the smaller interval so subdivision cannot amplify the score.
    distance = min(previous_distance, next_distance)
    return delta*(distance/next_distance)-previous*(distance/previous_distance)


def subdivision_point(previous, target, depth, extra_points, policy):
    previous = np.asarray(previous, dtype=float)
    target = np.asarray(target, dtype=float)
    if (depth >= policy.max_subdivision_depth or extra_points >= policy.max_extra_points or
            np.linalg.norm(target-previous) <= 1e-5):
        return None
    return (previous+target)*.5


def continuation_seed(names, limits, state, positions, previous_distance, next_distance, policy):
    if len(positions) < 2 or min(previous_distance, next_distance) <= 1e-9:
        return None
    delta=(np.asarray(positions[-1])-np.asarray(positions[-2]))*(next_distance/previous_distance)
    largest=float(np.max(np.abs(delta)))
    if largest <= 1e-9:
        return None
    delta*=min(1.,policy.max_joint_step_rad/largest)
    seed=dict(state)
    for name,step in zip(names,delta):
        low,high=limits[name]
        seed[name]=float(np.clip(state[name]+step,low+.02,high-.02))
    return seed
