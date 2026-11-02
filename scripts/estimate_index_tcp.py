"""Generate nominal bilateral RH56DFX index TCP transforms."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from workbench_model import OfflineModel


ROOT = Path(__file__).resolve().parents[1]
INDEX_OPEN = 0.15 / 1.1169
INDEX_FOLDED = 1.1
CONNECTOR_LENGTH = 0.04


def record(transform):
    return {
        "xyz_m": transform[:3, 3].tolist(),
        "quaternion_xyzw": Rotation.from_matrix(transform[:3, :3]).as_quat().tolist(),
        "matrix_row_major": transform.tolist(),
    }


def generate():
    result = {
        "schema_version": 1,
        "model": "RH56DFX community URDF",
        "definition": "index_tcp link origin; estimated geometry, not measured contact center",
        "connector_length_m": CONNECTOR_LENGTH,
        "hardware_calibrated": False,
        "finger_state_note": "DDS actuator feedback is not an exact passive phalanx angle measurement",
        "sides": {},
    }
    base = ROOT / "assets/workbench/rh56dfx/task"
    for side, original_y in (("left", 0.003), ("right", -0.003)):
        model = OfflineModel(base / f"{side}_index.urdf")
        wrist_from_palm = np.eye(4)
        wrist_from_palm[:3, 3] = [0.0415 + CONNECTOR_LENGTH, original_y, 0.0]
        states = {}
        for state, angle in (("open", INDEX_OPEN), ("folded", INDEX_FOLDED)):
            palm_from_tcp = model.transform(
                f"{side}_index_tcp", {f"{side}_index_1_joint": angle}
            )
            states[state] = {
                "index_1_angle_rad": angle,
                "palm_from_index_tcp": record(palm_from_tcp),
                "wrist_yaw_from_index_tcp": record(wrist_from_palm @ palm_from_tcp),
            }
        result["sides"][side] = {
            "wrist_yaw_from_palm": record(wrist_from_palm),
            "states": states,
        }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(generate(), indent=2) + "\n")
    print(args.output)


if __name__ == "__main__":
    main()
