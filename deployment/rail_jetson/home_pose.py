"""Reviewed upper-body R2+A reference; not a balance or power-off command."""
from bundle import digest

SOURCE = 'task_sessions/20260923_193133_106405/session_home.json'
CAPTURED_AT = '2026-09-23T19:31:44.954133+08:00'
TARGET = {
    'left_shoulder_pitch_joint': 0.24392692744731903,
    'left_shoulder_roll_joint': 0.2780100703239441,
    'left_shoulder_yaw_joint': -0.07480553537607193,
    'left_elbow_joint': 0.8288170099258423,
    'left_wrist_roll_joint': 0.004889564123004675,
    'left_wrist_pitch_joint': 0.011696604080498219,
    'left_wrist_yaw_joint': -0.005714078899472952,
    'right_shoulder_pitch_joint': 0.2593865692615509,
    'right_shoulder_roll_joint': -0.28865206241607666,
    'right_shoulder_yaw_joint': 0.08828778564929962,
    'right_elbow_joint': 0.7930681109428406,
    'right_wrist_roll_joint': -0.014333133585751057,
    'right_wrist_pitch_joint': 0.014196992851793766,
    'right_wrist_yaw_joint': 0.0037352433428168297,
    'waist_yaw_joint': -0.008544029667973518,
}


def reference():
    return dict(source=SOURCE, captured_at=CAPTURED_AT, target=dict(TARGET), sha256=digest(TARGET))
