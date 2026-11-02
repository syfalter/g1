"""Open startup posture has explicit finger phases outside arm-only planning."""
from types import SimpleNamespace
import unittest

from moveit_msgs.msg import RobotState
from workbench_app import WorkbenchApp, startup_joints, finger_pose, INDEX_FOLDED, INDEX_OPEN


class MotionHarness:
    append = WorkbenchApp.append
    change_finger = WorkbenchApp.change_finger
    joint_path = WorkbenchApp.joint_path
    enter_table = WorkbenchApp.enter_table
    return_home = WorkbenchApp.return_home
    arm_joint = staticmethod(WorkbenchApp.arm_joint)

    def __init__(self, posture='open'):
        self.home = startup_joints(['left_elbow_joint', 'right_elbow_joint', 'waist_yaw_joint'],
                                   dict(joints={}, hand_startup_posture=posture))
        self.names = list(self.home)
        self.motion = object()
        self.arm_requests = []

    def validate(self, q):
        pass

    def state(self, q):
        result = RobotState()
        result.joint_state.name = self.names
        result.joint_state.position = [q[n] for n in self.names]
        return result

    def call(self, client, request, timeout):
        request = request.motion_plan_request
        self.arm_requests.append(request)
        constraints = request.goal_constraints[0].joint_constraints
        path = SimpleNamespace(joint_names=[c.joint_name for c in constraints],
                               points=[SimpleNamespace(positions=[c.position for c in constraints])])
        return SimpleNamespace(motion_plan_response=SimpleNamespace(
            error_code=SimpleNamespace(val=1), trajectory=SimpleNamespace(joint_trajectory=path)))


class FingerTransitionTests(unittest.TestCase):
    def test_both_sides_fold_before_entry_and_restore_after_return(self):
        for side in ('left', 'right'):
            with self.subTest(side=side):
                app = MotionHarness()
                frames = []
                app.append(frames, app.home, 'START')
                goal = finger_pose(app.home, side, INDEX_FOLDED)
                goal[side + '_elbow_joint'] = .8
                app.enter_table(side, app.home, goal, frames)
                app.return_home(side, goal, app.home, frames)
                phases = [f['phase'] for i, f in enumerate(frames)
                          if i == 0 or frames[i - 1]['phase'] != f['phase']]
                self.assertEqual(phases, ['START', 'FOLD_START_INDEX', 'ENTER_TABLE',
                                          'RETURN_HOME', 'RESTORE_START_INDEX'])
                self.assertEqual(frames[-1]['q'], app.home)
                for request in app.arm_requests:
                    q = dict(zip(request.start_state.joint_state.name,
                                 request.start_state.joint_state.position))
                    self.assertEqual(q[side + '_index_1_joint'], INDEX_FOLDED)
                for frame in frames:
                    if frame['phase'] in ('FOLD_START_INDEX', 'RESTORE_START_INDEX'):
                        self.assertEqual(frame['q'][side + '_elbow_joint'], 0.)
                    other = 'right' if side == 'left' else 'left'
                    self.assertEqual(frame['q'][other + '_index_1_joint'], INDEX_OPEN)

    def test_arm_only_path_rejects_hidden_finger_change(self):
        app = MotionHarness()
        with self.assertRaisesRegex(RuntimeError, 'explicit finger phase'):
            app.joint_path('left', app.home, finger_pose(app.home, 'left', INDEX_FOLDED), 'ENTER_TABLE', [])
        self.assertFalse(app.arm_requests)

    def test_legacy_folded_start_needs_no_extra_phase(self):
        app = MotionHarness('folded')
        frames = []
        app.append(frames, app.home, 'START')
        app.enter_table('left', app.home, app.home, frames)
        app.return_home('left', app.home, app.home, frames)
        self.assertEqual([f['phase'] for f in frames],
                         ['START', 'ENTER_TABLE', 'ENTER_TABLE', 'RETURN_HOME', 'RETURN_HOME'])


if __name__ == '__main__':
    unittest.main()
