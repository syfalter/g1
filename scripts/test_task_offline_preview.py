import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from moveit_msgs.msg import DisplayRobotState, DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from task_composition_preview import stamp, at
from task_offline_preview import PreviewTimeline
from task_editor_offline import preview_config, preview_state_parameters


class PreviewTests(unittest.TestCase):
    def trajectory(self):
        points = [stamp(JointTrajectoryPoint(positions=[q], velocities=[0.], accelerations=[0.]), t)
                  for t, q in ((0., 0.), (1., 1.), (2., -.5))]
        return JointTrajectory(joint_names=['arm'], points=points)

    def test_time_sampling_matches_saved_polynomial_and_holds_end(self):
        tr = self.trajectory()
        timeline = PreviewTimeline(['arm', 'other'])
        timeline.trajectory(DisplayTrajectory(trajectory=[RobotTrajectory(joint_trajectory=tr)]), 10.)
        for t in np.linspace(0., 2., 101):
            self.assertAlmostEqual(timeline.sample(10.+t)['arm'], at(tr, t).positions[0])
        self.assertEqual(timeline.sample(100.), {'arm': -.5, 'other': 0.})

    def test_pause_seek_hide_and_resume(self):
        timeline = PreviewTimeline(['arm'])
        tr = self.trajectory()
        trajectory = DisplayTrajectory(trajectory=[RobotTrajectory(joint_trajectory=tr)])
        timeline.trajectory(trajectory, 0.)
        timeline.sample(.5)
        timeline.trajectory(DisplayTrajectory(), .5)
        self.assertAlmostEqual(timeline.sample(100.)['arm'], .5)
        state = DisplayRobotState()
        state.state.joint_state.name = ['arm', 'unknown']
        state.state.joint_state.position = [.2, 8.]
        timeline.state(state)
        self.assertEqual(timeline.sample(100.), {'arm': .2})
        timeline.trajectory(trajectory, 100.)
        timeline.state(DisplayRobotState(hide=True))
        self.assertAlmostEqual(timeline.sample(100.5)['arm'], .5)

    def test_multi_segment_and_invalid_times(self):
        timeline = PreviewTimeline(['arm'])
        tr = self.trajectory()
        timeline.trajectory(DisplayTrajectory(trajectory=[RobotTrajectory(joint_trajectory=tr)]*2), 0.)
        self.assertAlmostEqual(timeline.sample(2.5)['arm'], .5)
        tr.points[1].time_from_start = tr.points[0].time_from_start
        with self.assertRaises(ValueError):
            timeline.trajectory(DisplayTrajectory(trajectory=[RobotTrajectory(joint_trajectory=tr)]), 0.)

    def test_offline_display_is_namespaced_and_has_no_moveit_overlays(self):
        config = preview_config()['Visualization Manager']
        self.assertFalse(any(d['Class'].startswith('moveit_rviz_plugin/') for d in config['Displays']))
        robot = next(d for d in config['Displays'] if d['Class'].endswith('/RobotModel'))
        self.assertEqual(robot['TF Prefix'], 'offline_preview')
        self.assertEqual(robot['Description Topic']['Value'], '/g1/offline_preview/robot_description')
        self.assertEqual(config['Global Options']['Frame Rate'], 30)

    def test_offline_dds_avoids_ip_fragmentation(self):
        script = Path(__file__).with_name('task_editor_offline.sh').read_text()
        line = next(line for line in script.splitlines() if 'export CYCLONEDDS_URI=' in line)
        config = ET.fromstring(line.split("'", 2)[1])
        self.assertEqual(config.findtext('./Domain/General/MaxMessageSize'), '1400B')
        self.assertEqual(config.findtext('./Domain/General/FragmentSize'), '1024B')

    def test_preview_state_publisher_does_not_throttle_paced_samples(self):
        params = preview_state_parameters('<robot name="preview"/>')
        self.assertTrue(params['ignore_timestamp'])
        self.assertEqual(params['frame_prefix'], 'offline_preview/')
        self.assertEqual(params['robot_description'], '<robot name="preview"/>')


if __name__ == '__main__':
    unittest.main()
