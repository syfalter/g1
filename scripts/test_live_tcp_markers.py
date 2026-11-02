import unittest
import numpy as np
from visualization_msgs.msg import Marker
from task_preview_markers import live_tcp_markers, target_markers, with_live_tcp


class LiveTCPTests(unittest.TestCase):
    def test_origins_are_exact_link_origins_and_follow_tf(self):
        markers = live_tcp_markers().markers
        self.assertEqual(len(markers), 12)
        self.assertEqual(len({(m.ns, m.id) for m in markers}), 12)
        for m in markers:
            self.assertIn(m.header.frame_id, ('left_index_tcp', 'right_index_tcp'))
            self.assertTrue(m.frame_locked)
            self.assertEqual(m.action, Marker.ADD)
            self.assertEqual(m.lifetime.sec, 0)
            if m.type == Marker.SPHERE:
                self.assertEqual([m.pose.position.x, m.pose.position.y, m.pose.position.z], [0, 0, 0])
                self.assertAlmostEqual(m.scale.x, .006)

    def test_axes_and_labels(self):
        markers = live_tcp_markers().markers
        for arrow in (m for m in markers if m.type == Marker.ARROW):
            expected = np.zeros(3); expected[arrow.id - 1] = .03
            tip = arrow.points[1]
            np.testing.assert_allclose([tip.x, tip.y, tip.z], expected)
        self.assertEqual([m.text for m in markers if m.type == Marker.TEXT_VIEW_FACING],
                         ['L index_tcp (model)', 'R index_tcp (model)'])

    def test_planning_posture_is_explicit_and_independent_of_finger_tf(self):
        from scipy.spatial.transform import Rotation
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler('z', 40, degrees=True).as_matrix()
        transform[:3, 3] = [.269, -.025, .031]
        for m in live_tcp_markers(dict(left=transform, right=transform)).markers:
            self.assertTrue(m.header.frame_id.endswith('_wrist_yaw_link'))
            if m.type == Marker.SPHERE:
                p = m.pose.position
                np.testing.assert_allclose([p.x, p.y, p.z], transform[:3, 3])
            if m.type == Marker.TEXT_VIEW_FACING:
                self.assertIn('index_only', m.text)

    def test_refresh_clear_precedes_live_markers(self):
        result = with_live_tcp(target_markers({}, np.eye(4), 'both', 'prepare'))
        self.assertEqual(result.markers[0].action, Marker.DELETEALL)
        self.assertEqual(len(result.markers), 13)
        self.assertTrue(all(m.action == Marker.ADD for m in result.markers[1:]))

    def test_center_target_is_not_replaced(self):
        geometry = dict(task_preview={}, center_check=dict(center_pelvis_m=[.3, .1, 0.], target_pelvis_m=[.3, .1, .05]))
        original = target_markers(geometry, np.eye(4), 'left', 'task')
        count = len(original.markers)
        result = with_live_tcp(original)
        self.assertEqual(len(result.markers), count + 12)
        self.assertEqual(result.markers[2].pose.position.z, .05)


if __name__ == '__main__':
    unittest.main()
