import math
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from preview_payload import apply_preview_payload


class PreviewPayloadTest(unittest.TestCase):
    def model(self):
        return ET.fromstring('''<robot>
          <joint name="right_hand_palm_joint"><origin xyz="0.0415 -0.003 0" rpy="0 0 0"/></joint>
          <joint name="left_hand_palm_joint"><origin xyz="0.0415 0.003 0" rpy="0 0 0"/></joint>
          <link name="right_payload_collision"><collision><geometry><box size="0.3 0.14 0.12"/></geometry></collision></link>
        </robot>''')

    def test_cylinder_and_hand_share_connection_endpoint(self):
        robot = self.model()
        apply_preview_payload(robot, {"connector_length_m": 0.05, "connector_radius_m": 0.03})
        collision = robot.find("link/collision")
        cylinder = collision.find("geometry/cylinder")
        self.assertEqual(float(cylinder.get("length")), 0.05)
        self.assertEqual(float(cylinder.get("radius")), 0.03)
        centre = float(collision.find("origin").get("xyz").split()[0])
        hand = float(robot.find("joint/origin").get("xyz").split()[0])
        self.assertAlmostEqual(centre - 0.025, 0.0415)
        self.assertAlmostEqual(centre + 0.025, hand)
        self.assertAlmostEqual(hand, 0.0915)
        self.assertAlmostEqual(float(collision.find("origin").get("rpy").split()[1]), math.pi / 2)
        self.assertIsNotNone(robot.find("link/visual/geometry/cylinder"))
        self.assertIsNone(robot.find("link/collision/geometry/box"))

    def test_left_and_right_connectors_are_symmetric(self):
        robot = self.model()
        apply_preview_payload(robot, {"connector_length_m": 0.05, "connector_radius_m": 0.03})
        for side, sign in (("left", 1), ("right", -1)):
            collision = robot.find(f"link[@name='{side}_payload_collision']/collision")
            self.assertEqual(collision.find("geometry/cylinder").attrib, {"length": "0.05", "radius": "0.03"})
            xyz = list(map(float, collision.find("origin").get("xyz").split()))
            self.assertAlmostEqual(xyz[0], 0.0665)
            self.assertAlmostEqual(xyz[1], sign * 0.003)
            mount = robot.find(f"joint[@name='{side}_hand_palm_joint']/origin")
            self.assertAlmostEqual(float(mount.get("xyz").split()[0]), 0.0915)
        joint = robot.find("joint[@name='left_payload_collision_joint']")
        self.assertEqual(joint.find("parent").get("link"), "left_wrist_yaw_link")

    def test_invalid_dimensions(self):
        for value in (-0.05, 0, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                apply_preview_payload(self.model(), {"connector_length_m": value, "connector_radius_m": 0.03})

    def test_current_shared_dimensions(self):
        config = json.loads((Path(__file__).resolve().parents[1] /
                             'assets/workbench/payload_preview.json').read_text())
        self.assertEqual(config['connector_length_m'], .04)
        self.assertEqual(config['connector_radius_m'], .03)
        self.assertEqual(config['hand_model'], 'RH56DFX')
        robot = self.model()
        apply_preview_payload(robot, {k: config[k] for k in
                                     ('connector_length_m', 'connector_radius_m')})
        for side in ('left', 'right'):
            cylinder = robot.find(f"link[@name='{side}_payload_collision']/collision/geometry/cylinder")
            self.assertEqual(float(cylinder.get('length')), .04)
            self.assertEqual(float(cylinder.get('radius')), .03)
            mount = robot.find(f"joint[@name='{side}_hand_palm_joint']/origin")
            self.assertAlmostEqual(float(mount.get('xyz').split()[0]), .0815)

    def test_rh56dfx_replaces_visual_and_old_collision_boxes(self):
        robot = self.model()
        for side in ("left", "right"):
            hand = ET.SubElement(robot, "link", name=f"{side}_rubber_hand")
            ET.SubElement(hand, "visual")
            ET.SubElement(hand, "inertial")
            box = ET.SubElement(robot, "link", name=f"{side}_hand_collision")
            ET.SubElement(box, "collision")
        apply_preview_payload(robot, {"connector_length_m": 0.05, "connector_radius_m": 0.03,
                                      "hand_model": "RH56DFX"})
        for side in ("left", "right"):
            hand = robot.find(f"link[@name='{side}_rubber_hand']")
            self.assertEqual(len(hand.findall("visual")), 1)
            self.assertEqual(len(hand.findall("collision")), 1)
            self.assertIsNone(hand.find("inertial"))
            self.assertTrue(hand.find("visual/geometry/mesh").get("filename").endswith(f"rh56dfx/{side}_visual.stl"))
            self.assertIsNone(robot.find(f"link[@name='{side}_hand_collision']/collision"))


if __name__ == "__main__":
    unittest.main()
