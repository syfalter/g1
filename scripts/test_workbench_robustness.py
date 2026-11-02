"""Test the added test-only base translation without ROS or planning services."""
from pathlib import Path
import tempfile
import unittest

import numpy as np
from workbench_model import OfflineModel


class StanceModelTests(unittest.TestCase):
    def test_robot_moves_but_environment_does_not(self):
        xml='''<robot name="test"><link name="world"/><link name="x"/><link name="y"/>
        <link name="pelvis"/><link name="table"/>
        <joint name="dx" type="prismatic"><parent link="world"/><child link="x"/><axis xyz="1 0 0"/></joint>
        <joint name="dy" type="prismatic"><parent link="x"/><child link="y"/><axis xyz="0 1 0"/></joint>
        <joint name="base" type="fixed"><parent link="y"/><child link="pelvis"/><origin xyz="-.06 0 .793"/></joint>
        <joint name="table_mount" type="fixed"><parent link="world"/><child link="table"/><origin xyz=".111 0 0"/></joint></robot>'''
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'model.urdf';path.write_text(xml);model=OfflineModel(path)
            for dx,dy in ((0,0),(.1,-.12),(-.06,.12)):
                q=dict(dx=dx,dy=dy)
                np.testing.assert_allclose(model.transform('pelvis',q)[:3,3],[-.06+dx,dy,.793])
                np.testing.assert_allclose(model.transform('table',q)[:3,3],[.111,0,0])

    def test_prismatic_axis_obeys_joint_frame(self):
        xml='''<robot name="test"><link name="world"/><link name="slider"/>
        <joint name="dx" type="prismatic"><parent link="world"/><child link="slider"/>
        <origin xyz="0 0 1" rpy="0 0 1.5707963267948966"/><axis xyz="1 0 0"/></joint></robot>'''
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'model.urdf';path.write_text(xml);model=OfflineModel(path)
            np.testing.assert_allclose(model.transform('slider',dict(dx=.2))[:3,3],[0,.2,1],atol=1e-12)


if __name__=='__main__':unittest.main()
