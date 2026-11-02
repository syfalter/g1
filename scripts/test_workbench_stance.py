"""XY placement guards, geometry and Qt controls; no robot communication."""
import copy
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from PyQt5 import QtCore, QtWidgets, QtTest
from test_workbench_state import Harness
from workbench_app import Console
from workbench_model import OfflineModel
from workbench_stance import STANCE_JOINTS, add_stance_joints


def harness():
    node=Harness()
    node.status['message']='Ready'
    node.names+=list(STANCE_JOINTS)
    node.home.update(dict.fromkeys(STANCE_JOINTS,0.))
    node.current=dict(node.home)
    for frame in node.plan:frame['q'].update(dict.fromkeys(STANCE_JOINTS,0.))
    return node


class StanceTests(unittest.TestCase):
    def test_stance_invalidates_plan_preserves_pose_and_target(self):
        node=harness();target=copy.deepcopy(node.target)
        node.command(dict(action='stance',x=.02,y=-.04))
        self.assertEqual(node.home['workbench_stance_x'],.02)
        self.assertEqual(node.home['workbench_stance_y'],-.04)
        self.assertEqual(node.home['waist_yaw_joint'],0.)
        self.assertEqual(node.current,node.home)
        self.assertEqual(node.target,target)
        self.assertFalse(node.plan)
        with self.assertRaises(ValueError):node.command(dict(action='play'))
        node.command(dict(action='reset'))
        self.assertEqual(node.current['workbench_stance_x'],.02)

    def test_rejected_changes_are_atomic(self):
        for value in (.301,-.301,float('nan'),float('inf')):
            node=harness();home=dict(node.home)
            with self.assertRaises(ValueError):node.command(dict(action='stance',x=.1,y=value))
            self.assertEqual(node.home,home)
            self.assertTrue(node.plan)
        for condition in ('busy','playing','mid_task'):
            node=harness()
            if condition=='mid_task':node.current['task_handle_angle']=.2
            else:setattr(node,condition,True)
            with self.assertRaises(ValueError):node.command(dict(action='stance',x=.1,y=0.))
            self.assertEqual(node.home['workbench_stance_x'],0.)

    def test_same_stance_keeps_plan(self):
        node=harness()
        node.command(dict(action='stance',x=0.,y=0.))
        self.assertTrue(node.plan)

    def test_only_robot_moves(self):
        robot=ET.fromstring('''<robot name="test"><link name="world"/><link name="pelvis"/>
          <link name="table"/><link name="hand"/>
          <joint name="preview_world_pelvis" type="fixed"><parent link="world"/><child link="pelvis"/>
          <origin xyz="-.06 0 .793"/></joint>
          <joint name="wrist" type="fixed"><parent link="pelvis"/><child link="hand"/><origin xyz=".1 .2 .3"/></joint>
          <joint name="table_mount" type="fixed"><parent link="world"/><child link="table"/><origin xyz=".111 0 0"/></joint></robot>''')
        add_stance_joints(robot)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'robot.urdf';path.write_text(ET.tostring(robot,encoding='unicode'))
            model=OfflineModel(path)
            for x,y in ((0,0),(.02,-.04),(-.3,.3)):
                q=dict(zip(STANCE_JOINTS,(x,y)))
                for link in ('pelvis','hand'):
                    expected=model.transform(link,{})[:3,3]+[x,y,0]
                    np.testing.assert_allclose(model.transform(link,q)[:3,3],expected)
                np.testing.assert_allclose(model.transform('table',q)[:3,3],[.111,0,0])


class StanceUiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.qt=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.node=harness();self.window=Console(self.node)
        self.window.show();self.window.refresh();self.qt.processEvents()

    def tearDown(self):
        self.window.timer.stop();self.window.close();self.window.deleteLater();self.qt.processEvents()

    def test_sliders_and_numeric_fields_update_both_axes(self):
        self.window.stance_sliders['x'].setValue(20)
        self.window.stance_spins['y'].setValue(-4.)
        self.assertAlmostEqual(self.node.home['workbench_stance_x'],.02)
        self.assertAlmostEqual(self.node.home['workbench_stance_y'],-.04)
        self.assertEqual(self.window.stance_sliders['y'].value(),-40)
        QtTest.QTest.keyClick(self.window.stance_sliders['x'],QtCore.Qt.Key_Right)
        self.assertAlmostEqual(self.node.home['workbench_stance_x'],.021)
        self.assertFalse(self.node.plan)

    def test_disabled_when_busy_or_away_from_home(self):
        self.node.busy=True;self.window.refresh()
        self.assertFalse(self.window.stance_sliders['x'].isEnabled())
        self.node.busy=False;self.node.current['task_handle_angle']=.2;self.window.refresh()
        self.assertFalse(self.window.stance_spins['y'].isEnabled())
        self.node.command(dict(action='reset'));self.window.refresh()
        self.assertTrue(self.window.stance_sliders['x'].isEnabled())


if __name__=='__main__':unittest.main()
