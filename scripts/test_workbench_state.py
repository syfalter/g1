"""Offline console state guards, without constructing a ROS node."""
import copy
import json
import threading
import unittest

from workbench_app import WorkbenchApp, ASSETS, startup_joints, INDEX_OPEN, INDEX_FOLDED


class Harness:
    command=WorkbenchApp.command
    _command=WorkbenchApp._command
    sample_playback=WorkbenchApp.sample_playback
    begin_plan=WorkbenchApp.begin_plan

    def __init__(self):
        self.lock=threading.RLock()
        self.busy=False;self.playing=False
        self.names=['waist_yaw_joint','task_handle_angle']
        self.home=dict.fromkeys(self.names,0.)
        self.current=dict(self.home)
        self.plan=[dict(t=0.,q=dict(self.home),phase='START'),
                   dict(t=1.,q=dict(waist_yaw_joint=.2,task_handle_angle=.5),phase='INSERT')]
        self.plan_arm='left';self.play_time=0.
        self.target=json.loads((ASSETS/'task/target.json').read_text())
        self.plan_target=json.dumps(self.target,sort_keys=True)
        self.status=dict(arm='left',phase='PLAN_READY',progress=0.)
        self.cancel_event=threading.Event()

    def message(self,phase,text):
        self.status.update(phase=phase,message=text)


class StateTests(unittest.TestCase):
    def setUp(self):self.app=Harness()

    def test_arm_switch_invalidates_plan(self):
        self.app.command(dict(action='select_arm',arm='right'))
        self.assertFalse(self.app.plan)
        with self.assertRaises(ValueError):self.app.command(dict(action='play'))

    def test_no_new_task_mid_contact(self):
        self.app.command(dict(action='seek',fraction=.5))
        self.assertAlmostEqual(self.app.current['task_handle_angle'],.25)
        with self.assertRaises(ValueError):self.app.command(dict(action='plan',arm='left',scope='full'))
        self.assertFalse(self.app.busy)

    def test_reset_is_initial_not_last_start(self):
        self.app.command(dict(action='seek',fraction=1.))
        self.app.command(dict(action='reset'))
        self.assertEqual(self.app.current,self.app.home)
        self.assertFalse(self.app.plan)
        self.assertEqual(self.app.status['progress'],0.)

    def test_target_registration_guard(self):
        target=copy.deepcopy(self.app.target)
        target['axis_center'][0]+=.02;target['entry_center'][0]+=.02
        with self.assertRaises(ValueError):self.app.command(dict(action='target',target=target))
        self.assertNotEqual(self.app.target,target)

    def test_invalid_seek_and_cancel(self):
        for fraction in (-.1,1.1,float('nan')):
            with self.assertRaises(ValueError):self.app.command(dict(action='seek',fraction=fraction))
        self.app.command(dict(action='cancel'))
        self.assertFalse(self.app.playing)
        self.assertFalse(self.app.plan)

    def test_no_target_replace_during_play(self):
        self.app.command(dict(action='play'))
        with self.assertRaises(ValueError):self.app.command(dict(action='target',target=self.app.target))

    def test_default_startup_hands_open_preserves_measured_arms(self):
        profile=dict(joints={'left_elbow_joint':.82,'right_elbow_joint':.80})
        q=startup_joints(list(profile['joints']),profile)
        for name,value in profile['joints'].items():self.assertEqual(q[name],value)
        for side in ('left','right'):
            self.assertEqual(q[side+'_index_1_joint'],INDEX_OPEN)
            self.assertAlmostEqual(q[side+'_index_2_joint'],0.)

    def test_explicit_legacy_folded_posture(self):
        q=startup_joints([],dict(joints={},hand_startup_posture='folded'))
        self.assertEqual(q['left_index_1_joint'],INDEX_FOLDED)
        self.assertEqual(q['right_index_1_joint'],INDEX_FOLDED)
        with self.assertRaises(ValueError):startup_joints([],dict(joints={},hand_startup_posture='unknown'))


if __name__=='__main__':unittest.main()
