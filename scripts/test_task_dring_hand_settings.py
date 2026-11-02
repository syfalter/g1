import copy
from pathlib import Path
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock

from task_dring_hand_settings import DEFAULTS,load,save,validate,archive_variant
from task_hand_control import POSTURES,model_joints,posture_targets
from task_motion_planner import TaskPlanner
from task_plan_archive import stages
from task_saved_test import load_archive

ARCHIVE=Path(__file__).resolve().parents[1]/'data/offline_trajectory_library/all_replayable_diverse_preinsert_parallel6_v1/2ca166b9c0876d6596e7'


class HandSettingTests(unittest.TestCase):
    def test_default_and_persistence_do_not_mutate_global_postures(self):
        before=copy.deepcopy(POSTURES)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'hand.json'
            self.assertEqual(load(path),DEFAULTS)
            save(dict(index_q=.65,middle_q=.9),path)
            self.assertEqual(load(path),dict(index_q=.65,middle_q=.9))
        self.assertEqual(POSTURES,before)

    def test_invalid_targets_rejected(self):
        for value in (-.1,1.1,float('nan'),float('inf'),True):
            with self.assertRaises(ValueError):validate(dict(index_q=value,middle_q=.8))
        with self.assertRaises(ValueError):validate(dict(index_q=.8))

    def test_normalized_direction_and_independent_fingers(self):
        settings=dict(index_q=.2,middle_q=.9)
        self.assertEqual(posture_targets('dring_two_finger',settings),(0.,0.,.9,.2,1.,1.))
        self.assertEqual(posture_targets('open',settings),POSTURES['open'])
        self.assertEqual(posture_targets('index_only',settings),POSTURES['index_only'])
        closed=model_joints('left',[0.]*6);opened=model_joints('left',[1.]*6)
        self.assertGreater(closed['left_index_1_joint'],opened['left_index_1_joint'])

    def test_live_hand_transition_uses_frozen_settings(self):
        settings=dict(index_q=.65,middle_q=.9)
        q=model_joints('left',POSTURES['open'])
        planner=S(articulated_hands=True,dring_hand_settings=settings,validate=Mock())
        segment=TaskPlanner.hand_transition(planner,'left',q,'dring_two_finger',None)
        self.assertEqual(segment['target_q'],[0.,0.,.9,.65,1.,1.])
        self.assertEqual(segment['end'],model_joints('left',segment['target_q']))
        settings['index_q']=.1
        self.assertEqual(segment['target_q'][3],.65)

    @unittest.skipUnless(ARCHIVE.is_dir(),'Recorded archive unavailable')
    def test_archive_variant_updates_hand_transitions_and_keeps_arm_path_immutable(self):
        archive=load_archive(ARCHIVE,ARCHIVE/'context/robot.urdf')
        before=copy.deepcopy(archive)
        settings=dict(index_q=.65,middle_q=.9)
        result=archive_variant(archive,settings)
        self.assertEqual(archive,before)
        self.assertTrue(result['hand_settings_changed'])
        self.assertEqual(result['plan']['segments'][1]['target_q'],[0.,0.,.9,.65,1.,1.])
        self.assertEqual(result['stages'],stages(result['plan']))
        target=model_joints('left',[0.,0.,.9,.65,1.,1.])
        for stage in result['stages'][2:6]:
            self.assertEqual(stage['start']['left_index_1_joint'],target['left_index_1_joint'])
            self.assertEqual(stage['start']['left_middle_1_joint'],target['left_middle_1_joint'])
        for old,new in zip(archive['plan']['segments'],result['plan']['segments']):
            if old['kind']=='arm':self.assertEqual(old,new)
        self.assertEqual(result['plan']['segments'][5]['target_q'],[1.]*6)
        self.assertEqual(result['stages'][-1]['start'],archive['stages'][-1]['start'])

    @unittest.skipUnless(ARCHIVE.is_dir(),'Recorded archive unavailable')
    def test_default_settings_preserve_original_trajectories_and_cache_fingerprints(self):
        archive=load_archive(ARCHIVE,ARCHIVE/'context/robot.urdf')
        result=archive_variant(archive,DEFAULTS)
        self.assertFalse(result['hand_settings_changed'])
        self.assertEqual(result['stages'],archive['stages'])
        self.assertEqual(result['plan']['segments'],archive['plan']['segments'])


if __name__=='__main__':unittest.main()
