"""Offline archive parity and real FCL process checks; never initializes robot I/O."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from rosidl_runtime_py.convert import message_to_ordereddict

from task_saved_test import load_archive, prepare_range
from task_trajectory_library import playback_copy
from task_planning_config import load_parameters, DEFAULT_CONFIG
from workbench_model import OfflineModel


ARCHIVE=Path(__file__).resolve().parents[1]/'data/offline_trajectory_library/all_replayable_diverse_preinsert_parallel6_v1/2ca166b9c0876d6596e7'


@unittest.skipUnless(ARCHIVE.is_dir(),'Recorded archive fixture unavailable')
class AlignmentTests(unittest.TestCase):
    def test_full_archive_and_selected_hardware_range_have_identical_timestamps(self):
        archive=load_archive(ARCHIVE,ARCHIVE/'context/robot.urdf')
        before=copy.deepcopy(archive)
        parameters,config=load_parameters(DEFAULT_CONFIG)
        with tempfile.TemporaryDirectory() as directory:
            path,report=playback_copy(ARCHIVE,Path(directory)/'playback',speed_multiplier=24.,
                velocity=.3,acceleration=.5,motion_parameters=config['values'],timing_policy='guarded')
            model=OfflineModel(path.parent/'robot.urdf')
            scaling={}
            # Isolate timing parity from the separately tested dense collision pass.
            with patch('task_saved_test._validate_range',side_effect=lambda selected,scaled,*args:scaled):
                selected=prepare_range(archive,3,5,.3,.5,model,lambda:None,lambda message:None,
                                       parameters,speed_multiplier=24.,speed_report=scaling)
            self.assertEqual(scaling,report['speed_scaling'])
            for segment,preview in zip(selected,report['segments'][2:5]):
                self.assertEqual(message_to_ordereddict(segment['trajectory']),preview['trajectory'])
            self.assertEqual(archive,before)

    def test_actual_fcl_serial_and_spawn_workers_agree(self):
        from task_model_resources import archived_model_tree
        from task_hook_collision import HandBaseCollision
        from task_parallel_collision import create_pool,validate
        archive=load_archive(ARCHIVE,ARCHIVE/'context/robot.urdf')
        stage=archive['stages'][0]
        trajectory=stage['original_segment']['trajectory']
        names=trajectory['joint_names']
        path=[p['positions'] for p in trajectory['points']]
        with tempfile.TemporaryDirectory() as directory:
            urdf=Path(directory)/'robot.urdf'
            tree,_=archived_model_tree(ARCHIVE);tree.write(urdf,encoding='unicode')
            model=OfflineModel(urdf)
            top=archive['plan']['contact_scene']['base_top_pelvis_m']
            checker=HandBaseCollision(model,str(urdf),top)
            limits={name:model.limits[name] for name in names}
            serial=[];parallel=[]
            args=(checker,path,stage['start'],stage['side'],names,limits,.02,lambda:None)
            validate(None,*args,serial.append)
            with create_pool(urdf,top,workers=2) as pool:
                validate(pool,*args,parallel.append)
            self.assertEqual(serial,parallel)
            self.assertEqual(parallel[-1],len(path))


if __name__=='__main__':unittest.main()
