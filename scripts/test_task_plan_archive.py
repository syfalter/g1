import copy
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace as S
from unittest.mock import Mock,patch

from task_plan_archive import stages, freeze, verify, write_json
from task_session import ARMS, ARM_NAMES
from task_hand_control import model_joints


class ArchiveTests(unittest.TestCase):
    def setUp(self):
        self.names = list(ARMS['left'])
        self.start = dict.fromkeys(ARM_NAMES, 0.)
        self.start.update(model_joints('left', [1.] * 6))
        def point(q, t):
            return dict(positions=q, velocities=[0.] * 7, accelerations=[0.] * 7,
                        effort=[], time_from_start=dict(sec=t, nanosec=0))
        segment = dict(side='left', kind='arm', label='Approach',
                       trajectory=dict(joint_names=self.names,
                                       points=[point([0.] * 7, 0), point([.1] * 7, 2)]))
        self.plan = dict(kind='task', arm='left', start=self.start, segments=[segment])

    def test_preserves_trajectory_and_other_arm(self):
        original = copy.deepcopy(self.plan)
        record = stages(self.plan)[0]
        self.assertEqual(self.plan, original)
        self.assertEqual(record['original_segment'], original['segments'][0])
        self.assertFalse(record['hardware_executable'])
        self.assertEqual(record['end'][ARMS['right'][0]], 0.)

    def test_discontinuity_rejected(self):
        self.plan['segments'] *= 2
        with self.assertRaisesRegex(ValueError, 'Discontinuous'):
            stages(self.plan)

    def test_nonfinite_rejected(self):
        self.plan['segments'][0]['trajectory']['points'][1]['positions'][0] = float('nan')
        with self.assertRaises(ValueError):
            stages(self.plan)

    def test_missing_derivatives_rejected(self):
        self.plan['segments'][0]['trajectory']['points'][1]['velocities'] = []
        with self.assertRaises(ValueError):
            stages(self.plan)

    def test_nonrest_endpoint_rejected(self):
        self.plan['segments'][0]['trajectory']['points'][1]['velocities'][0] = .1
        with self.assertRaisesRegex(ValueError, 'not at rest'):
            stages(self.plan)

    def test_invalid_timestamp_rejected(self):
        self.plan['segments'][0]['trajectory']['points'][1]['time_from_start']['nanosec'] = -1
        with self.assertRaises(ValueError):
            stages(self.plan)

    def test_hand_target_mismatch_rejected(self):
        start = model_joints('left', [1.] * 6)
        self.plan['segments'] = [dict(side='left', kind='hand', label='Hand index_only',
            target_q=[0., 0., 0., 1., 0., 0.], trajectory=dict(joint_names=list(start),
            points=[dict(positions=list(start.values()), time_from_start=dict(sec=t, nanosec=0))
                    for t in (0, 2)]))]
        with self.assertRaisesRegex(ValueError, 'DDS target'):
            stages(self.plan)

    def test_freeze_verify_tamper_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            source, destination = root / 'source', root / 'archive'
            source.mkdir()
            for name in ('capture.json', 'capture_state.json', 'geometry.json', 'rgb.png',
                         'depth_aligned.png', 'intrinsics.json'):
                (source / name).touch()
            write_json(source / 'plan.json', self.plan)
            (root / 'robot.urdf').touch()
            freeze(source, destination, root / 'robot.urdf')
            self.assertEqual(verify(destination)['stages'][0]['label'], 'Approach')
            self.assertEqual(json.loads((destination / 'plan/plan.json').read_text()), self.plan)
            with self.assertRaises(FileExistsError):
                freeze(source, destination, root / 'robot.urdf')
            target = destination / 'plan/plan.json'
            target.chmod(0o644)
            target.write_text('{}')
            with self.assertRaisesRegex(ValueError, 'checksum'):
                verify(destination)

    def test_preflight_checks_other_arm_and_hand(self):
        from check_saved_task_stage import compare_start
        self.plan['start'].update(model_joints('right', [1.] * 6))
        stage = stages(self.plan)[0]
        hands = {side: {'feedback_q': [1.] * 6} for side in ('left', 'right')}
        actual = dict(self.start)
        self.assertTrue(compare_start(stage, actual, hands)['arm_waist_start_matches'])
        actual[ARMS['right'][0]] += .03
        self.assertFalse(compare_start(stage, actual, hands)['arm_waist_start_matches'])
        hands['right']['feedback_q'][0] = .5
        self.assertFalse(compare_start(stage, actual, hands)['hand_start_matches'])

    def test_offline_export_reloads_without_execution_or_scene_authorization(self):
        from task_offline_export import export_archive
        from task_saved_test import load_archive
        from task_dring_sequence import labels
        state=dict(self.start)
        segments=[]
        for label in labels('left'):
            hand=label.startswith('Hand ')
            target=([0.,0.,.8,.8,1.,1.] if label=='Hand dring_two_finger' else [1.]*6)
            goal=model_joints('left',target) if hand else {n:state[n] for n in self.names}
            names=list(goal)
            points=[dict(positions=[s[n] for n in names],velocities=[0.]*len(names),
                         accelerations=[0.]*len(names),effort=[],time_from_start=dict(sec=t,nanosec=0))
                    for s,t in ((state,0),(goal,2))]
            segments.append(dict(side='left',kind='hand' if hand else 'arm',label=label,
                                 target_q=target if hand else None,
                                 trajectory=dict(joint_names=names,points=points)))
            state.update(goal)
        report=dict(success=True,mode='PRODUCTION_SCOPED_OFFLINE',hardware_executable=False,
                    hardware_commands_sent=0,segments=segments,arm='left',sweep_direction='left',
                    ready_source='ready_poses.json',startup_heading_source='session_home.json',
                    planning_config={'effective_values':{}},velocity_scaling=.15,acceleration_scaling=.15,
                    waist_assist=False,search_diagnostics={},scene=dict(profile='steep_straight',
                    base_radius_m=.025,base_height_m=.02,arm_body_collision_checked=True))
        with tempfile.TemporaryDirectory() as root:
            root=Path(root);source=root/'capture';source.mkdir()
            for name in ('capture.json','capture_state.json','geometry.json','rgb.png',
                         'depth_aligned.png','intrinsics.json'):
                (source/name).write_text('{}')
            mesh=root/'fixture mesh.stl';mesh.write_text('opaque fixture mesh')
            model=root/'robot.urdf'
            joints=''.join(f'<joint name="{name}" type="revolute"><parent link="base"/>'
                           f'<child link="child_{index}"/><axis xyz="0 0 1"/>'
                           '<limit lower="-3" upper="3" velocity="2" effort="10"/></joint>'
                           for index,name in enumerate(self.names))
            model.write_text(f'<robot name="fixture"><link name="base"><visual><geometry>'
                             f'<mesh filename="{mesh.as_uri()}"/></geometry></visual></link>{joints}</robot>')
            archive=root/'archive with spaces'
            result=export_archive(source,archive,model,report,self.start)
            self.assertTrue(result['reload_verified'])
            provenance=json.loads((archive/'plan/offline_provenance.json').read_text())
            self.assertIsNone(provenance['runtime_sha256'])
            self.assertEqual(provenance['runtime_snapshot_scope'],'not recorded in source replay')
            from unittest.mock import patch
            with patch('task_offline_export.runtime_matches',return_value=False):
                with self.assertRaisesRegex(ValueError,'Native runtime changed'):
                    export_archive(source,root/'changed-runtime',model,
                                   dict(report,runtime_sha256={'fixture':'old'}),self.start)
            self.assertFalse((root/'changed-runtime').exists())
            loaded=load_archive(archive,model)
            from task_live_archive import publish_live_plan
            write_json(source/'plan.json',loaded['plan'])
            published=Path(publish_live_plan(source,model,root/'live_library'))
            self.assertEqual(len(load_archive(published,model)['stages']),7)
            self.assertFalse(load_archive(published,model)['manifest']['operator_scene_unchanged_commitment'])
            from task_trajectory_library import catalog
            live_entries,live_errors=catalog(root/'live_library')
            self.assertEqual(len(live_entries),1);self.assertEqual(live_errors,[])
            self.assertEqual(live_entries[0]['directory'],str(published))
            self.assertEqual(len(loaded['stages']),7)
            self.assertFalse(loaded['manifest']['operator_scene_unchanged_commitment'])
            self.assertFalse(loaded['plan']['hardware_executable'])
            self.assertEqual(loaded['plan']['segments'],segments)
            from task_trajectory_library import catalog,playback_copy,require_compatible_models
            entries,errors=catalog(archive)
            self.assertEqual(len(entries),1);self.assertEqual(errors,[])
            source_hash=loaded['manifest']['sha256']['context/robot.urdf']
            path,playback=playback_copy(archive,root/'playback')
            self.assertEqual(playback['source_model_sha256'],source_hash)
            self.assertFalse(playback['hardware_executable'])
            self.assertEqual(playback['timing_policy'],'guarded')
            self.assertTrue(playback['speed_scaling']['limits_enforced'])
            for old,new in zip(segments,playback['segments']):
                self.assertEqual([p['positions'] for p in old['trajectory']['points']],
                                 [p['positions'] for p in new['trajectory']['points']])
            import xml.etree.ElementTree as ET
            relocated=next(ET.parse(path.parent/'robot.urdf').iter('mesh')).attrib['filename']
            self.assertTrue(relocated.startswith((archive/'plan/model_resources').as_uri()+'/'))
            require_compatible_models([(path,playback),(path,copy.deepcopy(playback))])
            different=copy.deepcopy(playback);different['source_mesh_sha256']={mesh.as_uri():'different'}
            with self.assertRaisesRegex(ValueError,'mesh mismatch'):
                require_compatible_models([(path,playback),(path,different)])
            from task_preview_playback import PreviewTimeline
            timeline=PreviewTimeline(playback,self.start)
            self.assertEqual(len(timeline.parts),7)
            self.assertEqual(timeline.sample(timeline.duration)[0],loaded['stages'][-1]['end'])
            from publish_task_trajectory_library import publish
            from task_plan_archive import digest
            benchmark=dict(selected_contexts=1,manifest_sha256='fixture',results=[dict(
                case_id='a'*20,pair_id='fixture',direction='left',complete_archive_pass=True,
                status='archive_reload_pass',code_integrity_verified=True,hardware_commands_sent=0,
                case_wall_seconds=1.,archive=result)])
            write_json(root/'summary.json',benchmark)
            publication=publish(root/'summary.json',root/'library',2)
            copied=Path(publication['entries'][0]['directory'])
            self.assertEqual(digest(copied/'archive_manifest.json'),digest(archive/'archive_manifest.json'))
            self.assertFalse(load_archive(copied,model)['plan']['hardware_executable'])
            with self.assertRaises(FileExistsError):publish(root/'summary.json',root/'library',2)
            benchmark['results'][0]['case_wall_seconds']=181.
            write_json(root/'summary.json',benchmark)
            with self.assertRaisesRegex(ValueError,'180-second'):publish(root/'summary.json',root/'rejected_library',2)
            report['ready_source']='capture state fallback'
            with self.assertRaisesRegex(ValueError,'Historical ready/home'):
                export_archive(source,root/'rejected',model,report,self.start)
            mesh.unlink()
            with self.assertRaisesRegex(ValueError,'Live mesh unavailable'):
                load_archive(archive,model)
            portable=load_archive(archive,archive/'context/robot.urdf')
            portable['plan']['contact_scene']['base_top_pelvis_m']=[0.,0.,0.]
            from task_saved_test import prepare_range
            from task_model_resources import resolve_mesh
            def collision_model(model,urdf,top):
                reference=next(ET.parse(urdf).iter('mesh')).attrib['filename']
                self.assertTrue(resolve_mesh(reference,urdf).is_relative_to(archive/'plan/model_resources'))
                return Mock(check=Mock(return_value=None),check_arm_body=Mock(return_value=None))
            robot=Mock();robot.find.return_value=ET.fromstring('<limit lower="-2" upper="2"/>')
            with patch('task_hook_collision.HandBaseCollision',side_effect=collision_model):
                selected=prepare_range(portable,2,2,.15,.15,S(robot=robot),lambda:None,lambda m:None)
            self.assertEqual(len(selected),1)
            self.assertEqual(selected[0]['label'],'Hand dring_two_finger')
            verify(archive)


if __name__ == '__main__':
    unittest.main()
