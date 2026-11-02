import os
import argparse
from copy import deepcopy
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch
import numpy as np
from task_kinematic_preview import KinematicPreviewPlanner
from task_motion_planner import PlanningFailure
from task_session import ARMS,ARM_NAMES
from task_preview_playback import PreviewTimeline
import task_offline_replay as replay


class OfflineReplayInputTests(unittest.TestCase):
    def fixture(self):
        q=dict.fromkeys(ARM_NAMES,0.)
        q['waist_yaw_joint']=0.
        for side in ('left','right'):q.update(replay.model_joints(side,replay.POSTURES['open']))
        state=dict(joints=q,pelvis_from_torso_row_major=np.eye(4).tolist(),
                   captured_at='2026-09-17T19:19:02+08:00',source='archived feedback')
        geometry=dict(frame_id='torso_link',capture_frame=dict(frame='pelvis',
                      pelvis_from_torso_row_major=np.eye(4).tolist()))
        return state,geometry

    def load(self,state,geometry,articulated=True,waist=True):
        with patch.object(replay,'read_archive',return_value=state):
            return replay.load_start_state(Path('capture_state.json'),geometry,articulated,waist)

    def test_source_options_mutually_exclusive_and_domain_bounded(self):
        parser=argparse.ArgumentParser(allow_abbrev=False)
        replay.add_replay_source_arguments(parser)
        self.assertEqual(parser.parse_args([]).ros_domain_id,93)
        for domain in range(93,100):
            self.assertEqual(parser.parse_args(['--ros-domain-id',str(domain)]).ros_domain_id,domain)
        for args in (['--start-plan','a','--start-state','b'],['--ros-domain-id','0'],
                     ['--ros-domain-id','100'],['--ros-domain','94']):
            with self.subTest(args=args),patch('sys.stderr'),self.assertRaises(SystemExit):parser.parse_args(args)

    def test_measured_state_preserved_without_loading_plan(self):
        state,geometry=self.fixture()
        original=deepcopy(state)
        self.assertEqual(self.load(state,geometry),state['joints'])
        self.assertEqual(state,original)

    def test_state_fields_and_joint_types_fail_closed(self):
        state,geometry=self.fixture()
        variants=[]
        for key in state:
            bad=deepcopy(state);del bad[key];variants.append(bad)
        for value in (True,'0.1',None,float('nan'),float('inf')):
            bad=deepcopy(state);bad['joints'][ARM_NAMES[0]]=value;variants.append(bad)
        for name in (ARM_NAMES[0],'waist_yaw_joint','right_index_1_joint'):
            bad=deepcopy(state);del bad['joints'][name];variants.append(bad)
        bad=deepcopy(state);bad['joints']['left_index_normalized']=1.;variants.append(bad)
        bad=deepcopy(state);bad['captured_at']='2026-09-17T19:19:02';variants.append(bad)
        for bad in variants:
            with self.subTest(state=bad),self.assertRaises(ValueError):self.load(bad,geometry)

    def test_no_assumed_hands_or_fixed_nonzero_waist(self):
        state,geometry=self.fixture()
        with self.assertRaisesRegex(ValueError,'articulated'):self.load(state,geometry,articulated=False)
        state['joints']['waist_yaw_joint']=.1
        with self.assertRaisesRegex(ValueError,'waist-assist'):self.load(state,geometry,waist=False)

    def test_transform_shape_rigidity_frame_and_consistency(self):
        state,geometry=self.fixture()
        for value in (None,[],np.eye(3).tolist(),np.zeros((4,4)).tolist(),
                      [['1']*4]*4,[[True]*4]*4):
            bad=deepcopy(state);bad['pelvis_from_torso_row_major']=value
            with self.subTest(value=value),self.assertRaises(ValueError):self.load(bad,geometry)
        bad=deepcopy(state);bad['pelvis_from_torso_row_major'][0][3]=.1
        with self.assertRaisesRegex(ValueError,'differ'):self.load(bad,geometry)
        state['joints']['waist_yaw_joint']=.1
        with self.assertRaisesRegex(ValueError,'waist yaw'):self.load(state,geometry)
        for key,value in (('frame_id','pelvis'),('capture_frame',{})):
            bad=deepcopy(geometry);bad[key]=value
            with self.assertRaises(ValueError):self.load(state,bad)

    def test_duplicate_json_fields_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'state.json'
            path.write_text('{"joints": {}, "joints": {}}')
            with self.assertRaisesRegex(ValueError,'Duplicate'):replay.read_archive(path)

    def environment(self,domain):
        profile=str(replay.ROOT/'scripts/task_test_fastdds.xml')
        return dict(ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                    FASTRTPS_DEFAULT_PROFILES_FILE=profile,FASTDDS_DEFAULT_PROFILES_FILE=profile)

    def test_offline_environment_accepts_only_matching_reserved_domain(self):
        inode=S(st_dev=1,st_ino=2)
        with patch.object(replay.os,'fstat',return_value=inode),patch.object(Path,'stat',return_value=inode),\
                patch.object(replay.fcntl,'flock') as lock:
            for domain in range(93,100):
                with patch.dict(os.environ,self.environment(domain),clear=True):
                    replay.require_offline_environment(domain)
            self.assertEqual(lock.call_count,7)
        for domain in (0,92,100):
            with patch.dict(os.environ,self.environment(domain),clear=True),self.assertRaises(RuntimeError):
                replay.require_offline_environment(domain)

    def test_dds_overrides_and_wrong_environment_rejected_before_lock(self):
        for key,value in (('ROS_DISCOVERY_SERVER','192.168.1.1:11811'),('ROS_SUPER_CLIENT','TRUE'),
                          ('FASTDDS_ENVIRONMENT_FILE','remote.json'),('SKIP_DEFAULT_XML','1'),
                          ('ROS_STATIC_PEERS','192.168.1.2'),('CYCLONEDDS_URI','remote.xml'),
                          ('ROS_AUTOMATIC_DISCOVERY_RANGE','SUBNET'),('ROS_LOCALHOST_ONLY','0'),
                          ('ROS_DOMAIN_ID','95'),('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp'),
                          ('FASTDDS_DEFAULT_PROFILES_FILE','remote.xml')):
            env=self.environment(94);env[key]=value
            with self.subTest(key=key),patch.dict(os.environ,env,clear=True),\
                    patch.object(replay.fcntl,'flock') as lock,self.assertRaises(RuntimeError):
                replay.require_offline_environment(94)
            lock.assert_not_called()

    def test_missing_or_wrong_wrapper_lock_rejected(self):
        with patch.dict(os.environ,self.environment(94),clear=True):
            with patch.object(replay.os,'fstat',side_effect=OSError),self.assertRaisesRegex(RuntimeError,'flock'):
                replay.require_offline_environment(94)
            with patch.object(replay.os,'fstat',return_value=S(st_dev=1,st_ino=2)),\
                    patch.object(Path,'stat',return_value=S(st_dev=1,st_ino=3)),\
                    self.assertRaisesRegex(RuntimeError,'flock'):
                replay.require_offline_environment(94)

    def test_wrapper_invalid_domains_exit_before_sourcing_ros(self):
        for name in ('run_task_offline_replay.sh','run_task_offline_gallery.sh'):
            wrapper=replay.ROOT/'scripts'/name
            for args in (['--ros-domain-id','0'],['--ros-domain-id=100'],['--ros-domain-id'],
                         ['--ros-domain-id=94','--ros-domain-id=95']):
                result=subprocess.run(['bash',str(wrapper),*args],capture_output=True,text=True)
                self.assertEqual(result.returncode,2,result.stderr)
                self.assertNotIn('Traceback',result.stderr)

    def test_target_failure_archives_pass_pure_input_validation(self):
        session=replay.ROOT/'data/task_sessions/20260917_191654_927130'
        if not session.is_dir():self.skipTest('Local failure fixtures unavailable')
        for case in ('plan_0018','plan_0020'):
            path=session/case
            state=replay.load_start_state(path/'capture_state.json',replay.read_archive(path/'geometry.json'),True,True)
            self.assertEqual(state,replay.read_archive(path/'capture_state.json')['joints'])

    def test_launch_remains_hardware_disabled_and_has_no_action_client(self):
        source=Path(replay.__file__).read_text()
        for flag in ('start_state_bridge:=false','start_hardware_gateway:=false','hardware_output_enabled:=false',
                     'start_shadow_executor:=false','enable_rviz_interactive_execution:=false'):
            self.assertIn(flag,source)
        self.assertNotIn('ActionClient',source)
        self.assertNotIn('send_goal',source)

    def test_geometry_offsets_and_tcp_preserved_with_explicit_mismatch(self):
        from task_planning_config import PlanningParameters
        geometry=dict(task_preview=dict(radial_offset_m=.03,height_offset_m=.025,arc_center_toward_robot_m=0.),
                      fingertip_estimates={'archived_tcp':[1,2,3]},calibration={'sha256':'archive'})
        original=deepcopy(geometry)
        params=PlanningParameters(radial_offset_mm=30.,height_offset_mm=25.)
        record=replay.archived_geometry_record(geometry,params,require_match=True)
        self.assertTrue(record['yaml_offsets_match_archive'])
        self.assertFalse(record['current_calibration_loaded'])
        with self.assertRaisesRegex(ValueError,'preserve-archive-geometry'):
            replay.archived_geometry_record(geometry,PlanningParameters(),require_match=True)
        record=replay.archived_geometry_record(geometry,PlanningParameters())
        self.assertFalse(record['yaml_offsets_match_archive'])
        self.assertEqual(geometry,original)

    def test_legacy_offsets_need_exact_arc_evidence_and_never_rewrite_geometry(self):
        from scipy.spatial.transform import Rotation
        from task_planning_config import PlanningParameters
        pivot=np.array([.4,.1,.02]);entry=pivot+[.06,0.,0.]
        points=pivot+Rotation.from_euler('z',np.linspace(-120.,120.,121),degrees=True).apply(entry-pivot)
        task=dict(axis_center_m=[.4,.1,0.],insertion_reference_m=entry.tolist(),assumed_normal=[0.,0.,1.],
                  arc_minus120_to_plus120_m=points.tolist(),radial_offset_m=.01)
        geometry=dict(task_preview=task,fingertip_estimates={'original_tcp':[1,2,3]})
        original=deepcopy(geometry)
        record=replay.archived_geometry_record(geometry,PlanningParameters())
        self.assertEqual(record['derived_legacy_offsets'],dict(height_offset_m=.02,arc_center_toward_robot_m=0.))
        self.assertEqual(geometry,original)
        task['arc_minus120_to_plus120_m'][3][0]+=.000001
        with self.assertRaisesRegex(ValueError,'does not prove'):
            replay.archived_geometry_record(geometry,PlanningParameters())
        del task['arc_minus120_to_plus120_m']
        with self.assertRaisesRegex(ValueError,'frozen full arc'):
            replay.archived_geometry_record(geometry,PlanningParameters())

    def test_service_readiness_remains_cancellable_during_cold_start(self):
        client=Mock(srv_name='/offline/service')
        client.wait_for_service.side_effect=[False,False,True]
        check=Mock()
        replay.await_service(client,check,replay.time.monotonic()+2.)
        self.assertEqual(client.wait_for_service.call_count,3)
        self.assertTrue(all(c.kwargs['timeout_sec']<=.2 for c in client.wait_for_service.call_args_list))
        client.wait_for_service.reset_mock()
        with self.assertRaisesRegex(TimeoutError,'deadline'):
            replay.await_service(client,Mock(side_effect=TimeoutError('deadline')),replay.time.monotonic()+60.)
        client.wait_for_service.assert_not_called()

    def test_production_budget_config_is_forwarded_without_changing_config(self):
        from dataclasses import asdict,replace
        from task_planning_config import load_parameters,parameters_from_values,configure_planner
        parameters,_=load_parameters()
        effective=parameters_from_values(asdict(replace(parameters,budget_seconds=60.,max_attempts=600,
                                                       contact_search_seconds=180.)))
        planner=S()
        configure_planner(planner,effective)
        self.assertEqual(planner.search_seconds,60.)
        self.assertEqual(planner.search_attempts,600)
        self.assertEqual(planner.planning_parameters.contact_search_seconds,180.)

    def test_production_adapter_not_guarded_preview_subclass(self):
        import ast
        tree=ast.parse(Path(replay.__file__).read_text())
        branches=[node for node in ast.walk(tree) if isinstance(node,ast.If)
                  and ast.unparse(node.test)=='args.production_planner']
        text='\n'.join(ast.unparse(node) for node in branches)
        self.assertIn('planner_class = ScopedTaskPlanner',text)
        self.assertIn('check_plan=lambda ticket: check()',text)
        self.assertNotIn('StraightPreviewPlanner',text)
        self.assertNotIn('with_radial_offset',Path(replay.__file__).read_text())

    def test_domain_locks_exclude_same_domain_but_not_other_domains(self):
        with tempfile.TemporaryDirectory() as directory:
            a=Path(directory)/'domain94.lock';b=Path(directory)/'domain95.lock'
            with a.open('w') as held:
                replay.fcntl.flock(held,replay.fcntl.LOCK_EX|replay.fcntl.LOCK_NB)
                same=subprocess.run(['flock','-n',str(a),'true'],capture_output=True)
                other=subprocess.run(['flock','-n',str(b),'true'],capture_output=True)
                self.assertEqual(same.returncode,1)
                self.assertEqual(other.returncode,0)

    def test_main_live_environment_fails_before_ros_init(self):
        with patch.dict(os.environ,self.environment(0),clear=True),patch('sys.argv',['replay']),\
                patch.object(replay.rclpy,'init') as init,self.assertRaises(RuntimeError):
            replay.main()
        init.assert_not_called()


class KinematicPreviewTests(unittest.TestCase):
    def test_live_domain_rejected_before_any_ros_client(self):
        with patch.dict(os.environ,{'ROS_DOMAIN_ID':'0','ROS_LOCALHOST_ONLY':'1','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp'}):
            node=Mock()
            with self.assertRaisesRegex(RuntimeError,'offline domain 93'):
                KinematicPreviewPlanner(node,None,None,check_preview=Mock())
            node.create_client.assert_not_called()

    def planner(self):
        p=KinematicPreviewPlanner.__new__(KinematicPreviewPlanner)
        p.check_preview=Mock();p.unchecked_states=0
        p.preview_limits={side:{n:(-1,1) for n in ARMS[side]} for side in ARMS}
        return p

    def test_collision_validator_not_called_but_limits_remain(self):
        p=self.planner();p.call=Mock();q=dict.fromkeys(ARM_NAMES,.1)
        ticket=S(side='left',kind='task')
        p.validate(q,ticket)
        p.call.assert_not_called();p.check_preview.assert_called_once()
        q[ARMS['left'][0]]=.99
        with self.assertRaises(PlanningFailure):p.validate(q,ticket)

    def test_joint_preview_preserves_other_arm(self):
        p=self.planner();p.retime=Mock();q=dict.fromkeys(ARM_NAMES,.1)
        target=dict.fromkeys(ARM_NAMES,.3)
        p.joint_path('left',q,target,S(side='left',kind='task'),'Approach')
        path=p.retime.call_args.args[2].joint_trajectory
        self.assertEqual(list(path.joint_names),list(ARMS['left']))
        np.testing.assert_allclose(path.points[0].positions,[.1]*7)
        np.testing.assert_allclose(path.points[-1].positions,[.3]*7)
        self.assertEqual(q,dict.fromkeys(ARM_NAMES,.1))

    def test_preview_limit_rejection_retries_but_cancellation_propagates(self):
        p=self.planner()
        with patch('task_kinematic_preview.TaskPlanner.solve',side_effect=PlanningFailure('JOINT_MARGIN','insert','limit')):
            self.assertIsNone(p.solve())
        with patch('task_kinematic_preview.TaskPlanner.solve',side_effect=RuntimeError('cancelled')):
            with self.assertRaisesRegex(RuntimeError,'cancelled'):p.solve()


class PlaybackTests(unittest.TestCase):
    def test_rviz_window_matching_is_scoped_and_accepts_modified_title(self):
        from task_gallery_smoke import scoped_rviz_window,render_region
        line='0x600014 "/isolated/gallery.rviz* - RViz": ("rviz2" "rviz2") 1200x850+0+30 +0+30'
        other='0x500014 "/other/gallery.rviz - RViz": ("rviz2" "rviz2") 1200x850+0+0 +0+0'
        window=scoped_rviz_window(other+'\n'+line,'/isolated/gallery.rviz')
        self.assertEqual(window['id'],0x600014)
        self.assertEqual(render_region('0x600011 "rviz2": () 802x741+373+71 +373+101',window),(373,71,802,741))
        with self.assertRaisesRegex(ValueError,'Unique'):scoped_rviz_window(other,'/isolated/gallery.rviz')
        with self.assertRaisesRegex(ValueError,'Unique'):scoped_rviz_window(line+'\n'+line,'/isolated/gallery.rviz')
        with self.assertRaisesRegex(ValueError,'outside'):
            render_region('0x600011 "rviz2": () 802x741+900+71 +900+101',window)

    def test_rviz_viewport_rejects_uniform_background_and_detects_motion(self):
        from task_gallery_smoke import viewport_metrics,visible_viewport,changed_fraction
        blank=np.full((200,300,3),[38,34,32],dtype=np.uint8)
        self.assertFalse(visible_viewport(viewport_metrics(blank)))
        self.assertFalse(visible_viewport(viewport_metrics(np.zeros_like(blank))))
        rendered=blank.copy();rendered[20:180,50:100]=200
        self.assertTrue(visible_viewport(viewport_metrics(rendered)))
        moved=blank.copy();moved[20:180,100:150]=200
        self.assertGreater(changed_fraction(rendered,moved),.1)
        self.assertEqual(changed_fraction(rendered,rendered),0.)

    def report(self):
        def point(q,t):return {'positions':[q],'velocities':[0.],'accelerations':[0.],
                              'time_from_start':{'sec':t,'nanosec':0}}
        return {'mode':'KINEMATIC_PREVIEW_UNCHECKED','offline_only':True,
                'hardware_executable':False,'success':True,'segments':[
                    {'label':'Insert','trajectory':{'joint_names':['arm'], 'points':[point(0,0),point(1,2)]}},
                    {'label':'Hand index_only','trajectory':{'joint_names':['finger'],'points':[point(1,0),point(0,2)]}}]}

    def test_playback_keeps_other_joints_and_hand_posture(self):
        timeline=PreviewTimeline(self.report(),{'arm':0.,'finger':1.,'other':.2})
        q,label=timeline.sample(1.)
        self.assertEqual(label,'Insert');self.assertAlmostEqual(q['arm'],.5)
        self.assertEqual(q['other'],.2);self.assertEqual(q['finger'],1.)
        q,label=timeline.sample(3.)
        self.assertEqual(q['arm'],1.);self.assertAlmostEqual(q['finger'],.5)
        self.assertEqual(timeline.sample(10)[0],{'arm':1.,'finger':0.,'other':.2})

    def test_failed_or_hardware_reports_never_play(self):
        for key,value in (('success',False),('offline_only',False),('hardware_executable',True),('mode','PLAN_READY')):
            report=self.report();report[key]=value
            with self.assertRaises(ValueError):PreviewTimeline(report,{'arm':0.,'finger':1.})

    def test_discontinuous_segment_rejected(self):
        report=self.report();report['segments'][1]['trajectory']['points'][0]['positions']=[.5]
        with self.assertRaisesRegex(ValueError,'Discontinuous'):
            PreviewTimeline(report,{'arm':0.,'finger':1.})


if __name__=='__main__':unittest.main()
