import copy
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import sys

from bundle import ROOT, digest, write
from coordinator import Coordinator, Rejected
from executor import SequenceExecutor, materialize
from releases import (ReleaseStore, action_name, describe, load_release, load_source, package,
                      planned_action, validate)
from upload_release import upload

sys.path.insert(0, str(ROOT / 'scripts'))
import task_jetson_push as push_api
from task_screen_teach import fingerprint
from task_hand_control import ORDER, OPEN, CLOSED


def fixture(slot='left_3', full=True):
    names = ['left_' + n + '_joint' for n in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw',
        'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')] + ['waist_yaw_joint']
    points = [dict(positions=[x] * 8, velocities=[0.] * 8, accelerations=[0.] * 8,
                   time_from_start=dict(sec=i, nanosec=0)) for i, x in enumerate((0., .1))]
    tr = dict(joint_names=names, points=points)
    seg = dict(kind='arm', side='left', label='recording', recording_fit={'ok':True}, duration=1.,
               trajectory=tr, start=dict(zip(names, points[0]['positions'])), end=dict(zip(names, points[-1]['positions'])))
    config = dict(identity=dict(robot_sha256='a' * 64,
        hand_mapping=dict(order=list(ORDER), open=list(OPEN), closed=list(CLOSED))), traces={})
    plan = dict(kind='dring_demo', arm='left', hardware_executable=True,
                demo=dict(operation=slot, full_cycle=full, revision=0, config_sha256=fingerprint(config)),
                planning_config=dict(values={}), segments=[seg])
    init = dict(mode='simultaneous', velocity_scale=.7, acceleration_scale=.7,
                target={**dict(zip(names, [0.] * 8)), **{n.replace('left_', 'right_'):0. for n in names}})
    return plan, config, init


def envelope(version='v1', slot='left_3', full=True, kind='full'):
    plan, config, init = fixture(slot, full)
    name, action = planned_action(plan, config, 'session/dring_demo_0001')
    return package(version, {name:action}, init, kind=kind)


class ReleaseTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.store = ReleaseStore(self.root / 'releases')

    def tearDown(self):
        self.temp.cleanup()

    def test_wow_mapping_cannot_dispatch_an_unrelated_module(self):
        e = envelope()
        e['payload']['mapping']['AIR'] = 'dring_left'
        e['sha256'] = digest(e['payload'])
        with self.assertRaises(ValueError):
            validate(e)

    def test_plan_ready_without_execution_is_publishable_but_not_verified(self):
        e = envelope()
        self.store.install(e)
        result = self.store.load('v1')
        self.assertFalse(result['payload']['hardware_enabled'])
        self.assertEqual(result['payload']['actions']['dring_left']['verification'], 'planned')
        self.assertEqual(describe(result)['actions']['dring_left']['slot'], 'left_3')

    def test_single_segment_is_not_a_complete_cycle_binding(self):
        e = envelope(full=False)
        self.assertEqual(list(e['payload']['actions']), ['dring_left_segment'])

    def test_reference_array_is_not_modified(self):
        plan, config, init = fixture()
        name, a = planned_action(plan, config, 'session/plan')
        self.assertEqual(a['program'][0]['trajectory'], plan['segments'][0]['trajectory'])
        plan['segments'][0]['trajectory']['points'][0]['positions'][0] = .5
        self.assertEqual(a['program'][0]['trajectory']['points'][0]['positions'][0], 0.)

    def test_version_conflict_rejected_and_same_upload_idempotent(self):
        e = envelope(); self.store.install(e); self.store.install(e)
        other = envelope(slot='left_4')
        with self.assertRaises(ValueError):self.store.install(other)
        self.assertEqual(self.store.load('v1')['sha256'], e['sha256'])

    def test_corrupted_or_changed_program_refused(self):
        e = envelope()
        e['payload']['actions']['dring_left']['program'][0]['trajectory']['points'][0]['positions'][0] = .1
        with self.assertRaises(ValueError):validate(e)
        e['sha256'] = digest(e['payload'])
        with self.assertRaises(ValueError):validate(e)

    def test_execution_proof_may_not_be_invented(self):
        e = envelope();e['payload']['actions']['dring_left']['verification'] = 'executed'
        e['sha256'] = digest(e['payload'])
        with self.assertRaises(ValueError):validate(e)

    def test_unrestricted_preview_refused(self):
        plan, config, _ = fixture(); plan['preview_only_timing'] = True
        with self.assertRaises(ValueError):planned_action(plan, config, 's/p')

    def test_source_traversal_and_version_traversal_refused(self):
        with self.assertRaises(ValueError):self.store.path('../outside')
        with self.assertRaises(ValueError):load_source(self.root, '../outside')

    def test_module_patch_preserves_other_action_and_old_release(self):
        base = envelope('base', 'right')
        candidate = envelope('candidate', kind='patch')
        original = copy.deepcopy(base)
        composed = self.store.compose(base, candidate)
        self.assertEqual(set(composed['payload']['actions']), {'dring_left', 'dring_right'})
        self.assertEqual(composed['payload']['actions']['dring_right'], base['payload']['actions']['dring_right'])
        self.assertEqual(base, original)
        self.assertEqual(composed['payload']['release_kind'], 'full')

    def test_patch_cannot_silently_replace_preparation_or_model(self):
        base = envelope('base');candidate = envelope('patch', kind='patch')
        candidate['payload']['initialization']['target']['waist_yaw_joint'] = .5
        candidate['sha256'] = digest(candidate['payload'])
        with self.assertRaises(ValueError):self.store.compose(base, candidate)

    def test_symlink_release_is_refused(self):
        self.store.install(envelope())
        (self.store.root / 'alias').symlink_to(self.store.root / 'v1')
        with self.assertRaises(ValueError):self.store.load('alias')

    def test_upload_checks_remote_hash_and_never_activates(self):
        self.store.install(envelope())
        e = self.store.load('v1');calls=[]
        def run(args, **kwargs):
            calls.append(args)
            return SimpleNamespace(stdout=json.dumps(describe(e)) if len(calls)==3 else '')
        receipt=upload(self.store.path('v1'),'unitree@192.168.0.249',
                       '/home/unitree/rh56dfx/workspace/execution/test',run=run)
        self.assertFalse(receipt['activated']);self.assertEqual(receipt['hardware_commands_sent'],0)
        self.assertIn('import', calls[-1][-1]);self.assertNotIn('activate', str(calls))
        def corrupt(*args,**kwargs):return SimpleNamespace(stdout=json.dumps(dict(sha256='wrong')))
        with self.assertRaises(RuntimeError):upload(self.store.path('v1'),'unitree@192.168.0.249',
                       '/home/unitree/rh56dfx/workspace/execution/test',run=corrupt)

    def test_upload_rejects_shell_or_global_destination(self):
        self.store.install(envelope())
        for target, root in [('unitree@host;touch x','/tmp/x'), ('unitree@host','/etc/x')]:
            with self.assertRaises(ValueError):upload(self.store.path('v1'),target,root)

    def test_release_switch_pins_tasks_blocks_busy_and_requires_initialization(self):
        c=Coordinator(envelope(),self.root/'state.db',clock=lambda:0.)
        try:
            c.submit(dict(taskId='init',taskType='ROBOT_INITIALIZE',parameters={}))
            with self.assertRaises(Rejected):c.activate_release(envelope('next'),c.bundle['sha256'])
            self.assertEqual(c.task('init')['releaseSha256'], c.bundle['sha256'])
            c.tick(); c.next_tick=0.; c.tick()
            old=c.bundle['sha256'];c.activate_release(envelope('next'),old)
            self.assertEqual(c.state['phase'],'STARTUP_REQUIRED')
            self.assertEqual(c.state['previous_release']['version'],'v1')
            with self.assertRaises(Rejected):c.activate_release(envelope('third'),old)
        finally:c.close()

    def test_nws_counts_whole_groups_only(self):
        base=envelope();other=envelope('right','right',kind='patch')
        combined=self.store.compose(base,other)
        c=Coordinator(combined,self.root/'state.db',clock=lambda:0.)
        try:
            c.state['phase']='READY'
            c.submit(dict(taskId='nws',taskType='NWS_CYCLE',parameters={'count':2}))
            c.tick();c.next_tick=0.;c.tick()
            self.assertEqual(c.task('nws')['progress'],0)
            c.next_tick=0.;c.tick();self.assertEqual(c.task('nws')['progress'],1)
            c.stop();self.assertEqual(c.task('nws')['progress'],1)
        finally:c.close()


class SnapshotTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        plan, config, init=fixture()
        directory=self.root/'data/task_sessions/session/dring_demo_0001'
        plan['directory']=str(directory)
        write(directory/'plan.json',plan);write(directory/'config.json',config)
        self.node=SimpleNamespace(lock=threading.RLock(),
            session=SimpleNamespace(plan=plan,phase='PLAN_READY',generation=3),
            demo=SimpleNamespace(config=config,revision=0,recording=None),
            prepared={side:dict(joints={n:v for n,v in init['target'].items() if n.startswith(side)})
                      for side in ('left','right')})
        self.patch=patch.object(push_api,'ROOT',self.root);self.patch.start()

    def tearDown(self):
        self.patch.stop();self.temp.cleanup()

    def test_snapshot_exact_current_ready_plan_without_mutation(self):
        before=copy.deepcopy(self.node.session.plan)
        name, action, _=push_api.snapshot(self.node,'dring_demo',3,convert=copy.deepcopy)
        self.assertEqual(name,'dring_left');self.assertEqual(action['plan'],before)
        self.assertEqual(self.node.session.phase,'PLAN_READY');self.assertEqual(self.node.session.generation,3)

    def test_old_generation_and_wrong_module_refused(self):
        with self.assertRaises(RuntimeError):push_api.snapshot(self.node,'dring_demo',2,convert=copy.deepcopy)
        self.assertIsNotNone(push_api.issue(self.node,'rail'))

    def test_changed_during_serialization_refused(self):
        def convert(plan):
            self.node.session.generation+=1
            return copy.deepcopy(plan)
        with self.assertRaises(RuntimeError):push_api.snapshot(self.node,'dring_demo',3,convert=convert)

    def test_current_config_change_and_saved_plan_change_refused(self):
        self.node.demo.config['traces']['changed']=True
        with self.assertRaises(RuntimeError):push_api.snapshot(self.node,'dring_demo',3,convert=copy.deepcopy)
        self.node.demo.config['traces'].clear()
        self.node.session.plan['segments'][0]['trajectory']['points'][0]['positions'][0]=.01
        with self.assertRaises(RuntimeError):push_api.snapshot(self.node,'dring_demo',3,convert=copy.deepcopy)

    def test_execution_and_recording_disable_push(self):
        self.node.session.phase='EXECUTING';self.assertIsNotNone(push_api.issue(self.node,'dring_demo'))
        self.node.session.phase='PLAN_READY';self.node.demo.recording={}
        self.assertIsNotNone(push_api.issue(self.node,'dring_demo'))


class FakeTransport:
    def __init__(self):self.calls=[];self.fail=False;self.stop_ack=True
    def assert_authority(self):self.calls.append('authority')
    def verify_configuration(self,action):self.calls.append('configuration')
    def begin(self,action,check):check()
    def stationary(self,check):check();return fixture()[0]['segments'][0]['start']
    def set_stage(self,*args):self.calls.append('gains')
    def align_check(self,*args):self.calls.append('align')
    def arm(self,step,trajectory,check):
        self.calls.append('arm')
        if self.fail:raise TimeoutError('Unknown action result')
        return dict(status=4,error_code=0)
    def settle(self,*args):self.calls.append('settled')
    def stop_and_confirm(self):self.calls.append('stop');return self.stop_ack


class ExecutorTests(unittest.TestCase):
    def test_success_waits_for_action_and_settling(self):
        t=FakeTransport();executor=SequenceExecutor(t,None)
        result=executor.execute(envelope(),'dring_left')
        self.assertEqual(result['status'],'SUCCEEDED')
        self.assertEqual(t.calls.count('arm'),1)
        self.assertIn('settled',t.calls)

    def test_unknown_result_not_retried_and_unknown_stop_not_success(self):
        t=FakeTransport();t.fail=True;t.stop_ack=False
        executor=SequenceExecutor(t,None);result=executor.execute(envelope(),'dring_left')
        self.assertEqual(result['status'],'STOP_UNCONFIRMED');self.assertEqual(t.calls.count('arm'),1)
        executor.execute(envelope(),'dring_left');self.assertEqual(t.calls.count('arm'),1)

    def test_cancelled_executor_never_resumes_contact_segment(self):
        t=FakeTransport();executor=SequenceExecutor(t,None);executor.cancel()
        self.assertEqual(executor.execute(envelope(),'dring_left')['status'],'CANCELLED')
        self.assertNotIn('arm',t.calls)

    def test_materialization_preserves_recorded_array(self):
        e=envelope();q=fixture()[0]['segments'][0]['start']
        result=materialize(e,'dring_left',q,None)
        self.assertEqual(result['steps'][0]['trajectory'], e['payload']['actions']['dring_left']['program'][0]['trajectory'])
        self.assertEqual(result['hardware_commands_sent'],0)


if __name__ == '__main__':unittest.main()
