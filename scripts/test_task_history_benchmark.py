"""Offline history coverage and hard-deadline safety regressions."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from task_history_benchmark import supervise,parallel_cases,coverage_metrics,run_case
from task_history_manifest import build, digest, encoded, inside
from task_history_evaluation import semantics,build as evaluation,materialize


class HistoryTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)

    def write(self,name,value):
        path=self.root/name
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_bytes(encoded(value))

    def fixture(self):
        self.write('pairs/a/rgb.png',{'fixture':'opaque image bytes'})
        pair=dict(id='a',directory='pairs/a',sha256={'rgb.png':digest(self.root/'pairs/a/rgb.png')},
                  calibration={},sources=[],valid_3d=False)
        self.write('pairs.jsonl',pair)
        # A JSONL single record needs a single physical line.
        (self.root/'pairs.jsonl').write_text(json.dumps(pair)+'\n')
        (self.root/'cases.jsonl').write_text('')
        self.write('summary.json',{})
        return pair

    def test_detector_failure_without_context_stays_in_denominator(self):
        self.fixture()
        result=build(self.root)
        self.assertEqual(result['summary']['total_pairs'],1)
        row=result['pairs'][0]
        self.assertEqual(row['applicability'],'unresolved')
        self.assertEqual(row['status'],'blocked')
        self.assertTrue(row['denominator_retained'])
        self.assertFalse(result['policy']['final_task_denominator_frozen'])

    def test_null_ready_and_unknown_direction_are_explicit_blockers(self):
        self.fixture()
        for name in ('capture_state.json','geometry.json','manifest.json'):
            self.write('contexts/session/plan/'+name,{})
        self.write('contexts/session/ready_poses.json',{'left':None})
        context=dict(pair_id='a',directory='contexts/session/plan',context_integrity_issues=[],
                     planning_replay_candidate=True,recorded_kind='task',recorded_arm='left',
                     recorded_direction=None)
        (self.root/'cases.jsonl').write_text(json.dumps(context)+'\n')
        result=build(self.root)
        blockers=result['pairs'][0]['contexts'][0]['blockers']
        self.assertIn('historical_ready_missing',blockers)
        self.assertIn('historical_direction_missing',blockers)
        self.assertIn('historical_home_missing',blockers)
        self.assertEqual(result['summary']['replayable_contexts'],0)

    def test_input_mutation_and_path_escape_fail_closed(self):
        self.fixture()
        (self.root/'pairs/a/rgb.png').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'checksum mismatch'):build(self.root)
        with self.assertRaisesRegex(ValueError,'outside dataset'):inside(self.root,'../escape')

    def test_center_check_is_not_sweep_and_legacy_plan_requires_both(self):
        self.write('plan/plan.json',{'task_schema':'index_center_check_v1'})
        context=dict(direction=None,recorded_kind='task')
        self.assertEqual(semantics(context,self.root/'plan'),('center_check',[]))
        self.write('plan/plan.json',{'segments':[{'label':'Left 120'},{'label':'Right 120'}]})
        self.assertEqual(semantics(context,self.root/'plan'),('legacy_dual_sweep',['left','right']))
        context['direction']='left'
        self.assertEqual(semantics(context,self.root/'plan'),('conflicting_direction',[]))

    def test_byte_exact_recovery_preserves_original_and_both_directions(self):
        pair=self.fixture()
        directory='contexts/session/plan'
        for name in ('capture_state.json','cloud.npz','capture.json'):
            self.write(directory+'/'+name,{})
        self.write(directory+'/geometry.json',{'geometry_valid_for_planning_seed':True})
        self.write('pairs/a/intrinsics.json',{'fx':500})
        expected=digest(self.root/'pairs/a/intrinsics.json')
        self.write(directory+'/manifest.json',{'sha256':{'intrinsics.json':expected}})
        self.write(directory+'/plan.json',{'segments':[{'label':'Left 120'},{'label':'Right 120'}]})
        self.write('contexts/session/ready_poses.json',{'left':{'joints':{'joint':0.}}})
        self.write('contexts/session/session_home.json',{'joints':{'joint':0.}})
        context=dict(pair_id='a',directory=directory,context_integrity_issues=['missing:intrinsics.json'],
                     planning_replay_candidate=False,recorded_kind='task',recorded_arm='left',recorded_direction=None)
        (self.root/'cases.jsonl').write_text(__import__('json').dumps(context)+'\n')
        inventory=build(self.root)
        review=dict(pairs=[dict(pair_id='a',number=0,raw_rgb_sha256=pair['sha256']['rgb.png'],visual_category='ring_visible')])
        result=evaluation(inventory,review);row=result['pairs'][0]
        self.assertEqual(result['summary']['byte_exact_recoverable_files'],{'intrinsics.json':1})
        self.assertEqual(result['summary']['replayable_contexts'],2)
        self.assertEqual(row['status'],'pending')
        self.assertEqual([t['direction'] for t in row['required_tasks']],['left','right'])
        restored=materialize(self.root,row['contexts'][0],self.root/'derived')
        self.assertEqual(digest(restored/'intrinsics.json'),expected)
        self.assertFalse((self.root/directory/'intrinsics.json').exists())
        self.assertTrue((restored.parent/'session_home.json').is_file())
        one=dict(case_id=row['required_tasks'][0]['case_id'],complete_archive_pass=True)
        self.assertEqual(coverage_metrics(result,[one])['complete_archive_pairs'],0)
        two=dict(case_id=row['required_tasks'][1]['case_id'],complete_archive_pass=True)
        self.assertEqual(coverage_metrics(result,[one,two])['complete_archive_pairs'],1)
        self.assertFalse(coverage_metrics(result,[one,two])['final_95_claim_eligible'])
        (self.root/'pairs/a/intrinsics.json').write_text('changed')
        with self.assertRaisesRegex(ValueError,'Recovery input changed'):
            materialize(self.root,row['contexts'][1],self.root/'bad-derived')

    def test_unresolved_ring_pair_is_not_removed_by_detector_failure(self):
        pair=self.fixture();inventory=build(self.root)
        review=dict(pairs=[dict(pair_id='a',number=0,raw_rgb_sha256=pair['sha256']['rgb.png'],visual_category='ring_visible')])
        result=evaluation(inventory,review)
        self.assertEqual(result['summary']['coverage_denominator'],1)
        self.assertEqual(result['pairs'][0]['pair_blockers'],['historical_sweep_direction_unrecorded'])
        self.assertEqual(coverage_metrics(result,[])['conservative_coverage_rate'],0.)
        review['pairs'][0]['visual_category']='no_target_calibration_board'
        self.assertEqual(evaluation(inventory,review)['summary']['coverage_denominator'],0)

    def test_supervisor_success_and_hard_timeout(self):
        result=supervise([sys.executable,'-c','print("done")'],self.root/'ok.log',2.)
        self.assertEqual(result['returncode'],0)
        self.assertFalse(result['timed_out'])
        pidfile=self.root/'grandchild.pid'
        code=('import subprocess,sys,time,pathlib; '
              'p=subprocess.Popen([sys.executable,"-c","import time; time.sleep(30)"]); '
              f'pathlib.Path({str(pidfile)!r}).write_text(str(p.pid)); time.sleep(30)')
        result=supervise([sys.executable,'-c',code],self.root/'timeout.log',.5)
        self.assertTrue(result['timed_out'])
        self.assertLess(result['worker_wall_seconds'],2.)
        self.assertNotEqual(result['returncode'],0)
        self.assertGreater(result['resources']['peak_sampled_group_rss_bytes'],0)
        self.assertGreaterEqual(result['resources']['peak_sampled_process_count'],2)
        self.assertTrue(pidfile.is_file())
        stat=Path('/proc')/pidfile.read_text()/'stat'
        # A killed orphan may briefly remain a zombie until PID 1 reaps it.
        for _ in range(100):
            try:
                if stat.read_text().split(') ')[1][0]=='Z':break
            except FileNotFoundError:break
            time.sleep(.01)
        else:self.fail('Worker descendant survived the deadline')

    def test_numerical_threads_are_bounded_inside_worker(self):
        code='import os; assert os.environ["OPENBLAS_NUM_THREADS"]=="1"; assert os.environ["OMP_NUM_THREADS"]=="1"'
        result=supervise([sys.executable,'-c',code],self.root/'threads.log',2.)
        self.assertEqual(result['returncode'],0)

    def test_case_command_propagates_explicit_preinsert_policy_and_default(self):
        config=self.root/'config.yaml';config.write_text('{}')
        context=dict(directory='capture',direction='right',arm='left',input_sha256={})
        row=dict(pair_id='a',scene_group='fixture',recorded_dates=[])
        def completed(command,log,*args):
            replay=log.parent/'replay';replay.mkdir()
            (replay/'report.json').write_bytes(encoded(dict(success=True,segments=[
                dict(stage=stage) for stage in ('approach','insert','rotate','withdraw','return_ready')])))
            return dict(worker_wall_seconds=.1,returncode=0,timed_out=False,cancelled=False)
        for policy in ('legacy','diverse'):
            options={} if policy=='legacy' else {'preinsert_policy':policy}
            with patch('task_history_benchmark.materialize',return_value=self.root/'capture'),\
                    patch('task_history_benchmark.supervise',side_effect=completed):
                result=run_case(self.root,row,context,self.root/policy,94,180.,config,**options)
            command=result['command']
            self.assertEqual(command[command.index('--preinsert-policy')+1],policy)
            self.assertTrue(result['planner_pass'])

    def test_parallel_domains_are_exclusive_and_results_not_dropped(self):
        lock=threading.Lock();active=set();peak=[0];cancel=threading.Event()
        def run(index,item,domain):
            with lock:
                self.assertNotIn(domain,active)
                active.add(domain);peak[0]=max(peak[0],len(active))
            time.sleep(.03)
            with lock:active.remove(domain)
            return dict(value=item,domain=domain)
        results=list(parallel_cases(list(range(8)),[94,95,96,97],run,cancel))
        self.assertEqual(sorted(i for i,_ in results),list(range(8)))
        self.assertEqual(peak[0],4)
        self.assertTrue(all(r['worker_count']==4 and r['queue_wait_seconds']>=0 for _,r in results))
        self.assertEqual(active,set())

    def test_parallel_cancel_reaps_running_processes(self):
        cancel=threading.Event()
        def run(index,item,domain):
            return supervise([sys.executable,'-c','import time; time.sleep(30)'],
                             self.root/f'cancel-{index}.log',30,cancel)
        timer=threading.Timer(.2,cancel.set);timer.start()
        started=time.monotonic()
        results=list(parallel_cases([1,2],[94,95],run,cancel))
        timer.join()
        self.assertLess(time.monotonic()-started,2.)
        self.assertEqual(len(results),2)
        self.assertTrue(all(r['cancelled'] for _,r in results))


if __name__=='__main__':unittest.main()
