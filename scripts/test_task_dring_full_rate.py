"""High-rate capture and replay tests with in-memory LowState, no robot I/O."""
import copy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

import numpy as np

from task_rail_receiver import Recorder
from task_rail_capture import DirectCapture
from task_stage_control import validate_gateway_gains
from task_planning_config import PlanningParameters
import task_dring_teach_data as data
from test_task_dring_demo import fixture


class FullRateTests(unittest.TestCase):
    def test_real_lowstate_numeric_arrays_are_json_serializable(self):
        from unitree_hg.msg import LowState
        with TemporaryDirectory() as directory:
            path=Path(directory)
            (path/'request.json').write_text(json.dumps(dict(sample_policy='all_received',
                motors={'joint':15},limits={'joint':[-2,2]},initial={'joint':0},
                moving=['joint'],left=[],passive=[])))
            rec=Recorder(path);self.addCleanup(rec.stream.close)
            msg=LowState();msg.mode_machine=6;msg.mode_pr=0;msg.tick=1
            msg.motor_state[15].vol=48.
            rec.sample(msg,dict(source_timestamp=10**9,received_timestamp=0))
            self.assertIsNone(rec.error)
            rec.status(force=True)
            row=json.loads((path/'samples.jsonl').read_text())
            self.assertEqual(row['motor_status']['joint']['temperature'],[0,0])

    def test_all_distinct_ticks_and_telemetry_are_stored_without_distance_filter(self):
        with TemporaryDirectory() as directory:
            path=Path(directory)
            (path/'request.json').write_text(json.dumps(dict(sample_policy='all_received',
                motors={'joint':0},limits={'joint':[-2,2]},initial={'joint':0},
                moving=['joint'],left=[],passive=[])))
            rec=Recorder(path)
            self.addCleanup(rec.stream.close)
            motor=S(q=0.,dq=.02,vol=48.,tau_est=1.25,mode=1,
                    temperature=[40,41],sensor=[2,3],motorstate=0)
            msg=S(mode_machine=6,mode_pr=0,tick=0,motor_state=[motor])
            for i in range(1001):
                msg.tick=i;motor.q=i*1e-7
                info=dict(source_timestamp=10**9+i*10**6,received_timestamp=0)
                with patch('task_rail_receiver.time.monotonic',return_value=10.+i*.001):
                    rec.sample(msg,info)
                    rec.sample(msg,info)  # Repeated tick is not a new physical sample.
            self.assertIsNone(rec.error)
            self.assertEqual(rec.count,1001)
            with patch('task_rail_receiver.time.monotonic',return_value=11.001):
                rec.finish(11.);rec.status(force=True)
            rows=[json.loads(line) for line in (path/'samples.jsonl').read_text().splitlines()]
            self.assertEqual(len(rows),1001)
            self.assertEqual(rows[500]['tick'],500)
            self.assertEqual(rows[500]['torques_nm']['joint'],1.25)
            self.assertEqual(rows[500]['velocities']['joint'],.02)
            self.assertEqual(rows[500]['motor_status']['joint']['temperature'],[40,41])
            status=json.loads((path/'status.json').read_text())
            self.assertAlmostEqual(status['stored_hz'],1000.)
            self.assertEqual(status['sample_policy'],'all_received')
            self.assertTrue(status['done'])

    def test_full_rate_finish_preserves_archive_and_exact_click_cutoff(self):
        model,q,_=fixture()
        with TemporaryDirectory() as directory,patch('task_rail_capture.subprocess.Popen') as popen:
            popen.return_value.poll.return_value=0
            capture=DirectCapture(S(run_dir=Path(directory),model=model),'left',q,1,0,
                                  side='left',full_rate=True)
            try:
                rows=[dict(joints=q,received_monotonic=10.+i*.001,
                    source_timestamp_ns=10**9+i*10**6,tick=i,torques_nm={'extra':1.}) for i in range(801)]
                archive=capture.directory/'samples.jsonl'
                archive.write_text('\n'.join(json.dumps(r) for r in rows))
                before=archive.read_bytes()
                (capture.directory/'status.json').write_text(json.dumps(dict(done=True)))
                capture.request_finish(10.5)
                raw=capture.finish(lambda:None)
                self.assertEqual(len(raw['positions']),501)
                self.assertEqual(raw['tick'][-1],500)
                self.assertAlmostEqual(raw['time_s'][-1],.5)
                self.assertEqual(raw['sample_policy'],'all_received')
                self.assertEqual(Path(raw['telemetry_path']),archive)
                self.assertEqual(before,archive.read_bytes())
            finally:capture.close()

    def test_dense_recording_over_old_capacity_fits_all_samples_and_keeps_raw(self):
        model,q,config=fixture()
        trace=config['traces']['left'];t=np.linspace(0,32,32001);u=t/t[-1]
        positions=np.zeros((len(t),8));positions[:,0]=.12*u**3*(10-15*u+6*u*u)
        trace['raw']=dict(time_s=t.tolist(),positions=positions.tolist(),sample_policy='all_received',
                          stamp_ns=[10**9+round(v*1e9) for v in t])
        before=copy.deepcopy(trace)
        tr,report=data.fit_trace(trace,model,'left')
        self.assertEqual(report['raw_count'],32001)
        self.assertEqual(report['fit_tolerance_rad'],.001)
        self.assertLessEqual(report['fit_error_rad'],.001)
        self.assertAlmostEqual(report['stored_hz'],1000.)
        self.assertAlmostEqual(data.time_seconds(tr.points[-1]),32.)
        self.assertEqual(trace,before)

    def test_full_rate_replay_speed_changes_time_not_joint_path(self):
        model,q,config=fixture()
        config['traces']['left']['raw']['sample_policy']='all_received'
        plans=[data.build(model,config,q,'left',PlanningParameters(),.7,.7,factor) for factor in (.5,1.)]
        a,b=[p['segments'][-1]['trajectory'] for p in plans]
        self.assertEqual(len(a.points),len(b.points))
        for pa,pb in zip(a.points,b.points):
            np.testing.assert_allclose(pa.positions,pb.positions,atol=1e-12)
            self.assertAlmostEqual(data.time_seconds(pa),2*data.time_seconds(pb),places=7)
            np.testing.assert_allclose(pa.velocities,np.array(pb.velocities)/2,atol=1e-12)

    def test_wrist_override_rejects_old_gateway_but_not_normal_legacy_stage(self):
        params=PlanningParameters();gateway=dict(task_stage_supported='true')
        validate_gateway_gains(params,gateway)
        with self.assertRaisesRegex(RuntimeError,'wrist'):
            validate_gateway_gains(params,gateway,wrist_kp=110.)
        gateway['task_stage_wrist_supported']='true'
        validate_gateway_gains(params,gateway,wrist_kp=110.)
        for invalid in (-1,111,float('nan')):
            with self.assertRaises(RuntimeError):validate_gateway_gains(params,gateway,wrist_kp=invalid)

    def test_distal_wrist_override_requires_new_gateway_and_bounded_value(self):
        from task_planning_config import parameters_from_values
        params=parameters_from_values(dict(demo_wrist_kp=110.,demo_distal_wrist_kp=220.))
        gateway=dict(task_stage_supported='true',task_stage_wrist_supported='true')
        with self.assertRaisesRegex(RuntimeError,'distal wrist'):
            validate_gateway_gains(params,gateway,wrist_kp=110.,distal_wrist_kp=220.)
        gateway['task_stage_distal_wrist_supported']='true'
        validate_gateway_gains(params,gateway,wrist_kp=110.,distal_wrist_kp=220.)
        for invalid in (-1,220.1,float('inf'),float('nan')):
            with self.assertRaises(ValueError):parameters_from_values(dict(demo_distal_wrist_kp=invalid))
            with self.assertRaises(RuntimeError):validate_gateway_gains(params,gateway,distal_wrist_kp=invalid)

    def test_per_axis_kp_rejects_old_gateway_invalid_shape_and_nonfinite_values(self):
        params=PlanningParameters();gateway=dict(task_stage_supported='true')
        with self.assertRaisesRegex(RuntimeError,'Per-joint'):
            validate_gateway_gains(params,gateway,arm_kp=[220.]*7)
        gateway['task_stage_arm_kp_supported']='true'
        validate_gateway_gains(params,gateway,arm_kp=[550.,550.,550.,550.,110.,220.,230.])
        for values in ([],[1.]*6,[1.]*8,[float('nan')]*7,[-1.]*7,[1e39]*7):
            with self.assertRaises(RuntimeError):validate_gateway_gains(params,gateway,arm_kp=values)


if __name__=='__main__':unittest.main()
