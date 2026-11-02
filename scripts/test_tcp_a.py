import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from tcp_a_core import (DEFAULT_LIMITS, CONFIRMATIONS, synthetic_session, new_session,
                        append_sample, solve, point, metrics, stability, transform,
                        tracking, save_session, load_session, export_session, invalidate, checked_limits, average_observation)


class PivotTests(unittest.TestCase):
    def test_recovery_and_heldout(self):
        session=synthetic_session(); result=solve(session)
        expected=np.array(session['s_model'])+[.008,-.004,.005]
        np.testing.assert_allclose(result['s_candidate_m'],expected,atol=.00015)
        self.assertEqual(result['rank'],6)
        self.assertLess(result['fit_new']['rms_m'],.0003)
        v=result['validation']['fixture_point']
        self.assertLess(v['new']['rms_m'],v['old']['rms_m']/10)

    def test_identical_orientation_degenerate(self):
        s=synthetic_session()
        for r in s['samples']:r['G']=np.eye(4).tolist()
        with self.assertRaisesRegex(ValueError,'覆盖不足'):solve(s)

    def test_single_axis_degenerate(self):
        s=synthetic_session()
        for i,r in enumerate(s['samples']):
            g=np.eye(4);g[:3,:3]=Rotation.from_euler('z',i*.1).as_matrix();r['G']=g.tolist()
        with self.assertRaisesRegex(ValueError,'覆盖不足'):solve(s)

    def test_small_angles_ill_conditioned(self):
        s=synthetic_session()
        for i,r in enumerate(s['samples']):
            g=np.eye(4);g[:3,:3]=Rotation.from_rotvec(np.random.default_rng(i).normal(size=3)*1e-5).as_matrix();r['G']=g.tolist()
        with self.assertRaisesRegex(ValueError,'覆盖不足'):solve(s)

    def test_target_ids_not_pooled(self):
        s=synthetic_session();s['samples'][0]['target_id']='another'
        with self.assertRaisesRegex(ValueError,'一个实体点'):solve(s)

    def test_validation_does_not_refit(self):
        s=synthetic_session();first=solve(s)
        for r in s['samples']:
            if r['group']=='validation':r['G'][0][3]+=.02
        second=solve(s)
        np.testing.assert_array_equal(first['s_candidate_m'],second['s_candidate_m'])
        np.testing.assert_array_equal(first['fit_center_m'],second['fit_center_m'])
        self.assertGreater(second['validation']['fixture_point']['new']['rms_m'],.019)

    def test_new_target_consistency_only(self):
        s=synthetic_session()
        for r in s['samples']:
            if r['group']=='validation':r['target_id']='new_point';r['G'][0][3]+=.12
        v=solve(s)['validation']['new_point']
        self.assertFalse(v['same_target']);self.assertLess(v['new']['rms_m'],.0004)

    def test_outlier_retained(self):
        s=synthetic_session();s['samples'][0]['G'][0][3]+=.04
        result=solve(s)
        self.assertTrue(any('异常接触' in w for w in result['warnings']))
        self.assertFalse(any(r['excluded'] for r in s['samples']))

    def test_exclude_and_restore(self):
        s=synthetic_session();s['samples'][0]['excluded']=True
        self.assertEqual(solve(s)['fit_new']['n'],11)
        s['samples'][0]['excluded']=False
        self.assertEqual(solve(s)['fit_new']['n'],12)

    def test_no_validation_warning(self):
        s=synthetic_session();s['samples']=s['samples'][:12]
        self.assertTrue(any('尚无独立' in w for w in solve(s)['warnings']))

    def test_wrong_actual_source(self):
        s=new_session(mode='live',model=dict(sha256='synthetic'))
        r=synthetic_session()['samples'][0]
        for source in ('command','demo','joint_states_default'):
            r['source']=source
            with self.assertRaisesRegex(ValueError,'禁止'):append_sample(s,r)

    def test_missing_contact_confirmation(self):
        s=synthetic_session();r=copy.deepcopy(s['samples'][0]);r.pop('id');r['confirmations']['contact_fixed']=False
        with self.assertRaisesRegex(ValueError,'确认'):append_sample(s,r)

    def test_changed_hand_feedback(self):
        s=synthetic_session();s['samples'][0]['hand_feedback']={'feedback_q':[0,0,0,1,0,0]}
        r=copy.deepcopy(s['samples'][1]);r.pop('id');r['hand_feedback']={'feedback_q':[0,0,0,.9,0,0]}
        with self.assertRaisesRegex(ValueError,'手型'):append_sample(s,r)

    def test_model_mismatch(self):
        s=synthetic_session();r=copy.deepcopy(s['samples'][0]);r.pop('id');r['model_sha256']='wrong'
        with self.assertRaisesRegex(ValueError,'基线'):append_sample(s,r)

    def test_legacy_waist_metadata_does_not_force_group(self):
        s=synthetic_session();r=copy.deepcopy(s['samples'][0]);r.pop('id');r['phase']='waist_validation'
        append_sample(s,r)
        self.assertEqual(s['samples'][-1]['group'],'fit')
        self.assertEqual(s['samples'][-1]['phase'],'waist_validation')

    def test_unified_waist_and_failed_validation(self):
        s=synthetic_session()
        center=np.asarray(solve(s)['fit_old']['center_m'])
        for i,r in enumerate(s['samples']):
            r['phase']='unified';r['joints']['waist_yaw_joint']=i*.02
            if r['group']=='validation':
                g=np.asarray(r['G']);g[:3,3]=center-g[:3,:3]@s['s_model'];r['G']=g.tolist()
        result=solve(s)
        self.assertTrue(any('独立验证未改善' in w for w in result['warnings']))

    def test_metrics_units_and_rotation(self):
        a=metrics([[0,0,0],[.002,0,0]])
        self.assertAlmostEqual(a['rms_m']*1000,1.)
        g=np.eye(4);g[:3,:3]=Rotation.from_quat([0,0,np.sin(np.pi/4),np.cos(np.pi/4)]).as_matrix()
        np.testing.assert_allclose(point(g,[1,0,0]),[0,1,0],atol=1e-14)
        bad=g.copy();bad[0,0]=2
        with self.assertRaises(ValueError):transform(bad)

    def test_tracking_not_subtracted_from_fit(self):
        s=synthetic_session();first=solve(s)
        for r in s['samples']:r['tracking']={'norm_m':.04, 'actual_minus_reference_m':[.04,0,0]}
        np.testing.assert_array_equal(first['s_candidate_m'],solve(s)['s_candidate_m'])
        a=np.eye(4);b=np.eye(4);a[0,3]=.01
        self.assertAlmostEqual(tracking(a,b,[.2,0,0])['norm_m'],.01)

    def test_save_load_recompute_and_export(self):
        s=synthetic_session();result=solve(s)
        with tempfile.TemporaryDirectory() as directory:
            path=save_session(s,directory);original=path.read_bytes()
            loaded=load_session(path)
            self.assertIsNone(loaded['result'])
            np.testing.assert_array_equal(result['s_candidate_m'],solve(loaded)['s_candidate_m'])
            exported=export_session(loaded,directory)
            self.assertEqual(path.read_bytes(),original)
            self.assertTrue((exported/'REPORT.md').exists());self.assertTrue((exported/'samples.csv').exists())
            self.assertIn('applied: false',(exported/'candidate_tcp.yaml').read_text())

    def test_modified_sample_invalidates_candidate(self):
        s=synthetic_session();solve(s);invalidate(s,'exclude',s['samples'][0]['id'])
        self.assertIsNone(s['result'])

    def test_limits_reject_nan_and_broad_gates(self):
        for key,value in [('max_age_s',2.),('max_tip_drift_m',.02),('window_s',float('nan')),('max_condition',True)]:
            with self.assertRaises(ValueError):checked_limits({**DEFAULT_LIMITS,key:value})


class StabilityTests(unittest.TestCase):
    def rows(self):
        return [dict(received=i*.01,source='lowstate_actual',G=np.eye(4).tolist(),dq=[0.]*8,q_stability=[0.]*8) for i in range(110)]

    def test_stable_actual(self):
        self.assertGreater(stability(self.rows(),1.1,DEFAULT_LIMITS,[.25,0,0])['frames'],90)

    def test_stale(self):
        with self.assertRaisesRegex(ValueError,'过期'):stability(self.rows(),1.5,DEFAULT_LIMITS,[.25,0,0])

    def test_short_window(self):
        with self.assertRaisesRegex(ValueError,'不足'):stability(self.rows()[-8:],1.1,DEFAULT_LIMITS,[.25,0,0])

    def test_gap(self):
        rows=self.rows();del rows[40:60]
        with self.assertRaisesRegex(ValueError,'断流'):stability(rows,1.1,DEFAULT_LIMITS,[.25,0,0])

    def test_drift_out_and_back_not_hidden(self):
        rows=self.rows();rows[55]['G'][0][3]=.002
        with self.assertRaisesRegex(ValueError,'不稳定'):stability(rows,1.1,DEFAULT_LIMITS,[.25,0,0])

    def test_raw_velocity_spike_warns_when_positions_stable(self):
        rows=self.rows();rows[55]['dq'][0]=.1
        quality=stability(rows,1.1,DEFAULT_LIMITS,[.25,0,0])
        self.assertIn('原始dq',quality['warning'])
        self.assertEqual(quality['max_speed_rad_s'],.1)

    def test_measured_joint_motion_even_with_fixed_tip_rejects(self):
        rows=self.rows();rows[55]['q_stability'][0]=.01
        with self.assertRaisesRegex(ValueError,'关节角不稳定'):stability(rows,1.1,DEFAULT_LIMITS,[.25,0,0])

    def test_missing_measured_positions_rejected(self):
        rows=self.rows();del rows[55]['q_stability']
        with self.assertRaisesRegex(ValueError,'缺少实际关节角'):stability(rows,1.1,DEFAULT_LIMITS,[.25,0,0])

    def test_raw_speed_noise_small_position_jitter_accepted(self):
        rows=self.rows()
        for i,r in enumerate(rows):
            r['dq'][0]=.06*(-1)**i
            r['q_stability'][0]=.00005*(-1)**i
            r['G'][0][3]=.00003*(-1)**i
        self.assertLess(stability(rows,1.1,DEFAULT_LIMITS,[.25,0,0])['tip_drift_m'],.0001)

    def test_entire_capture_window_cannot_hide_early_excursion(self):
        rows=self.rows()+[dict(r,received=r['received']+1.1) for r in self.rows()]
        rows[25]['G'][0][3]=.01
        with self.assertRaisesRegex(ValueError,'不稳定'):stability(rows,2.2,DEFAULT_LIMITS,[.25,0,0],entire_window=True)

    def test_average_one_contact_preserves_raw_and_fk_consistency(self):
        rows=[]
        def fk(q):
            g=np.eye(4);g[0,3]=q['joint'];return g
        for i in range(101):
            q={'joint':.1+(-1)**i*.0001}
            rows.append(dict(q=q,G=fk(q).tolist(),dq_map={'joint':.06*(-1)**i},tau_est={'joint':0.},
                received=i*.01,tick=i,ros_receipt_ns=i*10000000))
        before=copy.deepcopy(rows)
        result,quality=average_observation(rows,fk,[.25,0,0])
        self.assertEqual(rows,before)
        self.assertEqual(quality['frames'],101);self.assertAlmostEqual(quality['duration_s'],1.)
        self.assertAlmostEqual(result['q']['joint'],np.mean([r['q']['joint'] for r in rows]))
        np.testing.assert_allclose(point(result['G'],[.25,0,0]),quality['mean_model_point_m'])

    def test_wrong_feedback(self):
        rows=self.rows();rows[55]['source']='target'
        with self.assertRaisesRegex(ValueError,'演示'):stability(rows,1.1,DEFAULT_LIMITS,[.25,0,0])

    def test_stable_but_tracking_error_allowed(self):
        rows=self.rows()
        for r in rows:r['reference_error_rad']=.05
        self.assertGreater(stability(rows,1.1,DEFAULT_LIMITS,[.25,0,0])['frames'],90)


class SafetyStructureTests(unittest.TestCase):
    def test_backend_output_allowlist(self):
        import ast
        tree=ast.parse(Path(__file__).with_name('tcp_a_node.py').read_text())
        calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
        publishers=[n for n in calls if n.func.attr=='create_publisher']
        self.assertEqual(len(publishers),2)
        self.assertFalse(any(n.func.attr in ('create_client','transition','rpc','hold_active','send_goal_async') for n in calls))
        for n in (n for n in ast.walk(tree) if isinstance(n,ast.Call)):
            if isinstance(n.func,ast.Name) and n.func.id=='HandControl':
                self.assertIn('enabled=False',ast.unparse(n))

    def test_launch_never_starts_control_or_tf(self):
        text=Path(__file__).resolve().parents[1].joinpath('src/g1_tcp_diagnostics/launch/experiment_a.launch.py').read_text()
        self.assertNotIn("package='g1_unitree_control'",text)
        self.assertNotIn('static_transform_publisher',text)
        self.assertNotIn('robot_state_publisher',text)


if __name__=='__main__':unittest.main()
