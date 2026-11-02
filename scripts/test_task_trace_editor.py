"""Offline editor contracts: no node, robot connection or motion client."""
import copy
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

import numpy as np

from task_trace_versions import EDIT_SLOTS, bank, append, select, delete, validate
from task_trace_edit import smooth, reverse_into, cut, curve, anchor, local_speed, trim_tail
import test_task_dring_demo as fixtures

fixture = fixtures.fixture


class VersionTests(unittest.TestCase):
    def test_ten_slots_full_refusal_delete_reuse_and_immutable_baseline(self):
        _, _, config = fixture()
        original = copy.deepcopy(config)
        for i in range(EDIT_SLOTS):
            trace = copy.deepcopy(config['traces']['left']); trace['label'] = str(i)
            config = append(config, 'left', trace)
        full = copy.deepcopy(config)
        with self.assertRaises(ValueError):append(config, 'left', trace)
        self.assertEqual(config, full)
        config = select(config, 'left', 2)
        config = delete(config, 'left', 2)
        self.assertEqual(config['traces']['left'], original['traces']['left'])
        config = append(config, 'left', trace)
        self.assertEqual(bank(config, 'left')['selected'], 2)
        self.assertEqual(bank(config, 'left')['baseline'], original['traces']['left'])
        with self.assertRaises(ValueError):delete(config, 'left', 0)

    def test_legacy_five_slots_expand_without_mutating_history(self):
        _,_,config=fixture()
        for _ in range(5):config=append(config,'left',config['traces']['left'])
        config['trace_versions']['left']['edits']=config['trace_versions']['left']['edits'][:5]
        before=copy.deepcopy(config)
        entry=bank(config,'left')
        self.assertEqual(len(entry['edits']),10)
        self.assertEqual(entry['edits'][:5],before['trace_versions']['left']['edits'])
        self.assertEqual(config,before)
        expanded=append(config,'left',config['traces']['left'])
        self.assertEqual(bank(expanded,'left')['selected'],6)
        validate(expanded,lambda trace:None)

    def test_inconsistent_selection_is_rejected(self):
        _, _, config = fixture()
        config = append(config, 'left', config['traces']['left'])
        config['traces']['left']['label'] = 'tampered'
        with self.assertRaises(ValueError):validate(config, lambda _:None)


class GeometryTests(unittest.TestCase):
    def test_trim_tail_discards_suffix_preserves_prefix_and_invalidates_old_timing(self):
        from task_dring_teach_data import arrays, fit_trace
        model,_,config=fixture();trace=config['traces']['left']
        trace['optimization']={'kind':'fast_timing','timed_trajectory':{'stale':True}}
        before=copy.deepcopy(trace)
        for seconds in (1.,1.003,3.6):
            edited=trim_tail(trace,seconds)
            q,t=arrays(edited,model,'left');end=round(4.-seconds,9)
            self.assertEqual(t[-1],end)
            old_t=np.asarray(trace['raw']['time_s']);old_q=np.asarray(trace['raw']['positions'])
            np.testing.assert_array_equal(t[:-1],old_t[old_t<end])
            np.testing.assert_array_equal(q[:-1],old_q[old_t<end])
            np.testing.assert_allclose(q[-1],[np.interp(end,old_t,v) for v in old_q.T])
            self.assertNotIn('timed_trajectory',edited['optimization'])
            self.assertFalse(edited['optimization']['bridge'])
            if end > .4:fit_trace(edited,model,'left')
        self.assertEqual(trace,before)
        for seconds in (0.,-1.,float('nan'),float('inf'),4.,3.601,1e-12):
            with self.assertRaises(ValueError):trim_tail(trace,seconds)

    def test_trim_tail_preserves_and_truncates_hand_recording_or_posture(self):
        from task_dring_teach_data import arrays
        model,_,config=fixture(True);trace=config['traces']['left'];before=copy.deepcopy(trace)
        edited=trim_tail(trace,1.003)
        arrays(edited,model,'left')
        self.assertEqual(edited['hand']['time_s'][-1],2.997)
        np.testing.assert_allclose(edited['hand']['q'][-1],
            [np.interp(2.997,trace['hand']['time_s'],v) for v in np.asarray(trace['hand']['q']).T])
        self.assertEqual(trace,before)
        trace['hand']='grip'
        self.assertEqual(trim_tail(trace,1.)['hand'],'grip')

    def test_local_speed_preserves_path_and_outer_timing(self):
        from task_dring_teach_data import arrays
        model,_,config=fixture();trace=config['traces']['left'];before=copy.deepcopy(trace)
        for factor in (.25,2.):
            edited=local_speed(trace,1.,3.,factor,.2)
            arrays(edited,model,'left')
            shift=(1/factor-1)*1.8
            self.assertAlmostEqual(edited['raw']['time_s'][-1],4.+shift)
            self.assertEqual(edited['optimization']['timing_policy'],'uniform')
            old_q=np.asarray(trace['raw']['positions']);new_q=np.asarray(edited['raw']['positions'])
            np.testing.assert_array_equal(new_q[[0,-1]],old_q[[0,-1]])
            for old_time in (.5,3.5):
                new_time=old_time+(shift if old_time>3 else 0.)
                np.testing.assert_allclose([np.interp(new_time,edited['raw']['time_s'],v) for v in new_q.T],
                                          [np.interp(old_time,trace['raw']['time_s'],v) for v in old_q.T])
        self.assertEqual(trace,before)
        for factor in (0.,1.,-1.,float('nan')):
            with self.assertRaises(ValueError):local_speed(trace,1.,3.,factor)

    def test_local_speed_preserves_synchronized_hand(self):
        from task_dring_teach_data import arrays
        model,_,config=fixture(True);trace=config['traces']['left']
        edited=local_speed(trace,1.,3.,.25)
        arrays(edited,model,'left')
        self.assertIsNotNone(edited['hand'])
        np.testing.assert_array_equal(edited['hand']['q'][0],trace['hand']['q'][0])
        np.testing.assert_array_equal(edited['hand']['q'][-1],trace['hand']['q'][-1])

    def test_keypoint_curve_interpolates_all_selected_points_without_stops(self):
        from task_dring_trim import sample
        from task_trace_curve import validate_curve
        from task_dring_teach_data import arrays,fit_trace,moving
        model,_,config=fixture();trace=config['traces']['left'];original=copy.deepcopy(trace)
        keys=[0.,1.,2.,3.,4.]
        edited=curve(trace,keys)
        arrays(edited,model,'left')
        info=edited['optimization'];spline=validate_curve(edited,model,moving('left'))
        np.testing.assert_allclose(sample(edited,info['key_times_s']),sample(trace,keys),atol=1e-12)
        tr,report=fit_trace(edited,model,'left')
        self.assertEqual(report['source'],'analytical_keypoint_curve')
        for u in np.asarray(info['key_times_s'][1:-1])/4.:
            self.assertGreater(np.linalg.norm(spline(u,1)),1e-4)
        for u in (0.,1.):
            np.testing.assert_allclose(spline(u,1),0.,atol=1e-10)
            np.testing.assert_allclose(spline(u,2),0.,atol=1e-10)
        self.assertEqual(trace,original)
        for a,b in zip(tr.points,tr.points[1:]):
            self.assertLess(a.time_from_start.sec+a.time_from_start.nanosec*1e-9,
                            b.time_from_start.sec+b.time_from_start.nanosec*1e-9)

    def test_partial_curve_keeps_outer_samples_and_all_endpoints(self):
        from task_dring_trim import sample
        from task_dring_teach_data import arrays
        model,_,config=fixture();trace=config['traces']['left']
        edited=curve(trace,[1.,1.5,2.5,3.],seconds=1.)
        q,t=arrays(edited,model,'left')
        np.testing.assert_array_equal(q[[0,-1]],np.asarray(trace['raw']['positions'])[[0,-1]])
        np.testing.assert_allclose(sample(edited,[.1,.5]),sample(trace,[.1,.5]),atol=1e-12)
        info=edited['optimization']
        np.testing.assert_allclose(sample(edited,info['key_times_s']),sample(trace,[1.,1.5,2.5,3.]),atol=1e-12)
        self.assertFalse(info['whole_trace'])
        from task_dring_trim import boundary_state
        from task_trace_curve import validate_curve
        from task_dring_teach_data import moving
        spline=validate_curve(edited,model,moving('left'))
        for u,stamp in ((0.,1.),(1.,3.)):
            _,v,a=boundary_state(trace,stamp)
            np.testing.assert_allclose(spline(u,1),v,atol=1e-10)
            np.testing.assert_allclose(spline(u,2),a,atol=1e-10)
            self.assertGreater(np.linalg.norm(v),1e-4)
        for keys in ([0.,1.],[0.,1.,1.],[0.,2.,5.],[-1.,1.,4.],[0.,float('nan'),4.]):
            with self.assertRaises(ValueError):curve(trace,keys)

    def test_curve_extrema_are_checked_and_tampered_metadata_is_rejected(self):
        from task_trace_curve import validate_curve
        from task_dring_teach_data import moving
        model,_,config=fixture();edited=curve(config['traces']['left'],[0.,2.,4.])
        model.limits=dict(model.limits);model.limits[moving('left')[0]]=(-2.,.1)
        with self.assertRaises(ValueError):validate_curve(edited,model,moving('left'))
        model.limits[moving('left')[0]]=(-2.,2.)
        edited['optimization']['spline']['controls'][0][0]+=.1
        with self.assertRaises(ValueError):validate_curve(edited,model,moving('left'))

    def test_anchoring_is_bounded_and_refuses_large_endpoint_change(self):
        _,_,config=fixture();trace=config['traces']['left'];target=np.array(trace['raw']['positions'][-1])+.003
        edited=anchor(trace,{-1:target})
        np.testing.assert_allclose(edited['raw']['positions'][-1],target,atol=1e-14)
        np.testing.assert_array_equal(edited['raw']['positions'][0],trace['raw']['positions'][0])
        with self.assertRaises(ValueError):anchor(trace,{-1:target+.1})

    def test_smoothing_preserves_endpoints_and_source_with_bounded_deviation(self):
        from task_dring_teach_data import arrays, fit_trace
        model, _, config = fixture()
        trace = config['traces']['left']; raw = trace['raw']
        q = np.asarray(raw['positions']); t = np.asarray(raw['time_s'])
        q[:, 0] += .001*np.sin(60*t)*np.sin(np.pi*t/t[-1])**2
        raw['positions'] = q.tolist(); original = copy.deepcopy(trace)
        edited = smooth(trace)
        rows, times = arrays(edited, model, 'left')
        np.testing.assert_array_equal(rows[[0, -1]], q[[0, -1]])
        self.assertLessEqual(np.max(np.abs(np.diff(rows, axis=0))), .0021)
        self.assertLessEqual(edited['optimization']['max_smoothing_rad'], .0030000001)
        self.assertEqual(trace, original)
        self.assertAlmostEqual(times[-1], t[-1])
        _, report = fit_trace(edited, model, 'left')
        self.assertLessEqual(report['fit_error_rad'], .001)
        for tolerance in (0., float('nan'), -.1):
            with self.assertRaises(ValueError):smooth(trace, tolerance=tolerance)

    def test_reverse_keeps_target_semantics_and_exact_endpoints(self):
        _, _, config = fixture()
        source = config['traces']['left']; target = dict(config['traces']['right'], stage='upper_to_lower', hand='grip')
        edited = reverse_into(source, target)
        self.assertEqual(edited['stage'], target['stage'])
        self.assertEqual(edited['hand'], 'grip')
        self.assertEqual(edited['raw']['positions'], source['raw']['positions'][::-1])
        self.assertTrue(np.all(np.diff(edited['raw']['time_s']) > 0))
        trimmed = cut(target, 1., 2., .6)
        self.assertEqual(trimmed['hand'], 'grip')


class EditorControlTests(unittest.TestCase):
    def setUp(self):
        fixtures.DemoPersistenceTests.setUp(self)
        self.node.launch_work = lambda work:work()
        self.control.persist(self.config)

    def test_full_controller_refuses_before_computing_and_keeps_store(self):
        c = self.control
        for _ in range(EDIT_SLOTS):
            c.edit_version('left','select',index=0)
            c.edit_version('left', 'cut', start=1., end=2., bridge=.6)
        previous = c.store.read_bytes()
        with patch('task_trace_edit.smooth', side_effect=AssertionError('must not compute')):
            with self.assertRaises(ValueError):c.edit_version('left', 'smooth')
        self.assertEqual(c.store.read_bytes(), previous)
        c.edit_version('left', 'delete', index=3)
        c.edit_version('left', 'select', index=0)
        c.edit_version('left', 'smooth')
        self.assertEqual(bank(c.config, 'left')['selected'], 3)

    def test_stale_worker_cannot_commit_after_configuration_change(self):
        c = self.control; jobs = []
        self.node.launch_work = jobs.append
        c.edit_version('left', 'smooth')
        c.revision += 1
        before = c.store.read_bytes()
        with self.assertRaises(RuntimeError):jobs[0]()
        self.assertEqual(c.store.read_bytes(), before)

    def test_commit_from_real_worker_does_not_reject_its_own_busy_flag(self):
        errors = []
        def launch(work):
            def run():
                try:work()
                except Exception as error:errors.append(error)
            self.node.worker = threading.Thread(target=run)
            self.node.worker.start()
        self.node.launch_work = launch
        self.control.edit_version('left', 'smooth')
        self.node.worker.join(timeout=10)
        self.assertFalse(self.node.worker.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(bank(self.control.config, 'left')['selected'], 1)


class HistoryTests(unittest.TestCase):
    def test_load_historical_plan_reauthorizes_only_at_original_start_without_computing(self):
        from task_dring_teach_data import build
        from task_planning_config import PlanningParameters
        from task_screen_teach import fingerprint
        from task_saved_test import range_fingerprint
        from task_recorded_history import load
        from task_session import Session
        from rosidl_runtime_py.convert import message_to_ordereddict
        setup=fixtures.DemoPersistenceTests();setup.setUp();self.addCleanup(setup.doCleanups)
        c,n=setup.control,setup.node;c.persist(setup.config)
        model,q,_=fixture();clock=[10.];n.session=Session(lambda:clock[0])
        for _ in range(8):clock[0]+=.1;n.session.update(q)
        n.session.capture_home();n.hand_control=False
        n.gateway=dict(task_stop_epoch='7',task_stop_active='false')
        n.launch_work=lambda work:work()
        plan=build(model,c.config,q,'left',PlanningParameters(),.7,.7,1.)
        original=range_fingerprint(plan['segments'])
        plan['demo'].update(workflow='dring_demo',config_sha256=fingerprint(c.config),trajectory_sha256=original)
        directory=setup.root/'history';directory.mkdir()
        path=directory/'plan.json'
        path.write_text(json.dumps(dict(plan,segments=[dict(s,trajectory=message_to_ordereddict(s['trajectory'])) for s in plan['segments']])))
        (directory/'config.json').write_text(json.dumps(c.config))
        def stationary(_n,ticket,**kwargs):
            self.assertEqual(kwargs['expected'],q)
            kwargs['guard']()
            return q
        with patch.object(c,'gateway_guard'),patch('real_task_app.TaskNode.wait_stationary',side_effect=stationary),\
                patch('task_dring_teach_data.fit_trace',side_effect=AssertionError('no fitting')):
            load(c,path)
            self.assertEqual(n.session.phase,'PLAN_READY')
            self.assertEqual(range_fingerprint(n.session.plan['segments']),original)
            c.execution_guard(n.session.plan,preflight=True)
            n.gateway['task_stop_active']='true'
            with self.assertRaises(RuntimeError):c.execution_guard(n.session.plan)

    def test_read_history_preserves_exact_path_and_timing_without_fit_or_limits(self):
        from task_dring_teach_data import build
        from task_planning_config import PlanningParameters
        from task_screen_teach import fingerprint
        from task_saved_test import range_fingerprint
        from task_recorded_history import read_plan
        from rosidl_runtime_py.convert import message_to_ordereddict
        model, q, config = fixture()
        config['identity'] = {'robot':'test'}
        plan = build(model, config, q, 'left', PlanningParameters(), .7, .7, 1.)
        plan['demo'].update(workflow='dring_demo', config_sha256=fingerprint(config),
                            trajectory_sha256=range_fingerprint(plan['segments']))
        for s in plan['segments']:s['trajectory'] = message_to_ordereddict(s['trajectory'])
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'plan.json'; path.write_text(json.dumps(plan))
            (Path(d)/'config.json').write_text(json.dumps(config))
            with patch('task_dring_teach_data.fit_trace', side_effect=AssertionError('no fit')), \
                    patch('task_trajectory_speed.speed_capacity', side_effect=AssertionError('no limit solve')):
                loaded, _, _ = read_plan(path, 'dring_demo', config['identity'])
            for a, b in zip(loaded['segments'], plan['segments']):
                self.assertEqual(message_to_ordereddict(a['trajectory']), b['trajectory'])
            with self.assertRaises(ValueError):read_plan(path, 'landing_gear', config['identity'])
            plan['segments'][0]['trajectory']['points'][0]['positions'][0] += .1
            path.write_text(json.dumps(plan))
            with self.assertRaises(ValueError):read_plan(path, 'dring_demo', config['identity'])


class CompareTests(unittest.TestCase):
    def test_only_differences_hides_matching_paths_and_roots(self):
        from task_trace_markers import comparison_markers
        from task_trace_compare import differing_pick
        pose=np.repeat(np.eye(4)[None],10,axis=0);pose[:,0,3]=np.arange(10)*.01
        end=pose.copy();end[:,1,3]-=.1
        result=dict(first={'left_index_tcp':pose},second={'left_index_tcp':pose},
                    first_endpoints={'left_index_tcp':end},second_endpoints={'left_index_tcp':end},
                    differences={'left_index_tcp':np.zeros(10,dtype=bool)})
        self.assertTrue(all(m.action==m.DELETE for m in comparison_markers(result).markers))
        self.assertTrue(any(m.points for m in comparison_markers(result,only_differences=False).markers))
        _,_,config=fixture();trace=config['traces']['left']
        mask=np.zeros(10,dtype=bool);mask[:3]=True
        self.assertTrue(differing_pick(trace,0.,mask))
        self.assertFalse(differing_pick(trace,0.,mask,reverse=True))

    def test_solid_arrow_has_volume_and_correct_tip(self):
        from task_trace_markers import solid_arrow_marker
        marker=solid_arrow_marker(np.array([[0.,0.,0.]]),np.array([[-1.,0.,0.]]),.04,12,(1.,1.,0.,1.))
        points=np.array([[p.x,p.y,p.z] for p in marker.points])
        self.assertEqual(marker.type,marker.TRIANGLE_LIST)
        self.assertEqual(len(points)%3,0)
        self.assertAlmostEqual(np.max(points[:,0]),.04)
        self.assertTrue(np.all(np.ptp(points,axis=0)>.005))
        thick=solid_arrow_marker(np.array([[0.,0.,0.]]),np.array([[-1.,0.,0.]]),.04,12,(1.,1.,0.,1.),diameter=.0072)
        thick_points=np.array([[p.x,p.y,p.z] for p in thick.points])
        np.testing.assert_allclose(np.ptp(thick_points,axis=0),np.ptp(points,axis=0)*[1.,2.,2.])
    def test_path_alignment_ignores_timing_and_reverse_is_explicit(self):
        from task_trace_compare import aligned_rows
        _,_,config=fixture();a=config['traces']['left'];b=copy.deepcopy(a)
        b['raw']['time_s']=(np.asarray(b['raw']['time_s'])**2).tolist()
        np.testing.assert_allclose(aligned_rows(a),aligned_rows(b))
        self.assertGreater(np.max(np.abs(aligned_rows(a,mode='time')-aligned_rows(b,mode='time'))),.01)
        reversed_trace=reverse_into(a,a)
        np.testing.assert_allclose(aligned_rows(a),aligned_rows(reversed_trace,reverse=True),atol=1e-12)

    def test_comparison_reports_only_real_geometry_difference(self):
        from types import SimpleNamespace as S
        from task_trace_compare import compare
        from task_trace_markers import comparison_markers
        from task_session import ARMS
        def transforms(links,context,names,rows):
            poses=np.repeat(np.eye(4)[None,:,:],len(rows),axis=0)
            poses[:,0,3]=rows[:,0]
            return {link:poses.copy() for link in links}
        model=S(batch_transforms=transforms)
        _,_,config=fixture();a=config['traces']['left'];b=copy.deepcopy(a)
        context=dict.fromkeys(ARMS['right'],0.)
        same=compare(model,a,b,'left',context)
        self.assertEqual(same['report']['max_joint_delta_rad'],0.)
        b['raw']['positions']=(np.asarray(b['raw']['positions'])+.02).tolist()
        changed=compare(model,a,b,'left',context)
        self.assertAlmostEqual(changed['report']['hands']['left']['max_position_mm'],20.)
        self.assertTrue(all(np.all(mask) for mask in changed['differences'].values()))
        markers=comparison_markers(changed)
        self.assertEqual(len(markers.markers),len({(m.ns,m.id) for m in markers.markers}))
        drawn=[m for m in markers.markers if m.action==m.ADD]
        self.assertEqual(len(drawn),6)
        self.assertTrue(all(m.points for m in drawn))
        missing=compare(model,a,b,'left',{})
        self.assertIsNone(missing['report']['hands']['right'])
        self.assertNotIn('right_index_tcp',missing['first'])


class PanelTests(unittest.TestCase):
    def setUp(self):
        fixtures.DemoPersistenceTests.setUp(self)
        self.node.launch_work=lambda work:work()
        self.control.persist(self.config)

    def test_tail_control_shared_layout_settings_and_saved_version(self):
        from types import SimpleNamespace as S
        from PyQt5 import QtWidgets
        from task_trace_editor_panel import TraceEditorPanel
        from task_editor_settings import values,restore_values
        self.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        window=S(node=self.node,invoke=lambda action:action(),clear=QtWidgets.QCheckBox())
        original=copy.deepcopy(self.control.config['traces']['left'])
        for timeline in (True,False):
            panel=TraceEditorPanel(window,self.control,lambda:'left',timeline=timeline)
            try:
                panel.external_pause=Mock()
                restore_values(panel,{'tail_seconds':.5})
                self.assertEqual(values(panel)['tail_seconds'],.5)
                panel.trim_tail()
                panel.external_pause.assert_called_once()
                edited=self.control.config['traces']['left']
                self.assertEqual(edited['edit_origin']['operation'],'trim_tail')
                self.assertTrue(edited['edit_origin']['created_at'])
                self.assertEqual(bank(self.control.config,'left')['baseline'],original)
            finally:panel.close();self.app.processEvents()
        self.assertEqual(edited['raw']['time_s'][-1],3.)

    def test_add_remove_keys_curve_and_independent_comparison_selection(self):
        from types import SimpleNamespace as S
        from PyQt5 import QtWidgets,QtCore
        from task_trace_editor_panel import TraceEditorPanel
        self.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        window=S(node=self.node,invoke=lambda action:action(),clear=QtWidgets.QCheckBox())
        panel=TraceEditorPanel(window,self.control,lambda:'left')
        panel.refresh();panel.show();self.app.processEvents()
        try:
            for time_ms in (0,1000,2000,3000,4000):panel.seek(time_ms);panel.add_key()
            self.assertEqual(panel.key_times,[0.,1.,2.,3.,4.])
            with self.assertRaises(ValueError):panel.add_key()
            panel.keys.setCurrentRow(2);panel.remove_key()
            self.assertEqual(panel.key_times,[0.,1.,3.,4.])
            panel.interpolate_keys();panel.refresh()
            self.assertEqual(bank(self.control.config,'left')['selected'],1)
            self.assertEqual(panel.key_times,self.control.config['traces']['left']['optimization']['key_times_s'])
            self.control.edit_version('left','select',index=0)
            self.control.edit_version('left','curve',key_times=[0.,2.,4.]);panel.refresh()
            revision=self.control.revision;trace=self.control.config['traces']['left']
            for index in (0,1):panel.versions.item(index,3).setCheckState(QtCore.Qt.Checked)
            self.assertEqual(panel.compared,[0,1])
            panel.versions.item(2,3).setCheckState(QtCore.Qt.Checked)
            self.assertEqual(panel.versions.item(2,3).checkState(),QtCore.Qt.Unchecked)
            self.assertEqual(self.control.revision,revision)
            self.assertIs(self.control.config['traces']['left'],trace)
            self.assertEqual(len(panel.selected_versions()),2)
        finally:panel.close();self.app.processEvents()


if __name__ == '__main__':unittest.main()
