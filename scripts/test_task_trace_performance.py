"""Selection durability and display reuse, without hardware or production writes."""
import copy
import json
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch

import numpy as np

from task_trace_versions import bank,select_shared
from task_trace_selection import journal
from test_task_dring_demo import fixture
import test_task_dring_demo as fixtures


class SelectionTests(unittest.TestCase):
    def setUp(self):
        fixtures.DemoPersistenceTests.setUp(self)
        self.node.launch_work=lambda work:work()
        self.control.persist(self.config)
        self.control.edit_version('left','cut',start=1.,end=2.,bridge=.6)

    def test_selection_only_writes_small_journal_and_restores(self):
        from task_dring_demo_control import DringDemoControl
        c=self.control;before=c.store.read_bytes();source=c.config;baseline=bank(source,'left')['baseline']
        with patch('task_trace_versions.copy.deepcopy',side_effect=AssertionError('must not copy samples')):
            c.edit_version('left','select',index=0)
        self.assertIs(c.config['traces']['left'],baseline)
        self.assertEqual(bank(source,'left')['selected'],1)
        self.assertEqual(c.store.read_bytes(),before)
        self.assertLess(journal(c.store).stat().st_size,4096)
        restored=DringDemoControl(self.node,c.store)
        self.assertEqual(bank(restored.config,'left')['selected'],0)
        self.assertEqual(restored.config,c.config)
        self.node.session.invalidate.assert_called()

    def test_failed_write_keeps_selection_and_store_unchanged(self):
        c=self.control;source=c.config;revision=c.revision;before=c.store.read_bytes()
        with patch('task_trace_selection.write_json',side_effect=OSError('disk full')):
            with self.assertRaises(OSError):c.edit_version('left','select',index=0)
        self.assertIs(c.config,source);self.assertEqual(c.revision,revision)
        self.assertEqual(c.store.read_bytes(),before)

    def test_full_edit_supersedes_old_journal(self):
        from task_dring_demo_control import DringDemoControl
        c=self.control;c.edit_version('left','select',index=0)
        c.edit_version('left','cut',start=1.,end=2.,bridge=.6)
        restored=DringDemoControl(self.node,c.store)
        self.assertEqual(bank(restored.config,'left')['selected'],2)
        self.assertEqual(restored.config,c.config)

    def test_external_file_replacement_refuses_selection(self):
        from task_screen_teach import write_json
        c=self.control;source=c.config
        write_json(c.store,source)
        with self.assertRaisesRegex(ValueError,'外部修改'):c.edit_version('left','select',index=0)
        self.assertIs(c.config,source)

    def test_side_switch_and_export_preserve_effective_selection(self):
        c=self.control;c.edit_version('left','select',index=0)
        c.save(self.root/'export.json')
        self.assertEqual(bank(json.loads((self.root/'export.json').read_text()),'left')['selected'],0)
        c.select_side('right');c.select_side('left')
        self.assertEqual(bank(c.config,'left')['selected'],0)

    def test_selected_metadata_is_copied_but_samples_are_shared(self):
        c=self.control;source=c.config
        result=select_shared(source,'left',0)
        self.assertIsNot(result['traces'],source['traces'])
        self.assertIsNot(bank(result,'left')['edits'],bank(source,'left')['edits'])
        result['trace_versions']['left']['selected']=1
        self.assertEqual(bank(source,'left')['selected'],1)


class DisplayTests(unittest.TestCase):
    def test_geometry_shared_between_arrows_and_picker_and_context_invalidates(self):
        from task_trace_display_cache import TraceDisplayCache
        from task_trace_markers import trace_markers,pick_points
        _,context,config=fixture();trace=config['traces']['left']
        def transforms(links,context,names,rows):
            result={}
            for link in links:
                pose=np.repeat(np.eye(4)[None],len(rows),axis=0)
                pose[:,0,3]=rows[:,0]
                if link.endswith('index_tcp'):pose[:,2,3]=.1
                result[link]=pose
            return result
        model=S(batch_transforms=Mock(side_effect=transforms));cache=TraceDisplayCache()
        first=cache.geometry(model,trace,'left',context,config,'left')
        trace_markers(model,trace,'left',context,0.,0.,config,'left',geometry=first)
        pick_points(model,trace,'left',context,config,'left',geometry=first)
        again=cache.geometry(model,trace,'left',context,config,'left')
        self.assertIs(first,again);self.assertEqual(model.batch_transforms.call_count,1)
        context['right_elbow_joint']+=.1
        self.assertIsNot(first,cache.geometry(model,trace,'left',context,config,'left'))
        self.assertEqual(model.batch_transforms.call_count,2)
        cache=TraceDisplayCache(maximum_bytes=1)
        cache.geometry(model,trace,'left',context,config,'left')
        self.assertEqual(cache.bytes,0)

    def test_picker_reuses_unchanged_geometry_and_updates_callbacks(self):
        from task_trace_picker import TracePicker
        from visualization_msgs.msg import InteractiveMarkerFeedback as Feedback
        with patch('task_trace_picker.InteractiveMarkerServer') as server:
            picker=TracePicker(Mock());owner=object();first=Mock();second=Mock()
            points=[dict(position=[0.,0.,0.],time=1.,trace={})]
            picker.show(owner,points,first)
            self.assertEqual(server.return_value.insert.call_count,1)
            old=server.return_value.insert.call_args.kwargs['feedback_callback']
            server.return_value.clear.reset_mock()
            picker.show(owner,points,second)
            self.assertEqual(server.return_value.insert.call_count,1)
            server.return_value.clear.assert_not_called()
            old(Feedback(event_type=Feedback.BUTTON_CLICK));first.assert_not_called()
            callback=server.return_value.setCallback.call_args.args[1]
            callback(Feedback(event_type=Feedback.BUTTON_CLICK));second.assert_called_once()
            picker.show(owner,[],second)
            server.return_value.erase.assert_called_once()

    def test_failed_drag_can_reset_unchanged_target(self):
        from task_trace_picker import TracePicker
        from visualization_msgs.msg import InteractiveMarkerFeedback as Feedback
        with patch('task_trace_picker.InteractiveMarkerServer') as server:
            picker=TracePicker(Mock());owner=object();picker.show(owner,[],Mock())
            picker.move_handle(owner,[0.,0.,0.],Mock())
            callback=server.return_value.insert.call_args.kwargs['feedback_callback']
            picker.move_handle(owner,[0.,0.,0.],Mock())
            self.assertEqual(server.return_value.insert.call_count,1)
            callback=server.return_value.setCallback.call_args.args[1]
            callback(Feedback(event_type=Feedback.MOUSE_UP))
            picker.move_handle(owner,[0.,0.,0.],Mock())
            self.assertEqual(server.return_value.insert.call_count,2)


if __name__=='__main__':unittest.main()
