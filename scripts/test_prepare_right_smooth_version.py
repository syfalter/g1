import copy
import unittest

from prepare_right_smooth_version import oldest_edit, install_candidate
from task_screen_teach import fingerprint


class ReplacementTests(unittest.TestCase):
    def config(self):
        edits=[dict(value=i,edit_origin=dict(created_at=f'2026-09-{20+i:02d}T10:00:00+08:00'))
               for i in range(3)]+[None]*7
        return dict(traces={'right_4':copy.deepcopy(edits[2])},trace_versions={
            'right_4':dict(baseline={'value':'baseline'},edits=edits,selected=3)})

    def test_replaces_only_oldest_and_retains_active_selection(self):
        config=self.config();before=copy.deepcopy(config)
        artifact=dict(slot='right_4',replace_version=1,source_version=2,
                      bundle_sha256=fingerprint(config),trace={'value':'candidate'})
        result=install_candidate(config,artifact)
        self.assertEqual(config,before)
        self.assertEqual(result['traces'],config['traces'])
        entry=result['trace_versions']['right_4']
        self.assertEqual(entry['baseline'],config['trace_versions']['right_4']['baseline'])
        self.assertEqual(entry['edits'][0],artifact['trace'])
        self.assertEqual(entry['edits'][1:],config['trace_versions']['right_4']['edits'][1:])
        self.assertEqual(entry['selected'],3)

    def test_rejects_changed_bundle_and_protected_version(self):
        config=self.config()
        artifact=dict(slot='right_4',replace_version=1,source_version=2,
                      bundle_sha256='stale',trace={})
        with self.assertRaises(ValueError):install_candidate(config,artifact)
        artifact.update(bundle_sha256=fingerprint(config),source_version=1)
        with self.assertRaises(ValueError):install_candidate(config,artifact)

    def test_orders_by_edit_timestamp_not_slot_number(self):
        entry=self.config()['trace_versions']['right_4']
        entry['edits'][1]['edit_origin']['created_at']='2026-09-01T00:00:00+08:00'
        self.assertEqual(oldest_edit(entry),2)


if __name__=='__main__':unittest.main()
