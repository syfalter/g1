"""Keep partial and completed batch statistics tied to the sampled inputs."""
import json
from pathlib import Path
import tempfile
import unittest

from report_workbench_robustness import bucket_counts, load_results


class ReportTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        (self.root / 'results').mkdir()
        self.cases = [dict(id=0, dy_m=-.06), dict(id=1, dy_m=0.)]

    def record(self, case, arm='left', outcome='PASS', name=None):
        path = self.root / 'results' / (name or f'{case["id"]:03d}_{arm}.json')
        path.write_text(json.dumps(dict(case=case, arm=arm, outcome=outcome)))

    def test_partial_results_do_not_count_pending_samples(self):
        self.record(self.cases[0])
        records = load_results(self.root, self.cases)
        self.assertEqual(len(records), 1)
        buckets = bucket_counts(self.cases, records, 'dy_m', ((-12, -6), (-6, 0), (0, 6)))
        self.assertEqual([b['stances'] for b in buckets], [0, 1, 1])
        self.assertEqual(buckets[1]['left']['counts'], {'PASS': 1})
        self.assertEqual(buckets[1]['right']['total'], 0)
        self.assertEqual(buckets[2]['left']['total'], 0)

    def test_record_must_match_sample_and_filename(self):
        for case, name in ((dict(id=0, dy_m=.01), '000_left.json'),
                           (self.cases[0], '001_left.json')):
            with self.subTest(case=case, name=name):
                self.record(case, name=name)
                with self.assertRaises(ValueError):load_results(self.root, self.cases)
                (self.root / 'results' / name).unlink()

    def test_unknown_outcome_rejected(self):
        self.record(self.cases[0], outcome='DONE')
        with self.assertRaises(ValueError):load_results(self.root, self.cases)

    def test_timeout_is_not_no_path(self):
        self.record(self.cases[0], outcome='TIME_BUDGET_EXCEEDED')
        self.record(self.cases[0], arm='right', outcome='NO_COMPLETE_PLAN')
        records = load_results(self.root, self.cases)
        bucket = bucket_counts(self.cases, records, 'dy_m', ((-6, 0),))[0]
        self.assertEqual(bucket['left']['counts'], {'TIME_BUDGET_EXCEEDED': 1})
        self.assertEqual(bucket['right']['counts'], {'NO_COMPLETE_PLAN': 1})


if __name__ == '__main__':unittest.main()
