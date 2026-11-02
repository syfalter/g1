"""Synthetic calibration and offscreen GUI tests; no DDS or robot commands."""
import csv
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import numpy as np
from PyQt5 import QtCore, QtWidgets

from center_calibration_data import (atomic_json, export_csv, fit_samples, load_samples,
                                     observation, read_source, save_samples, scan_sources)
from center_calibration_panel import Window


def fixture(root, number=1, height=50., fk=(-1.584, -.602, -1.567)):
    directory = Path(root) / 'session' / f'plan_{number:04d}'
    center = np.array([.35 + number * .01, .12, -.05])
    target = center + [0, 0, height / 1000]
    evidence = dict(frame='pelvis', operation_offsets_applied=False,
                    center_pelvis_m=center.tolist(), target_pelvis_m=target.tolist(), height_mm=height)
    atomic_json(directory / 'plan.json', dict(task_schema='index_center_check_v1', arm='left', center_check=evidence))
    result = dict(target_pelvis_m=target.tolist(), fk_error_mm=list(fk), settled=True,
                  feedback_age_s=.004, measured_joints={}, pelvis_from_wrist=np.eye(4).tolist())
    path = directory / 'center_check_result.json'
    atomic_json(path, result)
    return read_source(path)


class DataTests(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.source = fixture(self.root)

    def tearDown(self):
        self.temp.cleanup()

    def sample(self, source=None, role='fit'):
        return observation(source or self.source, 35, 0, 35, 'tip', role)

    def test_reported_trial_signs_and_fk_subtraction(self):
        sample = self.sample()
        np.testing.assert_allclose(sample['physical_error_mm'], [35, 0, -15])
        np.testing.assert_allclose(sample['geometric_error_mm'], [36.584, .602, -13.433])
        fit = fit_samples([sample])
        np.testing.assert_allclose(fit['suggested_target_offset_mm'], [-36.584, -.602, 13.433])
        self.assertFalse(fit['applied'])
        self.assertFalse(fit['extrinsics_identified'])

    def test_nondefault_target_height(self):
        sample = self.sample(fixture(self.root, 2, height=80))
        self.assertEqual(sample['physical_error_mm'][2], -45)

    def test_nan_inf_and_huge_inputs(self):
        for value in (float('nan'), float('inf'), -float('inf'), 1001):
            with self.assertRaises(ValueError):
                observation(self.source, value, 0, 35, 'tip')

    def test_empty_datum_and_invalid_role(self):
        with self.assertRaises(ValueError): observation(self.source, 0, 0, 50, '')
        with self.assertRaises(ValueError): self.sample(role='bad')

    def test_scan_skips_incomplete_and_broken_records(self):
        source = fixture(self.root, 2)
        result = source['result_snapshot']; result['settled'] = False
        atomic_json(source['source_path'], result)
        bad = fixture(self.root, 3)
        atomic_json(bad['source_path'], {})
        sources, errors = scan_sources(self.root)
        self.assertEqual(len(sources), 1)
        self.assertEqual(len(errors), 2)

    def test_mismatched_target_rejected(self):
        result = self.source['result_snapshot']
        result['target_pelvis_m'][0] += .01
        atomic_json(self.source['source_path'], result)
        with self.assertRaises(ValueError): read_source(self.source['source_path'])

    def test_wrong_schema_or_compensated_source_rejected(self):
        path = Path(self.source['source_path']).with_name('plan.json')
        plan = json.loads(path.read_text())
        plan['task_schema'] = 'ring'
        atomic_json(path, plan)
        with self.assertRaises(ValueError): read_source(self.source['source_path'])
        plan['task_schema'] = 'index_center_check_v1'
        plan['center_check']['operation_offsets_applied'] = True
        atomic_json(path, plan)
        with self.assertRaises(ValueError): read_source(self.source['source_path'])

    def test_holdout_is_not_used_to_fit(self):
        first = observation(self.source, 10, 0, 50, 'tip')
        second = observation(fixture(self.root, 2), 100, 0, 50, 'tip', 'validation')
        fit = fit_samples([first, second])
        self.assertEqual(fit['fit_count'], 1)
        np.testing.assert_allclose(fit['mean_physical_error_mm'], [10, 0, 0])
        self.assertAlmostEqual(fit['validation_after']['rmse_3d_mm'], 90.)

    def test_excluded_and_mixed_groups(self):
        other = self.sample(fixture(self.root, 2)); other['datum'] = 'pad'
        with self.assertRaises(ValueError): fit_samples([self.sample(), other])
        other['datum'] = 'tip'; other['source']['arm'] = 'right'
        with self.assertRaises(ValueError): fit_samples([self.sample(), other])
        other['role'] = 'excluded'
        self.assertEqual(fit_samples([self.sample(), other])['fit_count'], 1)

    def test_no_training_samples(self):
        with self.assertRaises(ValueError): fit_samples([])
        with self.assertRaises(ValueError): fit_samples([self.sample(role='validation')])

    def test_synthetic_known_offset_recovers_and_validates(self):
        bias = np.array([22., -8., -12.])
        samples = []
        for i in range(1, 8):
            fk = np.array([i * .3, -i * .2, i * .1])
            source = fixture(self.root, i, fk=fk)
            physical = bias + fk
            samples.append(observation(source, physical[0], physical[1], physical[2] + 50,
                                       'tip', 'fit' if i < 6 else 'validation'))
        fit = fit_samples(samples)
        np.testing.assert_allclose(fit['suggested_target_offset_mm'], -bias)
        self.assertLess(fit['validation_after']['rmse_3d_mm'], 1e-10)

    def test_position_dependent_error_shows_holdout_residual(self):
        samples = [observation(fixture(self.root, i), i * 10, 0, 50, 'tip',
                               'validation' if i == 5 else 'fit') for i in range(1, 6)]
        self.assertGreater(fit_samples(samples)['validation_after']['rmse_3d_mm'], 20)

    def test_persistence_and_snapshot_without_original(self):
        path = self.root / 'samples.json'
        sample = self.sample()
        save_samples(path, [sample])
        Path(self.source['source_path']).unlink()
        restored = load_samples(path)
        self.assertEqual(restored, [sample])
        fit_samples(restored)

    def test_duplicate_and_corrupt_files_rejected(self):
        path = self.root / 'samples.json'
        sample = self.sample()
        with self.assertRaises(ValueError): save_samples(path, [sample, sample])
        atomic_json(path, {'schema': 'center_observations_v2', 'samples': [sample, sample]})
        with self.assertRaises(ValueError): load_samples(path)
        atomic_json(path, {'schema': 'bad'})
        with self.assertRaises(ValueError): load_samples(path)

    def test_same_plan_multiple_samples_are_preserved(self):
        path = self.root / 'samples.json'
        first, second = self.sample(), self.sample()
        save_samples(path, [first, second])
        restored = load_samples(path)
        self.assertEqual(len(restored), 2)
        self.assertNotEqual(restored[0]['sample_id'], restored[1]['sample_id'])
        fit = fit_samples(restored)
        self.assertEqual(fit['fit_count'], 2)
        self.assertEqual(fit['fit_plan_count'], 1)
        self.assertEqual(len({r['sample_id'] for r in fit['residuals']}), 2)

    def test_legacy_migration_keeps_values_and_stable_ids(self):
        path = self.root / 'samples.json'
        sample = self.sample(); sample.pop('sample_id')
        atomic_json(path, {'schema': 'center_observations_v1', 'samples': [sample]})
        first = load_samples(path); second = load_samples(path)
        self.assertEqual(first, second)
        self.assertEqual(first[0]['measured_forward_left_height_mm'], sample['measured_forward_left_height_mm'])
        save_samples(path, first)
        self.assertEqual(load_samples(path), first)
        backups = list((self.root / 'samples_history').glob('*.json'))
        self.assertEqual(len(backups), 1)
        self.assertEqual(json.loads(backups[0].read_text())['schema'], 'center_observations_v1')

    def test_updates_and_deletes_keep_prior_versions(self):
        path = self.root / 'samples.json'
        first, second = self.sample(), self.sample()
        save_samples(path, [first])
        save_samples(path, [first, second])
        save_samples(path, [])
        backups = sorted((self.root / 'samples_history').glob('*.json'))
        self.assertEqual([len(load_samples(p)) for p in backups], [1, 2])

    def test_history_failure_does_not_overwrite_current(self):
        path = self.root / 'samples.json'
        first = self.sample(); save_samples(path, [first])
        with patch('center_calibration_data.atomic_json', side_effect=OSError('disk full')):
            with self.assertRaises(OSError): save_samples(path, [])
        self.assertEqual(load_samples(path), [first])

    def test_failed_atomic_write_preserves_previous_data(self):
        path = self.root / 'samples.json'
        save_samples(path, [self.sample()])
        before = path.read_bytes()
        with self.assertRaises(ValueError): atomic_json(path, {'x': float('nan')})
        self.assertEqual(path.read_bytes(), before)

    def test_csv(self):
        path = self.root / 'observations.csv'
        export_csv(path, [self.sample()])
        with path.open(encoding='utf-8-sig', newline='') as stream:
            rows = list(csv.reader(stream))
        self.assertEqual(len(rows), 2)
        self.assertEqual(len(rows[0]), len(rows[1]))
        self.assertEqual(rows[1][6], '35.0')


class PanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.temp = TemporaryDirectory()
        self.root = Path(self.temp.name)
        fixture(self.root / 'sessions')
        self.window = Window(self.root / 'sessions', self.root / 'measurements.json')

    def tearDown(self):
        self.window.close()
        self.temp.cleanup()

    def enter(self):
        for field, text in zip((self.window.forward, self.window.left, self.window.height), ('35', '0', '35')):
            field.setText(text)

    def test_entry_update_calculate_reopen(self):
        self.enter(); self.window.save(); self.window.calculate()
        report = json.loads((self.root / 'measurements_fit.json').read_text())
        self.assertEqual(report['fit_count'], 1)
        self.window.table.selectRow(0)
        self.window.forward.setText('30'); self.window.update_selected()
        self.assertEqual(len(self.window.samples), 1)
        reopened = Window(self.root / 'sessions', self.root / 'measurements.json')
        self.assertEqual(reopened.samples[0]['measured_forward_left_height_mm'][0], 30)
        self.assertEqual(reopened.forward.text(), '')
        reopened.close()

    def test_refresh_preserves_unsaved_input(self):
        self.enter()
        fixture(self.root / 'sessions', 2)
        self.window.refresh_sources()
        self.assertEqual(self.window.forward.text(), '35')
        self.assertEqual(self.window.source_combo.currentData(), 'session/plan_0001')

    def test_switch_clears_unmeasured_fields(self):
        fixture(self.root / 'sessions', 2)
        self.window.refresh_sources()
        self.window.source_combo.setCurrentIndex(1); self.enter()
        self.window.source_combo.setCurrentIndex(0)
        self.assertEqual(self.window.forward.text(), '')

    def test_blank_fields_do_not_save(self):
        with self.assertRaises(ValueError): self.window.save()
        self.assertFalse(self.window.database.exists())

    def test_save_label_never_changes_after_save_or_select(self):
        self.assertEqual(self.window.save_button.text(), '保存新样本')
        self.enter(); self.window.save()
        self.assertEqual(self.window.save_button.text(), '保存新样本')
        self.assertIn('已新增', self.window.save_state.text())
        self.window.table.selectRow(0)
        self.assertEqual(self.window.save_button.text(), '保存新样本')
        self.window.forward.textEdited.emit('20')
        self.assertEqual(self.window.save_state.text(), '有未保存的修改')

    def test_save_is_visible_at_all_scroll_positions(self):
        self.window.show()
        for width, height in ((1040, 790), (680, 550)):
            self.window.resize(width, height)
            self.app.processEvents()
            bar = self.window.scroll.verticalScrollBar()
            initial = self.window.save_button.mapTo(self.window, QtCore.QPoint(0, 0))
            for position in (bar.minimum(), bar.maximum()):
                bar.setValue(position); self.app.processEvents()
                button = self.window.save_button
                top_left = button.mapTo(self.window, button.rect().topLeft())
                bottom_right = button.mapTo(self.window, button.rect().bottomRight())
                self.assertEqual(top_left, initial)
                self.assertTrue(self.window.rect().contains(top_left))
                self.assertTrue(self.window.rect().contains(bottom_right))
                center = button.mapTo(self.window, button.rect().center())
                self.assertIs(self.window.childAt(center), button)

    def test_empty_sources(self):
        window = Window(self.root / 'empty', self.root / 'empty.json')
        self.assertFalse(window.save_button.isEnabled())
        window.close()

    def test_delete_and_exclude(self):
        self.enter(); self.window.save()
        self.window.table.selectRow(0)
        self.window.role.setCurrentIndex(2); self.window.update_selected()
        self.assertEqual(self.window.samples[0]['role'], 'excluded')
        self.window.table.selectRow(0)
        with patch.object(QtWidgets.QMessageBox, 'question', return_value=QtWidgets.QMessageBox.Yes):
            self.window.delete()
        self.assertEqual(load_samples(self.window.database), [])

    def test_repeated_save_appends_instead_of_overwriting(self):
        self.enter(); self.window.save()
        first = self.window.samples[0]
        self.enter(); self.window.height.setText('24'); self.window.save()
        self.assertEqual(len(self.window.samples), 2)
        self.assertEqual(self.window.samples[0], first)
        self.assertEqual(self.window.samples[1]['measured_forward_left_height_mm'][2], 24)

    def test_clean_refresh_selects_newest_plan(self):
        self.enter(); self.window.save()
        fixture(self.root / 'sessions', 2)
        self.window.refresh_sources()
        self.assertEqual(self.window.source_combo.currentData(), 'session/plan_0002')
        self.enter(); self.window.save()
        self.assertEqual([s['source']['source_id'] for s in self.window.samples],
                         ['session/plan_0001', 'session/plan_0002'])

    def test_edit_only_selected_sample_and_reassign_plan(self):
        self.enter(); self.window.save()
        self.enter(); self.window.save()
        original_id = self.window.samples[0]['sample_id']
        untouched = self.window.samples[1]
        fixture(self.root / 'sessions', 2); self.window.refresh_sources()
        self.window.table.selectRow(0)
        self.window.source_combo.setCurrentIndex(0)
        self.window.forward.setText('20'); self.window.update_selected()
        self.assertEqual(self.window.samples[0]['sample_id'], original_id)
        self.assertEqual(self.window.samples[0]['source']['source_id'], 'session/plan_0002')
        self.assertEqual(self.window.samples[1], untouched)
        self.assertEqual(len(self.window.samples), 2)

    def test_edit_without_selection_does_not_save(self):
        self.enter()
        with self.assertRaises(ValueError): self.window.update_selected()
        self.assertEqual(self.window.samples, [])

    def test_external_change_is_not_overwritten(self):
        self.enter()
        save_samples(self.window.database, [])
        with self.assertRaises(ValueError): self.window.save()
        self.assertEqual(load_samples(self.window.database), [])

    def test_widget_layout_and_render(self):
        self.enter(); self.window.save()
        self.window.show()
        for width, height in ((1040, 790), (680, 550)):
            self.window.resize(width, height); self.app.processEvents()
            for button in self.window.findChildren(QtWidgets.QPushButton):
                self.assertGreaterEqual(button.width(), button.fontMetrics().horizontalAdvance(button.text()) + 28)
            image = self.window.grab().toImage()
            self.assertFalse(image.isNull())


if __name__ == '__main__':
    unittest.main()
