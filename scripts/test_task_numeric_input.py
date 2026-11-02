"""Unbounded editor inputs, independent of ROS and hardware."""
import unittest

import numpy as np
from PyQt5 import QtGui, QtWidgets

from task_numeric_input import FreeDoubleSpinBox, FreeIntegerSpinBox


class NumericInputTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_positive_sizes_and_scientific_notation(self):
        widget = FreeDoubleSpinBox(18., positive=True)
        for value in (.00001, 100000.):
            widget.setValue(value)
            self.assertEqual(widget.value(), value)
        self.assertEqual(widget.valueFromText('1e-7'), 1e-7)
        self.assertEqual(widget.validate('1e-7', 4)[0], QtGui.QValidator.Acceptable)
        for value in (0., -1., float('nan'), float('inf')):
            with self.assertRaises(ValueError):widget.setValue(value)

    def test_offsets(self):
        widget = FreeDoubleSpinBox(-100.)
        self.assertEqual(widget.value(), -100.)

    def test_display_count_exceeds_native_integer(self):
        widget = FreeIntegerSpinBox()
        for count in (1, 501, 10000, 2**64):
            widget.setValue(count)
            self.assertEqual(widget.value(), count)
            self.assertEqual(widget.lineEdit().text(), str(count))
        self.assertIsNone(widget.maximum())
        widget.lineEdit().setText('10001')
        widget.commit()
        self.assertEqual(widget.value(), 10001)

    def test_render_sampling_has_no_500_limit(self):
        from task_trace_markers import sparse_indices
        poses = np.repeat(np.eye(4)[None], 1001, axis=0)
        poses[:, 0, 3] = np.arange(1001)
        self.assertEqual(len(sparse_indices(poses, .1, 10000)), 1001)
        self.assertEqual(len(sparse_indices(poses, .1, 2**64)), 1001)


if __name__ == '__main__':unittest.main()
