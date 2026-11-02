"""Positive finite, freely entered trajectory multiplier for Qt previews."""
import math

from PyQt5 import QtCore, QtGui, QtWidgets


class SpeedInput(QtWidgets.QLineEdit):
    valueChanged = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__('1', parent)
        self.setValidator(QtGui.QDoubleValidator(self))
        self.setMaximumWidth(140)
        self.setAccessibleName('RViz trajectory multiplier')
        self.setToolTip('Positive multiplier, e.g. 0.5, 4, 24; RViz only, no motion authorization')
        self.textChanged.connect(lambda _: self.valueChanged.emit())

    def value(self):
        try:
            value = float(self.text())
        except ValueError as error:
            raise ValueError('Enter a positive finite trajectory multiplier') from error
        if not math.isfinite(value) or value <= 0:
            raise ValueError('Enter a positive finite trajectory multiplier')
        return value

    def setValue(self, value):
        self.setText(str(value))
