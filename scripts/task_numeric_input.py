"""Finite numeric input without application-defined magnitude limits."""
import math
import sys

from PyQt5 import QtCore, QtGui, QtWidgets


class FreeDoubleSpinBox(QtWidgets.QDoubleSpinBox):
    def __init__(self, value=0., *, positive=False):
        super().__init__()
        self.positive = positive
        self.setKeyboardTracking(False)
        super().setDecimals(323)
        super().setRange(0. if positive else -sys.float_info.max, sys.float_info.max)
        self.setValue(value)

    def accepts(self, value):
        return math.isfinite(value) and (not self.positive or value > 0.)

    def setValue(self, value):
        if not self.accepts(value):raise ValueError('请输入有限正数' if self.positive else '请输入有限数值')
        super().setValue(value)

    def textFromValue(self, value):
        return format(value, '.12g')

    def valueFromText(self, text):
        text = text.strip()
        if self.suffix() and text.endswith(self.suffix()):text = text[:-len(self.suffix())]
        if self.prefix() and text.startswith(self.prefix()):text = text[len(self.prefix()):]
        try:
            value = float(text.strip())
            return value if self.accepts(value) else self.value()
        except ValueError:return self.value()

    def validate(self, text, position):
        raw = text.strip()
        if self.suffix() and raw.endswith(self.suffix()):raw = raw[:-len(self.suffix())]
        if self.prefix() and raw.startswith(self.prefix()):raw = raw[len(self.prefix()):]
        try:
            value = float(raw)
            state = QtGui.QValidator.Acceptable if self.accepts(value) else QtGui.QValidator.Intermediate
        except ValueError:state = QtGui.QValidator.Intermediate
        return state, text, position

    def stepBy(self, steps):
        value = self.value()+steps*self.singleStep()
        if self.positive and value <= 0.:value = self.value()/10.
        if self.accepts(value):self.setValue(value)


class FreeIntegerSpinBox(QtWidgets.QAbstractSpinBox):
    valueChanged = QtCore.pyqtSignal(object)

    def __init__(self, value=40):
        super().__init__()
        self._value = value
        self.lineEdit().setText(str(value))
        self.editingFinished.connect(self.commit)

    def value(self):return self._value
    def minimum(self):return 1
    def maximum(self):return None

    def accepts(self, value):return type(value) is int and value > 0

    def setValue(self, value):
        if not self.accepts(value):raise ValueError('显示点数须为正整数')
        changed = value != self._value
        self._value = value; self.lineEdit().setText(str(value))
        if changed:self.valueChanged.emit(value)

    def commit(self):
        try:self.setValue(int(self.lineEdit().text()))
        except ValueError:self.lineEdit().setText(str(self._value))

    def validate(self, text, position):
        try:valid = self.accepts(int(text))
        except ValueError:valid = False
        return (QtGui.QValidator.Acceptable if valid else QtGui.QValidator.Intermediate), text, position

    def stepBy(self, steps):self.setValue(max(1,self._value+steps))

    def stepEnabled(self):
        return self.StepUpEnabled | (self.StepDownEnabled if self._value>1 else self.StepNone)
