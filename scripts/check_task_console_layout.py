"""Render Qt widgets directly; screen capture is blank under this WSL compositor."""
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock
os.environ['QT_QPA_PLATFORM']='offscreen'
from PyQt5 import QtWidgets
from real_task_app import Window
from task_session import Session


def main():
    app=QtWidgets.QApplication([])
    node=SimpleNamespace(args=SimpleNamespace(hardware=False,disconnected=True),lock=threading.RLock(),
        hand_control=True,planner=SimpleNamespace(waist_assist=True),
        status_pub=Mock(),display_pub=Mock(),
        session=Session(),worker=None,status='Waiting for live LowState. No hardware command has been sent.',
        gateway={'phase':'OUTPUT_DISABLED','waist_kp':'60.0','waist_kd':'4.5',
                 'waist_ki':'20.0','waist_integral_torque_limit_nm':'1.2',
                 'waist_integral_torque_nm':'0.0','waist_integral_mode':'disabled'},
        gateway_received=time.monotonic(),
        stop=lambda:None,set_waist_gains=Mock(),launch_work=Mock())
    window=Window(node); window.show(); window.refresh(); app.processEvents()
    directory=Path(__file__).resolve().parents[1]/'logs/task_application_gateway_test'
    counts=[]
    for index,name in enumerate(('plan','saved')):
        window.tabs.setCurrentIndex(index);app.processEvents()
        visible=[button for button in window.findChildren(QtWidgets.QPushButton) if button.isVisible()]
        for button in visible:
            top_left=button.mapTo(window,button.rect().topLeft())
            bottom_right=button.mapTo(window,button.rect().bottomRight())
            assert 0<=top_left.x() and bottom_right.x()<window.width(),button.text()
            assert 0<=top_left.y() and bottom_right.y()<window.height(),button.text()
            assert button.fontMetrics().horizontalAdvance(button.text())+12<=button.width(),button.text()
        window.grab().save(str(directory/f'console_widget_{name}.png'))
        counts.append(len(visible))
    window.tabs.setCurrentIndex(0);app.processEvents()
    window.grab().save(str(directory/'console_widget.png'))
    print('Qt widget layout checked:',window.width(),window.height(),'visible buttons:',counts)
    window.timer.stop(); window.hide()


if __name__=='__main__': main()
