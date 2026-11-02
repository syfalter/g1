"""Offscreen widgets and real-model comparison, with mock publishers only."""
import argparse
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace as S
from unittest.mock import Mock

import numpy as np
from PyQt5 import QtWidgets,QtCore

from task_dring_demo_control import DringDemoControl
from task_dring_demo_panel import DringDemoPanel
from task_landing_gear_control import LandingGearControl
from task_landing_gear_panel import LandingGearPanel
from task_rail_control import RailControl
from task_rail_panel import RailPanel
from task_trace_edit import curve
from task_trace_versions import append
from workbench_model import OfflineModel

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('robot',type=Path)
    parser.add_argument('--output',type=Path,default=ROOT/'data/trajectory_edit_audit/screenshots')
    parser.add_argument('--fast',action='store_true')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    app=QtWidgets.QApplication([])
    n=S(args=S(robot_urdf=args.robot,hardware=False,disconnected=True),model=OfflineModel(args.robot),
        lock=threading.RLock(),session=S(phase='ARRIVED_HOLD',plan=None,current={},generation=0,
            invalidate=Mock(),stopped=False),display_pub=Mock(),demo_preview_pub=Mock(),marker_pub=Mock(),
        run_dir=ROOT/'data/task_sessions/offline_editor_layout',worker=None,shutdown_requested=False,
        hand_control=False,hand_states={},prepared={},gateway={},gateway_received=0.,status='')
    n.session.measured=lambda **_:dict(n.session.current)
    n.launch_work=lambda work:work()
    n.demo=DringDemoControl(n);n.landing_gear=LandingGearControl(n)
    n.rail=RailControl(n);n.rail.restore_latest()
    for name,control,panel_type,slot in (
            ('dring',n.demo,DringDemoPanel,'left_5'),
            ('gear',n.landing_gear,LandingGearPanel,'lower_to_upper'),
            ('rail',n.rail,RailPanel,'push')):
        source=control.config['traces'][slot]
        duration=source['raw']['time_s'][-1]
        if args.fast:
            from task_trace_timing import generate
            from task_planning_config import load_parameters
            params,_=load_parameters()
            candidate=generate(source,n.model,control.editor_side(),params,workflow=control.workflow,
                config=control.config,slot=slot,denoise=True)
        else:candidate=curve(source,np.linspace(0.,duration,5).tolist())
        # In-memory examples only. No persistence methods or hardware endpoints.
        control.config=append(control.config,slot,candidate);control.revision+=1
        window=S(node=n,invoke=lambda action:action(),clear=QtWidgets.QCheckBox())
        panel=panel_type(window)
        if name=='dring':
            from task_dring_teach_data import SLOTS
            panel.table.selectRow(list(SLOTS).index(slot))
        elif name=='gear':
            from task_landing_gear_data import SLOTS
            panel.table.selectRow(list(SLOTS).index(slot))
        else:panel.trace_stage.setCurrentIndex(panel.trace_stage.findData(slot))
        panel.show();panel.refresh(False);app.processEvents()
        panel.editor.versions.item(0,3).setCheckState(QtCore.Qt.Checked)
        panel.editor.versions.item(1,3).setCheckState(QtCore.Qt.Checked)
        began=time.monotonic()
        while not panel.editor.comparison_status.text() and time.monotonic()-began<20:
            app.processEvents();time.sleep(.02)
        if not panel.editor.comparison_status.text():raise RuntimeError(panel.editor.error.text() or 'Comparison timed out')
        print(name,panel.editor.comparison_status.text(),flush=True)
        scroll=panel.findChild(QtWidgets.QScrollArea)
        for width,height in ((620,900),(1100,1000)):
            panel.resize(width,height);app.processEvents()
            for section,value in (('top',0),('editor',panel.editor.y()),('bottom',scroll.verticalScrollBar().maximum())):
                scroll.verticalScrollBar().setValue(value);app.processEvents()
                for button in panel.editor.findChildren(QtWidgets.QPushButton):
                    if button.fontMetrics().horizontalAdvance(button.text())+24>button.width():
                        raise AssertionError('Clipped button: '+button.text())
                panel.grab().save(str(args.output/f'{name}_{width}_{section}.png'))
        panel.close();app.processEvents()
        thread=panel.editor.marker_thread
        if thread:thread.join(timeout=10)
        if thread and thread.is_alive():raise RuntimeError('Marker worker did not finish')
    print('PASS: three panels, two sizes, real-model comparison, mock publishers only',flush=True)


if __name__=='__main__':main()
