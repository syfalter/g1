"""Launch the existing disconnected editor with an isolated candidate catalog."""
import argparse
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--catalog',type=Path,default=ROOT/'data/rail_deployment/trajectory_only_20260929/panel')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '96' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('Source scripts/tcp_a_offline_environment.sh before opening this isolated preview')
    catalog = args.catalog.resolve()
    if not (catalog/'catalog.json').is_file():
        parser.error('Missing pinned preview catalog')
    import task_editor_offline_panel
    Original = task_editor_offline_panel.OfflineEditorWindow
    class ReviewWindow(Original):
        def __init__(self,node):
            super().__init__(node,snapshot_directory=catalog,
                settings_path=catalog/'editor_settings.json')
            self.setWindowTitle('G1 OFFLINE / '+catalog.name+' / NOT DEPLOYED')
            self.optimizer.generate.setEnabled(False)
            self.optimizer.generate.hide()
    task_editor_offline_panel.OfflineEditorWindow = ReviewWindow
    sys.argv = [sys.argv[0],'--robot-urdf',str(ROOT/'data/trajectory_edit_audit/fast_timing_all/panel_robot.urdf'),
                '--disconnected','--offline-editor','--hand-control','--waist-assist']
    from real_task_app import main as editor
    editor()


if __name__ == '__main__':
    main()
