"""Own only the isolated editor and visualization child processes."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parents[1]


def launch_command(urdf, rviz_config=None):
    return ['ros2', 'launch', 'g1_upper_body_moveit_config', 'real_shadow_moveit.launch.py',
            'active_arm:=left', 'task_application_enabled:=true',
            'task_articulated_hands:=true', 'task_waist_enabled:=true',
            'start_state_bridge:=false', 'start_hardware_gateway:=false',
            'hardware_output_enabled:=false', 'start_shadow_executor:=false',
            'enable_rviz_interactive_execution:=false', 'show_planned_overlay:=false',
            'use_rviz:=true', 'export_robot_urdf:='+str(urdf),
            'rviz_config:='+str(rviz_config or ROOT/'assets/camera_reference/real_task.rviz')]


def preview_config():
    import yaml
    config = yaml.safe_load((ROOT/'assets/camera_reference/real_task.rviz').read_text())
    manager = config['Visualization Manager']
    manager['Displays'] = [d for d in manager['Displays'] if not d['Class'].startswith('moveit_rviz_plugin/')]
    for display in manager['Displays']:
        if display['Class'] == 'rviz_default_plugins/RobotModel':
            display['Name'] = 'Offline trajectory preview - NOT measured state'
            display['TF Prefix'] = 'offline_preview'
            display['Description Topic']['Value'] = '/g1/offline_preview/robot_description'
    manager['Global Options']['Frame Rate'] = 30
    return config


def preview_state_parameters(description):
    return {'robot_description': description, 'frame_prefix': 'offline_preview/',
            'publish_frequency': 30.,
            # The preview node already paces samples. A second timestamp gate
            # drops alternating samples when timer jitter crosses 1/30 second.
            'ignore_timestamp': True}


def stop_child(child, sig, timeout):
    if child is None or child.poll() is not None:
        return
    os.killpg(child.pid, sig)
    try:
        child.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(child.pid, signal.SIGKILL)
        child.wait()


def main():
    stopping = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_:stopping.set())
    with tempfile.TemporaryDirectory(prefix='g1-offline-editor-') as runtime:
        urdf = Path(runtime)/'robot.urdf'
        import yaml
        config = Path(runtime)/'preview.rviz'
        config.write_text(yaml.safe_dump(preview_config(), sort_keys=False))
        launch = app = preview = state_publisher = None
        try:
            launch = subprocess.Popen(launch_command(urdf, config), start_new_session=True)
            deadline = time.monotonic()+60
            while not urdf.exists():
                if stopping.wait(.1):return
                if launch.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Offline model generation failed; check service journal')
            params = Path(runtime)/'preview_state.yaml'
            params.write_text(yaml.safe_dump({'/**': {'ros__parameters':
                preview_state_parameters(urdf.read_text())}}))
            state_publisher = subprocess.Popen([
                'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', '--ros-args',
                '-r', '__ns:=/g1/offline_preview', '-r', '__node:=preview_state_publisher',
                '--params-file', str(params)], start_new_session=True)
            preview = subprocess.Popen(['/usr/bin/python3', str(ROOT/'scripts/task_offline_preview.py'),
                                        '--robot-urdf', str(urdf)], start_new_session=True)
            app = subprocess.Popen([
                '/usr/bin/python3', str(ROOT/'scripts/real_task_app.py'),
                '--robot-urdf', str(urdf), '--disconnected', '--offline-editor',
                '--hand-control', '--waist-assist'], start_new_session=True)
            while all(p.poll() is None for p in (app, launch, preview, state_publisher)):
                if stopping.wait(.2):break
            if not stopping.is_set() and (app.poll() not in (None, 0) or
                    any(p.poll() is not None for p in (launch, preview, state_publisher))):
                raise RuntimeError('Offline editor or visualization exited unexpectedly')
        finally:
            stop_child(app, signal.SIGTERM, 90)
            stop_child(preview, signal.SIGINT, 5)
            stop_child(state_publisher, signal.SIGINT, 5)
            stop_child(launch, signal.SIGINT, 15)


if __name__ == '__main__':
    main()
