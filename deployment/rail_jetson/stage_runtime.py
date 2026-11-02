"""Stage only release tooling and offline planning dependencies; no controller launch."""
import argparse
import ast
import hashlib
from pathlib import Path
import shutil

from bundle import ROOT, write

PLANNER_FILES = ('workbench_model.py', 'task_path_smoothing.py', 'task_trajectory.py',
                 'task_trajectory_speed.py', 'task_local_timing.py', 'task_planning_config.py',
                 'task_stage_control.py', 'task_hand_control.py', 'task_hand_completion.py', 'task_sdk_environment.py',
                 'task_dring_demo_hand.py', 'task_rail_hand.py', 'task_hand_client.cpp', 'task_dds_observer.cpp')
DEPLOY_FILES = ('bundle.py', 'releases.py', 'coordinator.py', 'server.py', 'executor.py',
                'upload_release.py', 'preview_service.py', 'cloud_executor.py',
                'cloud_service.py', 'offline_transport.py', 'check_cloud_http.py', 'check_cloud_segments.py',
                'README.md', 'PUSH_TESTING.md', 'HEADLESS_TESTING.md', 'ros_transport.py',
                'native_worker.py', 'native_runtime.py', 'production_api.py', 'select_production.py',
                'run_standalone.sh', 'standalone_supervisor.py', 'native_acceptance.py', 'robot_request.py',
                'check_bernstein.py', 'STANDALONE_TESTING.md', 'dds_authority.py', 'build_native_helpers.sh',
                'check_dds_observer.py', 'remote_recovery.py', 'gateway_process.py', 'http_relay.py',
                'eln_interface_entrypoint.py', 'interface_relay.json', 'home_pose.py', 'RETURN_HOME.md',
                'compositions.py', 'COMPOSITIONS.md')


def stage(destination):
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    files = ([ROOT / 'scripts' / f for f in PLANNER_FILES] +
             [Path(__file__).parent / f for f in DEPLOY_FILES] +
             list((Path(__file__).parent / 'web').glob('*')))
    hashes = {}
    for source in files:
        relative = source.relative_to(ROOT)
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        if source.name == 'workbench_model.py':
            tree = ast.parse(source.read_text())
            class MatrixCompatibility(ast.NodeTransformer):
                count = 0
                def visit_Call(self, node):
                    self.generic_visit(node)
                    if isinstance(node.func, ast.Attribute) and node.func.attr == 'as_matrix':
                        self.count += 1
                        return ast.copy_location(ast.Call(func=ast.Name(id='_rotation_matrix', ctx=ast.Load()),
                                                          args=[node.func.value], keywords=[]), node)
                    return node
            compatibility = MatrixCompatibility()
            tree = compatibility.visit(tree)
            if compatibility.count != 3:
                raise ValueError('Review SciPy matrix API compatibility after model changes')
            tree.body += ast.parse('def _rotation_matrix(rotation):\n'
                                   '    if hasattr(rotation, "as_matrix"):\n'
                                   '        return rotation.as_matrix()\n'
                                   '    return rotation.as_dcm()\n').body
            target.write_text(ast.unparse(ast.fix_missing_locations(tree)) + '\n')
        hashes[str(relative)] = hashlib.sha256(target.read_bytes()).hexdigest()
    write(destination / 'runtime_manifest.json', dict(files=hashes,
        hardware_output_enabled=False, controller_started=False, contract='g1-recorded-actions-v2',
        compatibility=['isolated OfflineModel uses SciPy as_dcm only when as_matrix is unavailable']))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', type=Path)
    stage(parser.parse_args().destination)
