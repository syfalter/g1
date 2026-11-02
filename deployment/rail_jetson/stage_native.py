"""Stage a headless source-only gateway build without changing the live workspace."""
import argparse
import hashlib
import shutil
from pathlib import Path
from bundle import ROOT, write


def manifest(name, dependencies):
    return ('<?xml version="1.0"?>\n<package format="3">\n'
            f'<name>{name}</name><version>0.0.1</version><description>Offline migration build</description>\n'
            '<maintainer email="local@example.invalid">Local commissioning</maintainer><license>BSD-3-Clause</license>\n'
            '<buildtool_depend>ament_cmake</buildtool_depend><buildtool_depend>rosidl_default_generators</buildtool_depend>\n'
            '<exec_depend>rosidl_default_runtime</exec_depend><member_of_group>rosidl_interface_packages</member_of_group>\n'
            + ''.join(f'<depend>{dep}</depend>\n' for dep in dependencies)
            + '<export><build_type>ament_cmake</build_type></export></package>\n')


def stage(destination, control_msgs):
    destination = Path(destination)
    if destination.exists():
        raise ValueError('Use a new source staging directory')
    src = destination / 'src'
    src.mkdir(parents=True)
    hashes = {}
    for name in ('unitree_api', 'unitree_hg'):
        original = ROOT.parent / 'unitree_ws/unitree_ros2/cyclonedds_ws/src/unitree' / name
        target = src / name
        shutil.copytree(original / 'msg', target / 'msg')
        messages = sorted((target / 'msg').glob('*.msg'))
        # No Connext-only IDL generator is needed for the Cyclone/FastDDS build.
        cmake = f'cmake_minimum_required(VERSION 3.5)\nproject({name})\n'
        cmake += 'find_package(ament_cmake REQUIRED)\nfind_package(geometry_msgs REQUIRED)\nfind_package(rosidl_default_generators REQUIRED)\n'
        cmake += 'rosidl_generate_interfaces(${PROJECT_NAME}\n' + ''.join(f' "msg/{m.name}"\n' for m in messages)
        cmake += ' DEPENDENCIES geometry_msgs)\nament_export_dependencies(rosidl_default_runtime)\nament_package()\n'
        (target / 'CMakeLists.txt').write_text(cmake)
        (target / 'package.xml').write_text(manifest(name, ['geometry_msgs']))
    target = src / 'control_msgs'
    if (control_msgs / 'CMakeLists.txt').is_file():
        shutil.copytree(control_msgs, target, ignore=shutil.ignore_patterns('.git'))
    else:
        # Preserve the exact interface definitions used by the successful WSL build.
        # This private overlay does not replace Foxy's system packages.
        dependencies = ['action_msgs', 'builtin_interfaces', 'geometry_msgs',
                        'sensor_msgs', 'std_msgs', 'trajectory_msgs']
        interfaces = []
        for kind in ('msg', 'srv', 'action'):
            for original in sorted((control_msgs / kind).glob('*.' + kind)):
                path = target / kind / original.name
                path.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(original, path)
                interfaces.append(str(path.relative_to(target)))
        shutil.copy2(control_msgs / 'package.xml', target / 'package.xml')
        cmake = 'cmake_minimum_required(VERSION 3.5)\nproject(control_msgs)\nfind_package(ament_cmake REQUIRED)\nfind_package(rosidl_default_generators REQUIRED)\n'
        cmake += ''.join(f'find_package({dep} REQUIRED)\n' for dep in dependencies)
        cmake += 'rosidl_generate_interfaces(${PROJECT_NAME}\n' + ''.join(f' "{name}"\n' for name in interfaces)
        cmake += ' DEPENDENCIES ' + ' '.join(dependencies) + ')\nament_export_dependencies(rosidl_default_runtime)\nament_package()\n'
        (target / 'CMakeLists.txt').write_text(cmake)
    target = src / 'g1_unitree_control'
    original = ROOT / 'src/g1_unitree_control'
    shutil.copytree(original / 'include', target / 'include')
    shutil.copytree(original / 'srv', target / 'srv')
    (target / 'src').mkdir()
    for name in ('arm_sdk_hardware_gateway.cpp', 'arm_state_bridge.cpp'):
        shutil.copy2(original / 'src' / name, target / 'src' / name)
    gateway = target / 'src/arm_sdk_hardware_gateway.cpp'
    text = gateway.read_text()
    if text.count('KDL::Joint::Fixed') != 1:
        raise ValueError('Review KDL compatibility replacement after upstream changes')
    gateway.write_text(text.replace('KDL::Joint::Fixed', 'KDL::Joint::None'))
    shutil.copy2(Path(__file__).with_name('foxy_compat.hpp'), target / 'include/foxy_compat.hpp')
    dependencies = ['control_msgs', 'diagnostic_msgs', 'rcl_interfaces', 'rclcpp', 'rclcpp_action',
                    'sensor_msgs', 'std_msgs', 'std_srvs', 'trajectory_msgs', 'unitree_api', 'unitree_hg', 'urdf', 'kdl_parser']
    cmake = 'cmake_minimum_required(VERSION 3.5)\nproject(g1_unitree_control)\nfind_package(ament_cmake REQUIRED)\n'
    cmake += 'find_package(Eigen3 REQUIRED)\nfind_package(orocos_kdl REQUIRED)\nfind_package(rosidl_default_generators REQUIRED)\n'
    cmake += ''.join(f'find_package({dep} REQUIRED)\n' for dep in dependencies)
    cmake += 'rosidl_generate_interfaces(${PROJECT_NAME} "srv/ResumeTaskHold.srv")\n'
    for name in ('arm_sdk_hardware_gateway', 'arm_state_bridge'):
        cmake += f'add_executable({name} src/{name}.cpp)\ntarget_compile_features({name} PRIVATE cxx_std_17)\n'
        cmake += f'ament_target_dependencies({name} ' + ' '.join(dependencies) + ')\n'
        cmake += f'target_link_libraries({name} Eigen3::Eigen orocos-kdl)\n'
        cmake += f'install(TARGETS {name} DESTINATION lib/${{PROJECT_NAME}})\n'
    cmake += 'target_compile_options(arm_sdk_hardware_gateway PRIVATE -include ${CMAKE_CURRENT_SOURCE_DIR}/include/foxy_compat.hpp)\n'
    cmake += 'rosidl_target_interfaces(arm_sdk_hardware_gateway ${PROJECT_NAME} "rosidl_typesupport_cpp")\nament_package()\n'
    (target / 'CMakeLists.txt').write_text(cmake)
    (target / 'package.xml').write_text(manifest('g1_unitree_control', dependencies))
    for path in src.rglob('*'):
        if path.is_file(): hashes[str(path.relative_to(src))] = hashlib.sha256(path.read_bytes()).hexdigest()
    write(destination / 'source_hashes.json', hashes)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--control-msgs', required=True, type=Path)
    args = parser.parse_args()
    stage(args.destination, args.control_msgs)
