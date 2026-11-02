"""Offline-only base offsets, excluded from both arm planning groups."""
import xml.etree.ElementTree as ET

STANCE_JOINTS = ('workbench_stance_x', 'workbench_stance_y')
STANCE_LIMIT_M = .30


def add_stance_joints(robot):
    root = robot.find("joint[@name='preview_world_pelvis']")
    parent = root.find('parent').get('link')
    for name, axis in zip(STANCE_JOINTS, ('1 0 0', '0 1 0')):
        ET.SubElement(robot, 'link', name=name + '_link')
        joint = ET.SubElement(robot, 'joint', name=name, type='prismatic')
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=name + '_link')
        ET.SubElement(joint, 'axis', xyz=axis)
        ET.SubElement(joint, 'limit', lower=str(-STANCE_LIMIT_M), upper=str(STANCE_LIMIT_M),
                      effort='1', velocity='0.1')
        parent = name + '_link'
    root.find('parent').set('link', parent)
