"""One consistent, explicitly assumed index-only posture for task planning."""
import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
INDEX_OPEN=.15/1.1169
CONTACT_PARTS=('index_2','index_rubber_2','index_pad','index_tip')


def apply_task_hand_model(robot):
    assets=ROOT/'assets/workbench/rh56dfx/task'
    for side in ('left','right'):
        if robot.find(f"link[@name='{side}_index_tcp']") is not None:
            raise ValueError('Task index assembly already installed')
        hand=robot.find(f"link[@name='{side}_rubber_hand']")
        if hand is None:raise ValueError('Missing '+side+' hand')
        for kind in ('visual','collision'):
            hand.find(f'{kind}/geometry/mesh').set('filename',(assets/f'{side}_body_{kind}.stl').as_uri())
        assembly=ET.parse(assets/f'{side}_index.urdf').getroot()
        for original in assembly:
            element=copy.deepcopy(original)
            if element.tag=='joint' and element.get('type')=='revolute':
                angle=INDEX_OPEN if element.get('name')==side+'_index_1_joint' else 0.
                origin=element.find('origin')
                rotation=Rotation.from_euler('xyz',[float(v) for v in origin.get('rpy','0 0 0').split()])
                axis=np.array([float(v) for v in element.find('axis').get('xyz').split()])
                rotation=rotation*Rotation.from_rotvec(axis*angle)
                origin.set('rpy',' '.join(map(str,rotation.as_euler('xyz'))))
                element.set('type','fixed')
                for tag in ('axis','limit','mimic'):
                    for child in element.findall(tag):element.remove(child)
            robot.append(element)


def fixed_hand_pairs(robot):
    """Only rigid pieces within the same assumed hand, never other robot links."""
    pairs=[]
    for side in ('left','right'):
        names=[side+'_rubber_hand']+[link.get('name') for link in robot.findall('link')
                                     if link.get('name').startswith(side+'_index_')]
        for i,name in enumerate(names):
            pairs.extend((name,other) for other in names[i+1:])
    return pairs


def require_task_hand_model(model):
    for side in ('left','right'):
        for part in CONTACT_PARTS+('index_tcp',):
            if model.robot.find(f"link[@name='{side}_{part}']") is None:
                raise RuntimeError('MODEL_MISMATCH: reload the index-only task planning model; no fallback to the open hand')
        if model.robot.find(f"joint[@name='{side}_middle_1_joint']") is not None:
            continue
        mesh=model.robot.find(f"link[@name='{side}_rubber_hand']/collision/geometry/mesh")
        if mesh is None or not mesh.get('filename','').endswith(f'/task/{side}_body_collision.stl'):
            raise RuntimeError('MODEL_MISMATCH: non-index fingers are not the folded task model')


def apply_articulated_task_hands(robot):
    """Preserve the old mount/TCP, but retain all six actuator model joints."""
    import xacro
    source = ROOT/'assets/rh56dfx_description'
    metadata = json.loads((ROOT/'assets/workbench/rh56dfx/task/provenance.json').read_text())
    for side in ('left', 'right'):
        if robot.find(f"link[@name='{side}_index_tcp']") is not None:
            raise ValueError('Hand assembly already installed')
        wrapper = f'''<robot xmlns:xacro="http://ros.org/wiki/xacro">
          <xacro:include filename="{source}/urdf/rh56dfx_macro.xacro"/>
          <xacro:rh56dfx_hand side="{side}" prefix="{side}_"/></robot>'''
        document=xacro.parse(wrapper);xacro.process_doc(document)
        hand_model=ET.fromstring(document.toxml())
        base=side+'_hand_base'
        descendants={base}
        joints=hand_model.findall('joint')
        for _ in joints:
            for joint in joints:
                if joint.find('parent').get('link') in descendants:
                    descendants.add(joint.find('child').get('link'))
        mount=np.asarray(metadata['hands'][side]['mount_transform'])
        placeholder=robot.find(f"link[@name='{side}_rubber_hand']")
        for tag in ('visual','collision'):
            for child in placeholder.findall(tag):placeholder.remove(child)
        for link in hand_model.findall('link'):
            if link.get('name') not in descendants:continue
            for mesh in link.iter('mesh'):
                relative=mesh.get('filename').removeprefix('package://rh56dfx_description/')
                mesh.set('filename',(source/relative).as_uri())
            robot.append(copy.deepcopy(link))
        for joint in joints:
            if joint.find('parent').get('link') in descendants:
                robot.append(copy.deepcopy(joint))
        fixed=ET.SubElement(robot,'joint',name=side+'_task_hand_mount',type='fixed')
        ET.SubElement(fixed,'parent',link=side+'_rubber_hand')
        ET.SubElement(fixed,'child',link=base)
        ET.SubElement(fixed,'origin',xyz=' '.join(map(str,mount[:3,3])),
                      rpy=' '.join(map(str,Rotation.from_matrix(mount[:3,:3]).as_euler('xyz'))))
        previous=ET.parse(ROOT/f'assets/workbench/rh56dfx/task/{side}_index.urdf').getroot()
        for name,tag in ((side+'_index_tcp','link'),(side+'_index_tcp_fixed','joint')):
            robot.append(copy.deepcopy(previous.find(f"{tag}[@name='{name}']")))


def articulated_hand_pairs(robot):
    """Adjacent links and connected rigid pieces only; never disable inter-finger checks."""
    pairs=set()
    for side in ('left','right'):
        names={link.get('name') for link in robot.findall('link') if
               link.get('name').startswith(tuple(side+'_'+part for part in
                  ('hand_base','palm','thumb_','index_','middle_','ring_','little_')))}
        names.add(side+'_rubber_hand')
        rigid={name:{name} for name in names}
        for joint in robot.findall('joint'):
            a,b=joint.find('parent').get('link'),joint.find('child').get('link')
            if a not in names or b not in names:continue
            pairs.add(tuple(sorted((a,b))))
            if joint.get('type')=='fixed':
                union=rigid[a]|rigid[b]
                for name in union:rigid[name]=union
        for group in rigid.values():
            for a in group:
                for b in group:
                    if a<b:pairs.add((a,b))
    return sorted(pairs)
