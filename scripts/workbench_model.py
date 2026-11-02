"""Forward kinematics for checking offline plan artifacts against their URDF."""
import xml.etree.ElementTree as ET
import math
import threading

import numpy as np
from scipy.spatial.transform import Rotation


class OfflineModel:
    def __init__(self, path):
        self.robot = ET.parse(path).getroot()
        self.parents = {}
        self.joints = {}
        self.limits = {}
        self._local = threading.local()
        for joint in self.robot.findall('joint'):
            origin = joint.find('origin')
            transform = np.eye(4)
            if origin is not None:
                transform[:3, :3] = Rotation.from_euler('xyz', [float(v) for v in origin.get('rpy', '0 0 0').split()]).as_matrix()
                transform[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
            self.parents[joint.find('child').get('link')] = (joint, transform)
            axis=joint.find('axis');mimic=joint.find('mimic')
            axis_value=np.array([float(v) for v in axis.get('xyz').split()]) if axis is not None else np.zeros(3)
            cardinal=None
            nonzero=np.flatnonzero(np.abs(axis_value)>1e-15)
            if len(nonzero)==1 and abs(abs(axis_value[nonzero[0]])-1.)<1e-15:
                cardinal=(int(nonzero[0]),float(axis_value[nonzero[0]]))
            self.joints[joint.find('child').get('link')]=(joint.find('parent').get('link'),
                joint.get('type'),transform,
                axis_value,cardinal,
                joint.get('name') if mimic is None else mimic.get('joint'),
                1. if mimic is None else float(mimic.get('multiplier','1')),
                0. if mimic is None else float(mimic.get('offset','0')))
            limit=joint.find('limit')
            if limit is not None and limit.get('lower') is not None and limit.get('upper') is not None:
                self.limits[joint.get('name')]=(float(limit.get('lower')),float(limit.get('upper')))

    def transform(self, link, q):
        # Exact-state, thread-local memoization shares ancestor FK across links.
        # Copy both the input state and returned matrix: callers may mutate them.
        if getattr(self._local,'q',None)!=q:
            self._local.q=dict(q);self._local.frames={}
        return self._transform(link,self._local.q,self._local.frames).copy()

    def transforms(self, links, q):
        """Resolve several links with one state copy and one shared ancestor cache."""
        state=dict(q);frames={}
        result={link:self._transform(link,state,frames).copy() for link in dict.fromkeys(links)}
        self._local.q=state;self._local.frames=frames
        return result

    def pose_jacobian(self,link,q,names,offset):
        """Root-frame pose and geometric Jacobian, including revolute mimic contributions."""
        if len(set(names))!=len(names) or any(name not in q for name in names):
            raise ValueError('Jacobian requires distinct, recorded joint values')
        offset=np.asarray(offset,dtype=float)
        if offset.shape!=(4,4) or not np.isfinite(offset).all():raise ValueError('Invalid TCP transform')
        pose=self.transform(link,q)@offset
        jacobian=np.zeros((6,len(names)));columns={name:index for index,name in enumerate(names)}
        while link in self.joints:
            parent,kind,origin,axis,_,name,multiplier,_=self.joints[link]
            if name in columns and kind in ('revolute','continuous','prismatic'):
                frame=self.transform(parent,q)@origin
                direction=frame[:3,:3]@axis
                index=columns[name]
                if kind in ('revolute','continuous'):
                    direction=direction*multiplier
                    jacobian[:3,index]+=np.cross(direction,pose[:3,3]-frame[:3,3])
                    jacobian[3:,index]+=direction
                else:
                    if self.parents[link][0].find('mimic') is not None:
                        raise ValueError('Prismatic mimic Jacobian is unsupported')
                    jacobian[:3,index]+=direction
            link=parent
        return pose,jacobian

    def batch_transforms(self,links,q,joint_names,positions):
        """Exact FK for one sampled trajectory using vectorized parent transforms."""
        positions=np.asarray(positions,dtype=float)
        if positions.ndim!=2 or positions.shape[1]!=len(joint_names) or not np.isfinite(positions).all():
            raise ValueError('Invalid batched joint positions')
        columns={name:index for index,name in enumerate(joint_names)}
        count=len(positions);frames={};identity=np.broadcast_to(np.eye(4),(count,4,4))
        def resolve(link):
            if link in frames:return frames[link]
            if link not in self.joints:
                frames[link]=identity
                return frames[link]
            parent,kind,origin,axis,cardinal,name,multiplier,offset=self.joints[link]
            result=np.matmul(resolve(parent),origin)
            values=(positions[:,columns[name]] if name in columns else np.full(count,q.get(name,0.)))
            if kind in ('revolute','continuous'):
                angles=values*multiplier+offset
                if cardinal is None:
                    result[:,:3,:3]=np.matmul(result[:,:3,:3],Rotation.from_rotvec(angles[:,None]*axis).as_matrix())
                else:
                    index,sign=cardinal;c=np.cos(sign*angles)[:,None];s=np.sin(sign*angles)[:,None]
                    rotation=result[:,:3,:3]
                    if index==0:
                        first=rotation[:,:,1].copy();second=rotation[:,:,2].copy()
                        rotation[:,:,1]=c*first+s*second;rotation[:,:,2]=-s*first+c*second
                    elif index==1:
                        first=rotation[:,:,0].copy();second=rotation[:,:,2].copy()
                        rotation[:,:,0]=c*first-s*second;rotation[:,:,2]=s*first+c*second
                    else:
                        first=rotation[:,:,0].copy();second=rotation[:,:,1].copy()
                        rotation[:,:,0]=c*first+s*second;rotation[:,:,1]=-s*first+c*second
            elif kind=='prismatic':
                result[:,:3,3]+=np.einsum('nij,nj->ni',result[:,:3,:3],values[:,None]*axis)
            frames[link]=result
            return result
        return {link:resolve(link) for link in dict.fromkeys(links)}

    def _transform(self,link,q,frames):
        if link in frames:return frames[link]
        if link not in self.joints:
            frames[link]=np.eye(4)
            return frames[link]
        parent,kind,origin,axis,cardinal,name,multiplier,offset=self.joints[link]
        result = self._transform(parent,q,frames) @ origin
        if kind in ('revolute', 'continuous'):
            angle = q.get(name,0)*multiplier+offset
            if cardinal is None:
                result[:3,:3]=result[:3,:3]@Rotation.from_rotvec(axis*angle).as_matrix()
            else:
                index,sign=cardinal;c=math.cos(sign*angle);s=math.sin(sign*angle)
                rotation=result[:3,:3]
                if index==0:
                    first=rotation[:,1].copy();second=rotation[:,2].copy()
                    rotation[:,1]=c*first+s*second;rotation[:,2]=-s*first+c*second
                elif index==1:
                    first=rotation[:,0].copy();second=rotation[:,2].copy()
                    rotation[:,0]=c*first-s*second;rotation[:,2]=s*first+c*second
                else:
                    first=rotation[:,0].copy();second=rotation[:,1].copy()
                    rotation[:,0]=c*first+s*second;rotation[:,1]=-s*first+c*second
        elif kind == 'prismatic':
            result[:3,3]+=result[:3,:3]@(axis*q.get(self.parents[link][0].get('name'),0))
        frames[link]=result
        return result
