"""Offline FCL hand/base checks. No scene mutation or hardware imports."""
import json
import math
from pathlib import Path
import subprocess
import sys
from collections.abc import Mapping
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
# Only this isolated experiment uses the CAD environment's FCL extension.
sys.path.append(str(ROOT/'.venv-cad/lib/python3.10/site-packages'))
import fcl


class HandBaseCollision:
    def __init__(self,model,urdf,top,radius=.025,height=.02):
        self.model=model
        self.top=np.asarray(top,dtype=float)
        if self.top.shape!=(3,) or not np.isfinite(self.top).all() or radius<=0 or height<=0:
            raise ValueError('Invalid base cylinder')
        self.base=fcl.CollisionObject(fcl.Cylinder(radius,height),
            fcl.Transform(self.top-np.array([0.,0.,height/2])))
        self.base_center=self.top-np.array([0.,0.,height/2])
        self.base_radius=float(radius);self.base_half_height=float(height)/2
        self.request=fcl.CollisionRequest()
        result=subprocess.run([str(ROOT/'.venv-cad/bin/python'),str(ROOT/'scripts/task_hook_mesh_export.py'),str(urdf)],
                              capture_output=True,text=True,check=True,timeout=60)
        all_shapes=[];self.cache={};self.check_count=0;self.narrow_check_count=0
        for shape in json.loads(result.stdout):
            kind=shape['kind']
            if kind=='convex':
                vertices=np.asarray(shape['vertices'],dtype=float)
                faces=np.asarray(shape['faces'],dtype=np.int32)
                encoded=np.column_stack((np.full(len(faces),3),faces)).ravel()
                geom=fcl.Convex(vertices,len(faces),encoded)
                center=(vertices.min(axis=0)+vertices.max(axis=0))/2
                bound=float(np.linalg.norm(vertices-center,axis=1).max())
            elif kind=='cylinder':
                p=shape['parameters'];radius_value=float(p['radius']);length=float(p['length'])
                geom=fcl.Cylinder(radius_value,length);center=np.zeros(3)
                bound=math.hypot(radius_value,length/2)
            elif kind=='box':
                size=np.asarray(list(map(float,shape['parameters']['size'].split())))
                geom=fcl.Box(*size);center=np.zeros(3);bound=float(np.linalg.norm(size)/2)
            elif kind=='sphere':
                bound=float(shape['parameters']['radius']);geom=fcl.Sphere(bound);center=np.zeros(3)
            else:raise ValueError('Unsupported hand shape: '+kind)
            local=np.eye(4);local[:3,:3]=Rotation.from_euler('xyz',shape['rpy']).as_matrix();local[:3,3]=shape['xyz']
            all_shapes.append((shape['link'],fcl.CollisionObject(geom),local,center,bound))
        self.body_shapes=[s for s in all_shapes if not s[0].startswith(('left_','right_'))]
        self.arm_shapes=[s for s in all_shapes if any(s[0].endswith(p) for p in (
            'elbow_link','wrist_roll_link','wrist_pitch_link','shoulder_yaw_link'))]
        body_ids={id(s) for s in self.body_shapes};arm_ids={id(s) for s in self.arm_shapes}
        self.shapes=[s for s in all_shapes if id(s) not in body_ids and id(s) not in arm_ids]
        self.arm_by_side={side:[s for s in self.arm_shapes if s[0].startswith(side+'_')]
                          for side in ('left','right')}
        self.hand_by_side={side:[s for s in self.shapes if s[0].startswith(side+'_')]
                           for side in ('left','right')}
        self.links=sorted({shape[0] for shape in self.shapes})

    def frames(self,links,q):
        links=list(dict.fromkeys(links))
        transform_many=getattr(self.model,'transforms',None)
        if callable(transform_many):
            frames=transform_many(links,q)
            if isinstance(frames,Mapping) and all(link in frames for link in links):return frames
        return {link:self.model.transform(link,q) for link in links}

    def conservative_hand_shapes(self, side):
        """Enclose all finger angles, without substituting a measured posture."""
        key=('all_finger_angles',side)
        if key in self.cache:return self.cache[key]
        root=side+'_wrist_yaw_link';result=[]
        for name,obj,origin,center,radius in self.hand_by_side[side]:
            chain=[];link=name
            while link!=root:
                if link not in self.model.joints:raise ValueError('Hand is not below wrist: '+name)
                parent,kind,local,*_=self.model.joints[link]
                if kind not in ('fixed','revolute','continuous'):
                    raise ValueError('Unsupported conservative hand joint: '+kind)
                chain.append((kind,local));link=parent
            chain.reverse();fixed=np.eye(4)
            for index,(kind,local) in enumerate(chain):
                fixed=fixed@local
                if kind!='fixed':
                    # Rotations preserve length; the triangle inequality covers every angle.
                    bound=(sum(float(np.linalg.norm(t[:3,3])) for _,t in chain[index+1:])+
                           float(np.linalg.norm(origin[:3,:3]@center+origin[:3,3]))+radius+.003)
                    local_sphere=np.eye(4);local_sphere[:3,3]=fixed[:3,3]
                    result.append((name,fcl.CollisionObject(fcl.Sphere(bound)),local_sphere,bound))
                    break
            else:
                result.append((name,obj,fixed@origin,None))
        self.cache[key]=result
        return result

    def check_conservative_hands(self,q):
        placed={}
        for side in ('left','right'):
            wrist=self.model.transform(side+'_wrist_yaw_link',q);placed[side]=[]
            for name,obj,local,_ in self.conservative_hand_shapes(side):
                transform=wrist@local
                obj.setTransform(fcl.Transform(transform[:3,:3],transform[:3,3]))
                placed[side].append((name,obj))
        body=list(self.body_objects(q,self.body_shapes))
        arms={s:list(self.body_objects(q,self.arm_by_side[s])) for s in placed}
        for side,hand in placed.items():
            other='right' if side=='left' else 'left'
            own_proximal=[item for item in arms[side] if item[0].endswith(('elbow_link','shoulder_yaw_link'))]
            obstacles=[('task_base',self.base)]+body+arms[other]+own_proximal
            for name,obj in hand:
                for other_name,obstacle in obstacles:
                    if self.collides(obj,obstacle):return name,other_name
        for name,obj in placed['left']:
            for other,obstacle in placed['right']:
                if self.collides(obj,obstacle):return name,other
        return None

    def body_objects(self,q,shapes):
        frames=self.frames((shape[0] for shape in shapes),q)
        for name,obj,origin,_,_ in shapes:
            transform=frames[name]@origin
            obj.setTransform(fcl.Transform(transform[:3,:3],transform[:3,3]))
            yield name,obj

    def collides(self,first,second):
        self.narrow_check_count+=1
        return bool(fcl.collide(first,second,self.request,fcl.CollisionResult()))

    def check_arm_body(self,q,side,frames=None):
        arm=self.arm_by_side[side];shapes=self.body_shapes+arm
        if frames is None:frames=self.frames((shape[0] for shape in shapes),q)
        placed=[]
        for name,obj,origin,center,radius in shapes:
            transform=frames[name]@origin
            world_center=transform[:3,:3]@center+transform[:3,3]
            placed.append((name,obj,transform,world_center,radius))
        body=placed[:len(self.body_shapes)];moving=placed[len(self.body_shapes):]
        for name,obj,transform,center,radius in moving:
            candidates=[]
            for other,obstacle,other_transform,other_center,other_radius in body:
                if np.dot(center-other_center,center-other_center) <= (radius+other_radius)**2:
                    candidates.append((other,obstacle,other_transform))
            if not candidates:continue
            obj.setTransform(fcl.Transform(transform[:3,:3],transform[:3,3]))
            for other,obstacle,other_transform in candidates:
                obstacle.setTransform(fcl.Transform(other_transform[:3,:3],other_transform[:3,3]))
                if self.collides(obj,obstacle):return name,other
        return None

    def arm_body_clearance(self,q,side):
        body=list(self.body_objects(q,self.body_shapes))
        arm=[s for s in self.arm_shapes if s[0].startswith(side+'_')]
        return min((float(fcl.distance(obj,obstacle)),name,other)
                   for name,obj in self.body_objects(q,arm) for other,obstacle in body)

    def hand_key(self,q,side):
        from task_hand_control import ORDER
        return (side,)+tuple(q[side+'_'+name+'_joint'] for name in ORDER)

    def hand_local(self,q,side,wrist,frames=None):
        key=self.hand_key(q,side)
        if key not in self.cache:
            inverse=np.linalg.inv(wrist)
            shapes=self.hand_by_side[side]
            if frames is None:frames=self.frames((shape[0] for shape in shapes),q)
            local=[]
            for name,obj,origin,center,radius in shapes:
                transform=inverse@frames[name]@origin
                local_center=transform[:3,:3]@center+transform[:3,3]
                local.append((name,obj,transform,local_center,radius))
            if len(self.cache)>256:self.cache.clear()
            self.cache[key]=local
        return self.cache[key]

    def objects(self,q,side):
        wrist=self.model.transform(side+'_wrist_yaw_link',q)
        local_shapes=self.hand_local(q,side,wrist)
        for name,obj,local,_,_ in local_shapes:
            transform=wrist@local
            obj.setTransform(fcl.Transform(transform[:3,:3],transform[:3,3]))
            yield name,obj

    def check(self,q,side,frames=None):
        self.check_count+=1
        wrist=(frames[side+'_wrist_yaw_link'] if frames is not None else
               self.model.transform(side+'_wrist_yaw_link',q))
        local=self.hand_local(q,side,wrist,frames)
        centers=np.asarray([item[3] for item in local])@wrist[:3,:3].T+wrist[:3,3]
        delta=centers-self.base_center
        radial=np.maximum(np.linalg.norm(delta[:,:2],axis=1)-self.base_radius,0.)
        vertical=np.maximum(np.abs(delta[:,2])-self.base_half_height,0.)
        candidates=radial*radial+vertical*vertical <= np.square([item[4] for item in local])
        for candidate,(name,obj,local_transform,_,_) in zip(candidates,local):
            if not candidate:continue
            transform=wrist@local_transform
            obj.setTransform(fcl.Transform(transform[:3,:3],transform[:3,3]))
            if self.collides(obj,self.base):return name
        return None

    def check_state(self,q,side):
        key=self.hand_key(q,side)
        shapes=self.body_shapes+self.arm_by_side[side]
        links=[side+'_wrist_yaw_link']+[shape[0] for shape in shapes]
        if key not in self.cache:links.extend(shape[0] for shape in self.hand_by_side[side])
        frames=self.frames(links,q)
        hand=self.check(q,side,frames)
        if hand:return hand,None
        return None,self.check_arm_body(q,side,frames)

    def check_batch(self,q,side,joint_names,positions,progress=None,check_cancel=None):
        """Return the first exact collision while vectorizing FK and broad phase."""
        positions=np.asarray(positions,dtype=float);count=len(positions)
        key=self.hand_key(q,side);shapes=self.body_shapes+self.arm_by_side[side]
        links=[side+'_wrist_yaw_link']+[shape[0] for shape in shapes]
        frames=self.model.batch_transforms(links,q,joint_names,positions)
        wrists=frames[side+'_wrist_yaw_link']
        if key not in self.cache:
            first=dict(q);first.update(zip(joint_names,map(float,positions[0])))
            self.hand_local(first,side,wrists[0])
        hand=self.cache[key]
        local_centers=np.asarray([item[3] for item in hand])
        hand_centers=(np.einsum('nij,hj->nhi',wrists[:,:3,:3],local_centers)+
                      wrists[:,:3,3][:,None,:])
        delta=hand_centers-self.base_center
        radial=np.maximum(np.linalg.norm(delta[:,:,:2],axis=2)-self.base_radius,0.)
        vertical=np.maximum(np.abs(delta[:,:,2])-self.base_half_height,0.)
        hand_candidates=radial*radial+vertical*vertical <= np.square([item[4] for item in hand])[None,:]

        placed=[]
        for name,obj,origin,center,radius in shapes:
            transform=np.matmul(frames[name],origin)
            world_center=np.einsum('nij,j->ni',transform[:,:3,:3],center)+transform[:,:3,3]
            placed.append((name,obj,transform,world_center,radius))
        body=placed[:len(self.body_shapes)];moving=placed[len(self.body_shapes):]
        arm_candidates=[]
        for _,_,_,center,radius in moving:
            other_centers=np.stack([item[3] for item in body],axis=1)
            distances=np.square(center[:,None,:]-other_centers).sum(axis=2)
            arm_candidates.append(distances <= np.square([radius+item[4] for item in body])[None,:])

        self.check_count+=count
        for sample in range(count):
            if check_cancel is not None and sample%128==0:check_cancel()
            if progress is not None and (sample%100==99 or sample+1==count):progress(sample+1)
            for index in np.flatnonzero(hand_candidates[sample]):
                name,obj,local_transform,_,_=hand[int(index)]
                transform=wrists[sample]@local_transform
                obj.setTransform(fcl.Transform(transform[:3,:3],transform[:3,3]))
                if self.collides(obj,self.base):
                    return {'sample_index':sample,'kind':'hand','link':name},wrists
            for arm_index,candidates in enumerate(arm_candidates):
                name,obj,transform,_,_=moving[arm_index]
                body_indices=np.flatnonzero(candidates[sample])
                if not len(body_indices):continue
                current=transform[sample]
                obj.setTransform(fcl.Transform(current[:3,:3],current[:3,3]))
                for body_index in body_indices:
                    other,obstacle,other_transform,_,_=body[int(body_index)]
                    obstacle_current=other_transform[sample]
                    obstacle.setTransform(fcl.Transform(obstacle_current[:3,:3],obstacle_current[:3,3]))
                    if self.collides(obj,obstacle):
                        return {'sample_index':sample,'kind':'arm_body','pair':(name,other)},wrists
        return None,wrists

    def clearance(self,q,side):
        return min((float(fcl.distance(obj,self.base)),name) for name,obj in self.objects(q,side))
