"""Isolated RViz/Qt playback of immutable archives or preview reports. No control clients."""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import cv2
import yaml
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import JointState,PointCloud2
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster,Buffer,TransformListener
from PyQt5 import QtCore,QtGui,QtWidgets
from task_preview_playback import PreviewTimeline
from task_preview_markers import target_markers,scoped_target_markers
from vision_task_preview import load_cloud
from workbench_model import OfflineModel
from task_kinematics import capture_transform

ROOT=Path(__file__).resolve().parents[1]


class Gallery(QtWidgets.QWidget):
    def __init__(self,node,reports,output,*,defer_scene=False):
        super().__init__();self.node=node;self.reports=reports;self.output=output
        self.source_reports=copy.deepcopy(reports)
        self.defer_scene=defer_scene
        self.scene_published=False
        self.elapsed=0.;self.last=time.monotonic();self.playing=True;self.last_status=0.
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state=node.create_publisher(JointState,'/joint_states',10)
        self.base_tf=StaticTransformBroadcaster(node)
        self.cloud=node.create_publisher(PointCloud2,'/g1/vision_rgbd_surface',qos)
        self.markers=node.create_publisher(MarkerArray,'/g1/vision_task_preview',qos)
        self.setWindowTitle('D-ring offline replay');self.resize(510,790)
        layout=QtWidgets.QVBoxLayout(self)
        self.badge=QtWidgets.QLabel()
        self.badge.setStyleSheet('color:#a44800;font-weight:bold;padding:8px;background:#fff0cf;')
        layout.addWidget(self.badge)
        self.capture=QtWidgets.QComboBox()
        for path,report in reports:
            self.capture.addItem(Path(report['capture']).name+' / '+report['arm']+
                                 f' / +{100*report.get("robot_raise_m",0):g} cm'+
                                 (' / curved index' if report.get('mode')=='HOOK_PREVIEW_BASE_ONLY' else
                                  ' / straight index' if report.get('mode')=='STRAIGHT_PREVIEW_SCOPED' else ''))
        layout.addWidget(self.capture)
        row=QtWidgets.QHBoxLayout();layout.addLayout(row)
        def button(icon,tip,fn):
            b=QtWidgets.QToolButton();b.setIcon(self.style().standardIcon(icon));b.setToolTip(tip)
            b.setFixedSize(38,34);b.clicked.connect(fn);row.addWidget(b);return b
        button(QtWidgets.QStyle.SP_MediaSkipBackward,'Previous capture',lambda:self.shift(-1))
        self.play=button(QtWidgets.QStyle.SP_MediaPause,'Pause / play',self.toggle)
        button(QtWidgets.QStyle.SP_MediaSkipForward,'Next capture',lambda:self.shift(1))
        button(QtWidgets.QStyle.SP_MediaSeekBackward,'Restart trajectory',self.restart)
        self.speed=QtWidgets.QComboBox()
        for value in (.5,1.,2.,4.,8.):self.speed.addItem(str(value)+'x',value)
        self.speed.setCurrentIndex(1 if any(r.get('speed_scaling') for _,r in reports) else 3);row.addWidget(self.speed)
        self.speed.setToolTip('Viewing speed only; does not modify trajectory limits')
        timing_row=QtWidgets.QHBoxLayout();layout.addLayout(timing_row)
        from task_speed_control import SpeedInput
        self.trajectory_speed=SpeedInput()
        timing_row.addWidget(QtWidgets.QLabel('Trajectory speed (x)'))
        timing_row.addWidget(self.trajectory_speed)
        self.apply_speed=QtWidgets.QToolButton()
        self.apply_speed.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_BrowserReload))
        self.apply_speed.setToolTip('Apply requested trajectory speed; RViz only')
        self.apply_speed.clicked.connect(self.retime)
        self.trajectory_speed.returnPressed.connect(self.retime)
        timing_row.addWidget(self.apply_speed);timing_row.addStretch(1)
        self.autonext=QtWidgets.QCheckBox('Cycle captures');self.autonext.setChecked(True);row.addWidget(self.autonext)
        self.seek=QtWidgets.QSlider(QtCore.Qt.Horizontal);self.seek.setRange(0,10000)
        self.seek.sliderMoved.connect(self.scrub);layout.addWidget(self.seek)
        self.stage=QtWidgets.QComboBox();layout.addWidget(self.stage)
        self.stage.currentIndexChanged.connect(self.select_stage)
        self.phase=QtWidgets.QLabel();layout.addWidget(self.phase)
        self.rgb=QtWidgets.QLabel();self.depth=QtWidgets.QLabel()
        for label,title in ((self.rgb,'Scene RGB'),(self.depth,'Scene depth')):
            layout.addWidget(QtWidgets.QLabel(title));label.setAlignment(QtCore.Qt.AlignCenter)
            label.setMinimumSize(440,230);label.setMaximumHeight(275);layout.addWidget(label)
        self.source=QtWidgets.QLabel();self.source.setWordWrap(True);layout.addWidget(self.source)
        self.capture.currentIndexChanged.connect(self.select)
        self.select(0)
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(33)

    def shift(self,delta):
        index=(self.capture.currentIndex()+delta)%len(self.reports)
        if index==self.capture.currentIndex():self.select(index)
        else:self.capture.setCurrentIndex(index)
    def restart(self):self.elapsed=getattr(self,'range_start',0.);self.last=time.monotonic()
    def toggle(self):
        self.playing=not self.playing;self.last=time.monotonic()
        self.play.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaPause if self.playing else QtWidgets.QStyle.SP_MediaPlay))
    def scrub(self,value):
        self.elapsed=self.range_start+(self.range_end-self.range_start)*value/10000.;self.last=time.monotonic()

    def select_stage(self,index):
        if index<=0:self.range_start,self.range_end=0.,self.timeline.duration
        else:self.range_start,self.range_end=self.timeline.parts[index-1][0],self.timeline.ends[index-1]
        self.restart()

    def select(self,index):
        path,report=self.reports[index];self.report=report
        hook=report.get('mode') in ('HOOK_PREVIEW_BASE_ONLY','STRAIGHT_PREVIEW_SCOPED','ARCHIVE_SCOPED_OFFLINE')
        scope='BASE + ARM/BODY' if report.get('arm_body_collision_checked') else ('HAND / BASE ONLY' if hook else 'COLLISION UNCHECKED')
        self.badge.setText('OFFLINE | '+scope+' | NO HARDWARE')
        start=json.loads((path.parent/'reconstructed_state.json').read_text())['joints']
        self.timeline=PreviewTimeline(report,start)
        self.stage.blockSignals(True);self.stage.clear();self.stage.addItem('All stages')
        for number,part in enumerate(self.timeline.parts,1):self.stage.addItem(f'{number}. {part[-1]}')
        self.stage.blockSignals(False);self.select_stage(0)
        overlay=report.get('scene_overlay')
        if overlay:
            self.badge.setText('RGB-D OVERLAY | NOT REVALIDATED | NO HARDWARE')
        capture=Path(overlay['capture'] if overlay else report['capture'])
        geometry_path=path.parent/'preview_geometry.json'
        geometry=json.loads((geometry_path if geometry_path.is_file() else capture/'geometry.json').read_text())
        model=OfflineModel(path.parent/'robot.urdf')
        scene_state=json.loads((capture/'capture_state.json').read_text())['joints'] if overlay else start
        torso=capture_transform(geometry,model,scene_state)
        transform=TransformStamped();transform.header.frame_id='preview_world';transform.child_frame_id='pelvis'
        transform.header.stamp=self.node.get_clock().now().to_msg()
        transform.transform.translation.z=float(report.get('robot_raise_m',0.))
        transform.transform.rotation.w=1.;self.base_tf.sendTransform(transform)
        cloud=load_cloud(capture,geometry)
        packed=np.frombuffer(bytes(cloud.data),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')]).copy()
        xyz=np.column_stack([packed[a] for a in 'xyz'])@torso[:3,:3].T+torso[:3,3]
        for i,axis in enumerate('xyz'):packed[axis]=xyz[:,i]
        cloud.data=packed.tobytes();cloud.header.frame_id='pelvis';cloud.header.stamp=self.node.get_clock().now().to_msg()
        markers=target_markers(geometry,torso,report['arm'],'task')
        if hook:
            scene=dict(report['scene'])
            if overlay:
                scene['base_top_pelvis_m']=(torso[:3,:3]@geometry['task_preview']['axis_center_m']+torso[:3,3]).tolist()
            markers=scoped_target_markers(geometry,torso,report['arm'],scene)
        self.pending_scene=(cloud,markers)
        if not self.defer_scene:self.publish_scene()
        self.rgb.setPixmap(QtGui.QPixmap(str(capture/'rgb.png')).scaled(480,270,QtCore.Qt.KeepAspectRatio,QtCore.Qt.SmoothTransformation))
        depth=cv2.imread(str(capture/'depth_aligned.png'),cv2.IMREAD_UNCHANGED)
        valid=depth>0;lo,hi=np.percentile(depth[valid],[5,95])
        normalized=np.clip((depth.astype(float)-lo)/max(1.,hi-lo)*255,0,255).astype(np.uint8)
        rgb=cv2.cvtColor(cv2.applyColorMap(normalized,cv2.COLORMAP_TURBO),cv2.COLOR_BGR2RGB);rgb[~valid]=0
        image=QtGui.QImage(rgb.data,rgb.shape[1],rgb.shape[0],rgb.strides[0],QtGui.QImage.Format_RGB888).copy()
        self.depth.setPixmap(QtGui.QPixmap.fromImage(image).scaled(480,270,QtCore.Qt.KeepAspectRatio,QtCore.Qt.SmoothTransformation))
        scaling=report.get('speed_scaling')
        guarded=report.get('timing_policy')=='guarded'
        if guarded:self.speed.setCurrentIndex(1)
        self.speed.setEnabled(not guarded)
        self.trajectory_speed.setValue(scaling['requested_multiplier'] if scaling else 1.)
        speed_note=(f'Trajectory {scaling["applied_multiplier"]:.2f}x / requested {scaling["requested_multiplier"]:.2f}x; '
                    f'{scaling["duration_after_s"]:.1f} s' if scaling else '')
        self.source.setText(report['state_note']+'\n'+speed_note+'\n'+str(capture))
        if scaling and scaling.get('joint_peaks'):
            self.source.setToolTip(json.dumps(scaling['joint_peaks'],indent=2))

    def retime(self):
        try:
            from rosidl_runtime_py.set_message import set_message_fields
            from rosidl_runtime_py.convert import message_to_ordereddict
            from trajectory_msgs.msg import JointTrajectory
            from task_trajectory_speed import preview_segments,restore_baseline
            index=self.capture.currentIndex();path,original=self.source_reports[index]
            multiplier=self.trajectory_speed.value()
            old=original.get('speed_scaling',{})
            segments=[]
            for record in original['segments']:
                trajectory=JointTrajectory();set_message_fields(trajectory,copy.deepcopy(record['trajectory']))
                segments.append(dict(record,trajectory=trajectory))
            segments=restore_baseline(segments,old)
            if original.get('timing_policy')=='guarded':
                from task_trajectory_speed import scale_arm_segments
                from task_planning_config import parameters_from_values
                from workbench_model import OfflineModel
                scaled,report=scale_arm_segments(segments,multiplier,OfflineModel(path.parent/'robot.urdf'),
                    parameters_from_values(original['motion_parameters']),
                    velocity=original['velocity_limit_scaling'],acceleration=original['acceleration_limit_scaling'])
            else:
                scaled,report=preview_segments(segments,multiplier)
            updated=dict(original,speed_scaling=report,
                segments=[dict(s,trajectory=message_to_ordereddict(s['trajectory'])) for s in scaled])
            self.reports[index]=(path,updated)
            from task_plan_archive import write_json
            write_json(self.output/f'retimed_preview_{index:03d}.json',updated)
            self.speed.setCurrentIndex(1)
            self.select(index)
            self.playing=True
            self.play.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaPause))
        except Exception as error:
            self.phase.setText(str(error))

    def publish_scene(self):
        cloud,markers=self.pending_scene
        cloud.header.stamp=self.node.get_clock().now().to_msg()
        self.cloud.publish(cloud);self.markers.publish(markers)
        self.scene_published=True

    def tick(self):
        now=time.monotonic();dt=now-self.last;self.last=now
        if self.playing and not self.seek.isSliderDown():self.elapsed+=dt*self.speed.currentData()
        if self.elapsed>self.range_end+1.:
            if self.autonext.isChecked() and self.stage.currentIndex()==0:self.shift(1)
            else:self.restart()
        q,label=self.timeline.sample(min(self.elapsed,self.range_end))
        if self.stage.currentIndex()>0:label=self.timeline.parts[self.stage.currentIndex()-1][-1]
        msg=JointState();msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.name=list(q);msg.position=[float(q[n]) for n in q];self.state.publish(msg)
        self.phase.setText(f'{label}    {min(self.elapsed,self.timeline.duration):.1f} / {self.timeline.duration:.1f} s')
        if not self.seek.isSliderDown():
            self.seek.setValue(min(10000,round(10000*(self.elapsed-self.range_start)/(self.range_end-self.range_start))))
        # Joint-state and TF subscriptions can otherwise outpace one callback per frame.
        for _ in range(4):rclpy.spin_once(self.node,timeout_sec=0.)
        if now-self.last_status>1:
            self.last_status=now
            if self.scene_published:self.publish_scene()
            (self.output/'playback_status.json').write_text(json.dumps({'offline_only':True,'hardware_commands_sent':0,
                'collision_checked':False,'capture':self.report['capture'],'phase':label,
                'base_collision_checked':self.report.get('base_collision_checked',False),
                'robot_raise_m':self.report.get('robot_raise_m',0.),
                'scene_overlay':self.report.get('scene_overlay'),'speed_scaling':self.report.get('speed_scaling'),
                'time_s':self.elapsed,'playing':self.playing,'joints':q},indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    source=parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--reports',type=Path,nargs='+')
    source.add_argument('--archives',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--ros-domain-id',type=int,choices=range(93,100),default=94)
    parser.add_argument('--scene-capture',type=Path,help='Separate captured RGB-D overlay; never execution validation')
    parser.add_argument('--scene-model',type=Path,help='Require this live model to match the immutable archive')
    parser.add_argument('--trajectory-speed',type=float,default=1.)
    parser.add_argument('--timing-policy',choices=('preview','guarded'),default='preview')
    parser.add_argument('--motion-parameters',type=Path,help='Panel motion-limit snapshot for guarded playback')
    parser.add_argument('--hand-settings',type=Path,help='Next-run finger settings, applied only to the playback copy')
    parser.add_argument('--velocity-limit-scaling',type=float,default=1.)
    parser.add_argument('--acceleration-limit-scaling',type=float,default=1.)
    parser.add_argument('--smoke-seconds',type=float,help='Exit automatically after a bounded visual smoke test')
    parser.add_argument('--smoke-stages',action='store_true',help='Exercise each stage and capture the scoped RViz window')
    parser.add_argument('--smoke-renderer',type=Path,help='Test-only RViz native framebuffer capture executable')
    args=parser.parse_args()
    from task_offline_replay import require_offline_environment
    require_offline_environment(args.ros_domain_id)
    if args.smoke_seconds is not None and (not np.isfinite(args.smoke_seconds) or args.smoke_seconds<3):
        parser.error('--smoke-seconds must be finite and at least 3')
    if args.smoke_stages and (args.smoke_seconds is None or args.smoke_seconds<20):
        parser.error('--smoke-stages requires --smoke-seconds >= 20 and an X11 display')
    if args.smoke_renderer and not args.smoke_stages:
        parser.error('--smoke-renderer requires --smoke-stages')
    if args.archives:
        from task_trajectory_library import playback_copy,require_compatible_models
        reports=[playback_copy(path,args.output/f'archive_{index:03d}',speed_multiplier=args.trajectory_speed,
            velocity=args.velocity_limit_scaling,acceleration=args.acceleration_limit_scaling,
            timing_policy=args.timing_policy,
            motion_parameters=json.loads(args.motion_parameters.read_text()) if args.motion_parameters else None,
            hand_settings=json.loads(args.hand_settings.read_text()) if args.hand_settings else None,
            scene_capture=args.scene_capture,scene_model=args.scene_model) for index,path in enumerate(args.archives)]
    else:
        if (args.scene_capture or args.scene_model or args.trajectory_speed!=1. or
                args.velocity_limit_scaling!=1. or args.acceleration_limit_scaling!=1.):
            parser.error('Scene overlays and trajectory scaling require verified archives')
        reports=[(p,json.loads(p.read_text())) for p in args.reports]
    for path,report in reports:PreviewTimeline(report,json.loads((path.parent/'reconstructed_state.json').read_text())['joints'])
    urdf=(reports[0][0].parent/'robot.urdf').read_text()
    if args.archives:
        require_compatible_models(reports)
    elif any((p.parent/'robot.urdf').read_text()!=urdf for p,_ in reports):raise ValueError('Gallery model mismatch')
    args.output.mkdir(parents=True,exist_ok=True)
    config=yaml.safe_load((ROOT/'assets/camera_reference/real_task.rviz').read_text())
    config['Visualization Manager']['Displays']=config['Visualization Manager']['Displays'][:3]
    config['Visualization Manager']['Displays'][0]['Name']='OFFLINE simulated playback - NOT live'
    config['Visualization Manager']['Displays'][1]['Name']='SCENE RGB-D - visualization only'
    config['Visualization Manager']['Global Options']['Fixed Frame']='preview_world'
    config['Visualization Manager']['Views']['Current'].update(
        **{'Target Frame':'pelvis','Distance':2.2,'Pitch':.45,'Focal Point':dict(X=.2,Y=0.,Z=-.05)})
    config['Visualization Manager']['Displays'][1]['Alpha']=.65
    config['Window Geometry'].update(Width=1200,Height=850,X=0,Y=30)
    rviz=args.output/'gallery.rviz';rviz.write_text(yaml.safe_dump(config,sort_keys=False))
    params=args.output/'robot.yaml';params.write_text(yaml.safe_dump({'/**':{'ros__parameters':{'robot_description':urdf}}}))
    rclpy.init();node_name=f'g1_offline_gallery_{os.getpid()}';node=rclpy.create_node(node_name);time.sleep(.5)
    if any(name!=node_name for name,_ in node.get_node_names_and_namespaces()):
        raise RuntimeError(f'Playback domain {args.ros_domain_id} is already in use')
    app=QtWidgets.QApplication([]);children=[]
    with (args.output/'rviz.log').open('w') as log:
        try:
            children.append(subprocess.Popen(['ros2','run','robot_state_publisher','robot_state_publisher',
                '--ros-args','--params-file',str(params)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
            render_command=([str(args.smoke_renderer),str(rviz),str(args.output)] if args.smoke_renderer
                            else ['rviz2','-d',str(rviz)])
            children.append(subprocess.Popen(render_command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
            gallery=Gallery(node,reports,args.output,defer_scene=args.smoke_stages);gallery.move(1210,30);gallery.show()
            evidence=dict(stages=[],screenshots=[],errors=[],joint_messages=0,max_joint_change_rad=0.)
            last_joint={}
            def observe(msg):
                nonlocal last_joint
                state=dict(zip(msg.name,msg.position))
                if last_joint:
                    change=max(abs(value-last_joint.get(name,value)) for name,value in state.items())
                    evidence['max_joint_change_rad']=max(evidence['max_joint_change_rad'],change)
                last_joint=state;evidence['joint_messages']+=1
            if args.smoke_stages:
                from task_gallery_smoke import (scoped_rviz_window,render_region,viewport_metrics,
                                                visible_viewport,changed_fraction,window_geometry)
                subscription=node.create_subscription(JointState,'/joint_states',observe,10)
                buffer=Buffer();listener=TransformListener(buffer,node)
                began=time.monotonic();probe_state=dict(next_stage=1,selected_at=None,ready=False)
                frames=[];gallery.autonext.setChecked(False)
                native_request=0
                def capture_viewport():
                    nonlocal native_request
                    if args.smoke_renderer:
                        native_request+=1
                        (args.output/'native_request.txt').write_text(str(native_request))
                        deadline=time.monotonic()+3.
                        while time.monotonic()<deadline:
                            ready=args.output/'native_ready.txt'
                            if ready.is_file() and ready.read_text().strip()==str(native_request):break
                            time.sleep(.02)
                        else:raise ValueError('Native RViz framebuffer capture timed out')
                        image=cv2.imread(str(args.output/'native_frame.png'))
                        if image is None:raise ValueError('Missing native RViz framebuffer')
                        pix=QtGui.QPixmap.fromImage(QtGui.QImage(str(args.output/'native_frame.png')))
                        return dict(id=None),(0,0,image.shape[1],image.shape[0]),pix,image
                    tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True,timeout=2)
                    window=scoped_rviz_window(tree,rviz)
                    children=subprocess.check_output(['xwininfo','-id',hex(window['id']),'-tree'],text=True,timeout=2)
                    region=render_region(children,window)
                    candidates=[item for line in children.splitlines() if (item:=window_geometry(line)) is not None
                                and item['id']!=window['id'] and (item['width'],item['height'])==region[2:]]
                    if len(candidates)!=1:raise ValueError('Unique RViz render child not found')
                    pix=app.primaryScreen().grabWindow(candidates[0]['id'])
                    if pix.isNull():raise ValueError('RViz viewport capture failed')
                    image=pix.toImage().convertToFormat(QtGui.QImage.Format_RGB888)
                    data=np.frombuffer(image.bits().asstring(image.byteCount()),dtype=np.uint8)
                    data=data.reshape(image.height(),image.bytesPerLine())[:,:image.width()*3]
                    bgr=data.reshape(image.height(),image.width(),3)[:,:,::-1].copy()
                    return window,region,pix,bgr
                def probe():
                    try:
                        if any(child.poll() is not None for child in children):
                            raise RuntimeError('Offline visualization child exited before smoke completion')
                        required=('pelvis','torso_link',gallery.report['arm']+'_index_tcp')
                        missing=[link for link in required if not buffer.can_transform('preview_world',link,rclpy.time.Time())]
                        evidence['missing_tf_frames']=missing
                        tf_ready=not missing
                        if not probe_state['ready']:
                            if not tf_ready or evidence['joint_messages']<=20:return
                            try:window,region,pix,image=capture_viewport()
                            except (ValueError,subprocess.SubprocessError) as error:
                                evidence['last_readiness_error']=str(error);return
                            evidence['last_readiness_viewport']=viewport_metrics(image)
                            if not visible_viewport(evidence['last_readiness_viewport']):return
                            if 'robot_settling_since' not in probe_state:
                                gallery.playing=False
                                probe_state['robot_settling_since']=time.monotonic()
                                return
                            if time.monotonic()-probe_state['robot_settling_since']<1.:return
                            if 'robot_only_frame' not in probe_state:
                                if not pix.save(str(args.output/'rviz_robot_only_viewport.png')):
                                    raise ValueError('Robot-only screenshot save failed')
                                probe_state['robot_only_frame']=image[::4,::4].copy()
                                evidence['robot_only_viewport']=viewport_metrics(image)
                                gallery.publish_scene();probe_state['scene_sent_at']=time.monotonic()
                                return
                            if time.monotonic()-probe_state['scene_sent_at']<1.:return
                            scene_change=changed_fraction(probe_state['robot_only_frame'],image[::4,::4])
                            if scene_change<=.003:return
                            evidence['scene_added_changed_fraction']=scene_change
                            evidence['ready_after_seconds']=time.monotonic()-began
                            evidence['required_tf_frames']=list(required)
                            probe_state['ready']=True;gallery.playing=False
                        index=probe_state['next_stage']
                        if probe_state['selected_at'] is None:
                            gallery.stage.setCurrentIndex(index);gallery.scrub(5000);gallery.tick()
                            probe_state['selected_at']=time.monotonic()
                            probe_state['expected_joints']=gallery.timeline.sample(gallery.elapsed)[0]
                            return
                        expected=probe_state['expected_joints']
                        matched=all(name in last_joint and abs(last_joint[name]-value)<1e-7
                                    for name,value in expected.items())
                        if time.monotonic()-probe_state['selected_at']<1. or not tf_ready or not matched:return
                        window,region,pix,image=capture_viewport()
                        metrics=viewport_metrics(image)
                        if not visible_viewport(metrics):raise ValueError('Blank RViz stage viewport')
                        path=args.output/f'rviz_stage_{index:02d}.png'
                        viewport=args.output/f'rviz_stage_{index:02d}_viewport.png'
                        full=pix if args.smoke_renderer else app.primaryScreen().grabWindow(window['id'])
                        if (not pix.save(str(viewport)) or not full.save(str(path))):
                            raise ValueError('RViz screenshot save failed')
                        frames.append(image[::4,::4].copy())
                        evidence['screenshots'].append(dict(path=str(path),viewport=str(viewport),
                            viewport_rectangle=list(region),stage=index,**metrics))
                        evidence['stages'].append(dict(index=index,label=gallery.stage.currentText(),
                            range_start=gallery.range_start,range_end=gallery.range_end,
                            published_state_matches=True,tf_ready=True))
                        probe_state.update(next_stage=index+1,selected_at=None)
                        if index==len(gallery.timeline.parts):
                            evidence['max_viewport_changed_fraction']=max(
                                changed_fraction(frames[0],frame) for frame in frames[1:]) if len(frames)>1 else 0.
                            probe_timer.stop();app.quit()
                    except Exception as error:
                        evidence['errors'].append(str(error));probe_timer.stop();app.quit()
                probe_timer=QtCore.QTimer(gallery);probe_timer.timeout.connect(probe);probe_timer.start(250)
            QtCore.QTimer.singleShot(2000,lambda:gallery.grab().save(str(args.output/'panel.png')))
            if args.smoke_seconds is not None:QtCore.QTimer.singleShot(round(args.smoke_seconds*1000),app.quit)
            signal.signal(signal.SIGTERM,lambda *_:app.quit());signal.signal(signal.SIGINT,lambda *_:app.quit())
            app.exec_()
            if args.smoke_stages:
                if len(evidence['stages'])!=len(gallery.timeline.parts):
                    evidence['errors'].append('Smoke deadline reached before all stage viewport/state checks completed')
                evidence['passed']=bool(not evidence['errors'] and len(evidence['stages'])==len(gallery.timeline.parts)
                    and len(evidence['screenshots'])==len(gallery.timeline.parts)
                    and all(visible_viewport(s) for s in evidence['screenshots'])
                    and evidence['joint_messages']>20 and evidence['max_joint_change_rad']>.01
                    and visible_viewport(evidence.get('robot_only_viewport',dict(spatial_pixel_std=0.,foreground_fraction=0.)))
                    and evidence.get('scene_added_changed_fraction',0.)>.003
                    and evidence.get('max_viewport_changed_fraction',0.)>.001)
                evidence.update(offline_only=True,hardware_commands_sent=0)
                (args.output/'smoke_checks.json').write_text(json.dumps(evidence,indent=2)+'\n')
                if not evidence['passed']:raise RuntimeError('Offline gallery smoke checks failed')
        finally:
            for child in children:
                if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
            for child in children:
                try:child.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
