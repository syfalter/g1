"""RViz button markers forward selection to Qt, never to an actuator."""
import threading
import math
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker


class TracePicker:
    def __init__(self, node):
        self.server = InteractiveMarkerServer(node, '/g1/task/trace_points')
        self.lock = threading.RLock()
        self.owner = None
        self.generation = 0
        self.move_generation = 0
        self.markers = {}
        self.move_signature = None

    def move_handle(self, owner, position, callback, diameter=.018):
        with self.lock:
            if self.owner is not owner:return
            self.move_generation += 1
            generation, token = self.generation, self.move_generation
            signature=(tuple(map(float,position)),float(diameter)) if position is not None else None
            def feedback(msg):
                if (msg.event_type == InteractiveMarkerFeedback.MOUSE_UP and self.owner is owner and
                        self.generation == generation and self.move_generation == token):
                    self.move_signature=None
                    p=msg.pose.position;callback([p.x,p.y,p.z])
            if signature is not None and signature==self.move_signature:
                self.server.setCallback('manual_move',feedback)
                self.server.applyChanges();return
            if self.move_signature is not None:self.server.erase('manual_move')
            self.move_signature=signature
            if position is not None:
                marker = InteractiveMarker()
                marker.header.frame_id = 'pelvis'; marker.name = 'manual_move'
                marker.description = 'XYZ'; marker.pose.orientation.w = 1.
                marker.pose.position.x,marker.pose.position.y,marker.pose.position.z = map(float,position)
                marker.scale = max(.08,diameter*3)
                sphere = Marker(type=Marker.SPHERE); sphere.pose.orientation.w = 1.
                sphere.scale.x=sphere.scale.y=sphere.scale.z=float(diameter)
                sphere.color.r,sphere.color.g,sphere.color.b,sphere.color.a = (1.,.25,.65,1.)
                visible = InteractiveMarkerControl(); visible.always_visible = True
                visible.markers.append(sphere);marker.controls.append(visible)
                for name,rotation in (('x',(0.,0.,0.,1.)),('y',(0.,0.,2**-.5,2**-.5)),
                                      ('z',(0.,-2**-.5,0.,2**-.5))):
                    axis = InteractiveMarkerControl();axis.name='move_'+name
                    axis.orientation.x,axis.orientation.y,axis.orientation.z,axis.orientation.w=rotation
                    axis.orientation_mode=InteractiveMarkerControl.FIXED
                    axis.interaction_mode=InteractiveMarkerControl.MOVE_AXIS
                    marker.controls.append(axis)
                self.server.insert(marker,feedback_callback=feedback)
            self.server.applyChanges()

    def clear(self, owner):
        with self.lock:self._clear(owner)

    def _clear(self, owner):
        if self.owner is not owner:return
        self.generation += 1
        self.owner = None
        self.markers={};self.move_signature=None
        self.server.clear(); self.server.applyChanges()

    def show(self, owner, points, callback, diameter=.018):
        with self.lock:self._show(owner, points, callback, diameter)

    def _show(self, owner, points, callback, diameter):
        if not math.isfinite(diameter) or diameter <= 0.:
            raise ValueError('Point diameter must be finite and positive')
        self.generation += 1
        generation = self.generation
        if self.owner is not owner:
            self.server.clear();self.markers={};self.move_signature=None
        self.owner = owner
        updated={}
        for i, point in enumerate(points):
            name=f'p_{id(point.get("trace",owner)):x}_{float(point["time"]).hex()}_{i}'
            color=point.get('color',(1.,.85,0.,1.))
            signature=(tuple(map(float,point['position'])),tuple(color),float(diameter))
            updated[name]=signature
            def feedback(msg, point=point):
                if (msg.event_type == InteractiveMarkerFeedback.BUTTON_CLICK and
                        self.owner is owner and self.generation == generation):
                    callback(point)
            if self.markers.get(name)==signature:
                self.server.setCallback(name,feedback)
                continue
            marker = InteractiveMarker()
            marker.header.frame_id = 'pelvis'
            marker.name = name
            # Per-point text creates hundreds of extra RViz objects. The panel
            # shows the selected point's label/time instead.
            marker.description = ''
            marker.pose.orientation.w = 1.
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, point['position'])
            marker.scale = max(.025, diameter)
            button = InteractiveMarkerControl()
            button.name = 'select'; button.interaction_mode = InteractiveMarkerControl.BUTTON
            button.always_visible = True
            sphere = Marker(type=Marker.SPHERE)
            sphere.pose.orientation.w = 1.
            sphere.scale.x = sphere.scale.y = sphere.scale.z = float(diameter)
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = color
            button.markers.append(sphere); marker.controls.append(button)

            self.server.insert(marker, feedback_callback=feedback)
        for name in self.markers.keys()-updated.keys():self.server.erase(name)
        self.markers=updated
        self.server.applyChanges()
