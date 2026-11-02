"""Task targets are coordinates, not a second robot/hand model."""
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def planning_tcp_offsets(model):
    from task_hand_control import POSTURES, model_joints
    result = {}
    for side in ('left', 'right'):
        state = model_joints(side, POSTURES['index_only'])
        result[side] = (np.linalg.inv(model.transform(side + '_wrist_yaw_link', state))
                        @ model.transform(side + '_index_tcp', state))
    return result


def live_tcp_markers(wrist_offsets=None):
    """The live model's link origins, not physical measurements or ghost poses."""
    array = MarkerArray()
    for side in ('left', 'right'):
        def make(id_, type_, color):
            item = Marker()
            item.header.frame_id = side + '_index_tcp'
            item.ns = 'live_model_index_tcp_' + side
            item.id, item.type = id_, type_
            item.frame_locked = True
            item.pose.orientation.w = 1.
            item.color.r, item.color.g, item.color.b, item.color.a = color
            array.markers.append(item)
            return item

        ball = make(0, Marker.SPHERE, (1., .05, .8, 1.))
        ball.scale.x = ball.scale.y = ball.scale.z = .006
        for axis, color in enumerate(((1., .1, .1, 1.), (.1, 1., .1, 1.), (.1, .4, 1., 1.))):
            arrow = make(axis + 1, Marker.ARROW, color)
            endpoint = Point()
            setattr(endpoint, 'xyz'[axis], .03)
            arrow.points = [Point(), endpoint]
            arrow.scale.x, arrow.scale.y, arrow.scale.z = .0015, .0035, .005
        label = make(4, Marker.TEXT_VIEW_FACING, (1., .7, .95, 1.))
        label.pose.position.z = .055
        label.scale.z = .012
        label.text = ('L' if side == 'left' else 'R') + ' index_tcp (model)'
        leader = make(5, Marker.LINE_STRIP, (1., .4, .85, .9))
        leader.points = [Point(), Point(z=.045)]
        leader.scale.x = .0008
        if wrist_offsets is not None:
            from scipy.spatial.transform import Rotation
            transform = wrist_offsets[side]
            rotation = Rotation.from_matrix(transform[:3, :3]).as_quat()
            label.text = ('L' if side == 'left' else 'R') + ' index_tcp (index_only)'
            for item in array.markers[-6:]:
                item.header.frame_id = side + '_wrist_yaw_link'
                position = item.pose.position
                p = transform[:3, :3] @ [position.x, position.y, position.z] + transform[:3, 3]
                item.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                q = item.pose.orientation
                q.x, q.y, q.z, q.w = map(float, rotation)
    return array


def with_live_tcp(array, wrist_offsets=None):
    array.markers.extend(live_tcp_markers(wrist_offsets).markers)
    return array


def target_markers(geometry, torso, side, kind):
    array = MarkerArray()
    clear = Marker(action=Marker.DELETEALL)
    array.markers.append(clear)
    if kind != 'task':
        return array
    if side not in ('left', 'right'):
        raise ValueError('Select one hand per D-ring task')

    def point(p):
        return Point(**dict(zip('xyz', (torso[:3,:3] @ np.asarray(p)+torso[:3,3]).tolist())))

    def marker(type_, color):
        item = Marker()
        item.header.frame_id = 'pelvis'
        item.ns = 'task_target'
        item.id = len(array.markers)
        item.type = type_
        item.pose.orientation.w = 1.
        item.color.r, item.color.g, item.color.b, item.color.a = color
        array.markers.append(item)
        return item

    task = geometry['task_preview']
    if geometry.get('center_check'):
        check=geometry['center_check']
        for key,color in (('center_pelvis_m',(1.,.2,.2,1.)),('target_pelvis_m',(.1,.9,.7,1.))):
            item=marker(Marker.SPHERE,color)
            item.pose.position=Point(**dict(zip('xyz',check[key])))
            item.scale.x=item.scale.y=item.scale.z=.01
        item=marker(Marker.LINE_STRIP,(.1,.9,.7,1.));item.scale.x=.003
        item.points=[Point(**dict(zip('xyz',check[k]))) for k in ('center_pelvis_m','target_pelvis_m')]
        return array
    for key, color in (('axis_center_m',(1.,.2,.2,1.)), ('insertion_reference_m',(.1,.9,.7,1.))):
        item = marker(Marker.SPHERE, color)
        item.pose.position = point(task[key])
        item.scale.x = item.scale.y = item.scale.z = .014
    if task.get('arc_center_toward_robot_m',0.):
        item = marker(Marker.SPHERE, (1.,.8,.1,1.))
        item.pose.position = point(task['arc_center_m'])
        item.scale.x = item.scale.y = item.scale.z = .012
    arc = marker(Marker.LINE_STRIP, (.1,.9,.7,1.))
    arc.scale.x = .004
    arc.points = [point(p) for p in task['arc_minus120_to_plus120_m']]
    tcp = np.asarray(geometry['grasp_candidates'][side]['poses']['index_tcp']['matrix_row_major'])
    for axis, color in enumerate(((1.,.2,.2,1.),(.2,1.,.2,1.),(.2,.5,1.,1.))):
        item = marker(Marker.ARROW, color)
        item.points = [point(tcp[:3,3]), point(tcp[:3,3]+tcp[:3,axis]*.06)]
        item.scale.x, item.scale.y, item.scale.z = .003, .007, .012
    return array


def scoped_target_markers(geometry,torso,side,scene):
    array=target_markers(geometry,torso,side,'task')
    array.markers=[m for m in array.markers if m.type!=Marker.ARROW or m.action==Marker.DELETEALL]
    base=Marker();base.header.frame_id='pelvis';base.ns='task_base';base.id=100
    base.type=Marker.CYLINDER;base.pose.orientation.w=1.
    top=scene['base_top_pelvis_m']
    base.pose.position.x,base.pose.position.y=map(float,top[:2])
    base.pose.position.z=float(top[2]-scene['base_height_m']/2)
    base.scale.x=base.scale.y=2*scene['base_radius_m'];base.scale.z=scene['base_height_m']
    base.color.r,base.color.g,base.color.b,base.color.a=.85,.25,.12,.85
    array.markers.append(base)
    return array
