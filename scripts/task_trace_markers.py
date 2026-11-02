"""Continuous paths with sparse, prominent fingertip orientation arrows."""
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

NAMESPACE = 'recorded_trace_editor'


def arrow_vertices(fingertips, endpoints, length):
    """Arrows rooted at the fingertip, pointing from tool0 toward the fingertip."""
    direction = fingertips-endpoints
    norm = np.linalg.norm(direction, axis=1)
    valid = norm > 1e-9
    direction = direction/np.maximum(norm[:, None], 1e-9)
    reference = np.repeat([[0., 0., 1.]], len(direction), axis=0)
    reference[np.abs(direction[:, 2]) > .9] = (0., 1., 0.)
    lateral = np.cross(direction, reference)
    lateral /= np.maximum(np.linalg.norm(lateral, axis=1)[:, None], 1e-9)
    tip = fingertips+length*direction
    back = tip-length*.25*direction
    vertices = np.stack((fingertips, tip, tip, back+length*.13*lateral,
                         tip, back-length*.13*lateral), axis=1)
    return vertices, valid


def solid_arrow_marker(origins, endpoints, length, marker_id, color, diameter=.0036):
    direction = origins-endpoints
    norms = np.linalg.norm(direction, axis=1)
    origins, direction, norms = origins[norms>1e-9], direction[norms>1e-9], norms[norms>1e-9]
    direction = direction/norms[:, None]
    reference = np.repeat([[0.,0.,1.]],len(direction),axis=0)
    reference[np.abs(direction[:,2])>.9] = (0.,1.,0.)
    u = np.cross(direction,reference)
    u /= np.maximum(np.linalg.norm(u,axis=1)[:,None],1e-9)
    v = np.cross(direction,u)
    angles = np.arange(8)*np.pi/4
    ring = np.cos(angles)[None,:,None]*u[:,None,:]+np.sin(angles)[None,:,None]*v[:,None,:]
    base = origins[:,None,:]+diameter*.5*ring
    neck = origins[:,None,:]+length*.7*direction[:,None,:]+diameter*.5*ring
    head = origins[:,None,:]+length*.7*direction[:,None,:]+diameter*(16/9)*ring
    tip = origins+length*direction
    triangles=[]
    for i in range(8):
        j=(i+1)%8
        for a,b,c in ((base[:,i],base[:,j],neck[:,j]),(base[:,i],neck[:,j],neck[:,i]),
                      (head[:,i],head[:,j],tip),(origins,base[:,j],base[:,i]),
                      (origins+length*.7*direction,head[:,j],head[:,i])):
            triangles.append(np.stack((a,b,c),axis=1))
    vertices=np.concatenate(triangles,axis=1).reshape(-1,3)
    item=Marker(ns=NAMESPACE,id=marker_id,type=Marker.TRIANGLE_LIST)
    item.header.frame_id='pelvis';item.pose.orientation.w=1.
    item.scale.x=item.scale.y=item.scale.z=1.
    item.color.r,item.color.g,item.color.b,item.color.a=color
    item.points=[Point(x=float(x),y=float(y),z=float(z)) for x,y,z in vertices]
    return item


def sparse_indices(pose, spacing=.02, maximum=40):
    if len(pose) == 0:return np.array([], dtype=int)
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(pose[:, :3, 3], axis=0), axis=1))]
    indices = [0]
    for i in range(1, len(pose)-1):
        if distance[i]-distance[indices[-1]] >= spacing:
            indices.append(i)
    if len(pose) > 1:indices.append(len(pose)-1)
    if len(indices) > maximum:
        indices = np.asarray(indices)[np.linspace(0, len(indices)-1, maximum).round().astype(int)]
    return np.asarray(indices, dtype=int)


def display_indices(count, selected, maximum=2000):
    # Keep interval boundaries exact while bounding visualization geometry only.
    indices = np.linspace(0, count-1, min(count, maximum)).round().astype(int)
    boundaries = np.flatnonzero(selected[1:] != selected[:-1])
    return np.unique(np.r_[indices, boundaries, boundaries+1])


def pick_points(model, trace, side, context, config, slot, spacing=.02, maximum=40, *, geometry=None):
    from task_dring_teach_data import moving
    if geometry is None:
        context = hand_context(trace, side, context, config, slot)
        rows = np.asarray(trace['raw']['positions'])
        pose = model.batch_transforms([side+'_index_tcp'], context, moving(side), rows)[side+'_index_tcp']
    else:rows,pose=geometry['rows'],geometry['transforms'][side+'_index_tcp']
    return [dict(position=pose[i, :3, 3], time=float(trace['raw']['time_s'][i]),
                 positions=rows[i], trace=trace, slot=slot)
            for i in sparse_indices(pose, spacing, maximum)]


def clear_markers():
    return MarkerArray(markers=[Marker(ns=NAMESPACE, id=i, action=Marker.DELETE) for i in range(20)])


def replace_markers(array):
    array.markers=[item for item in array.markers if item.points]
    active={item.id for item in array.markers}
    array.markers.extend(item for item in clear_markers().markers if item.id not in active)
    return array


def hand_context(trace,side,context,config,slot):
    from task_hand_control import model_joints, POSTURES
    from task_landing_gear_data import HAND_POLICY
    from task_rail_trace import STAGES
    context = dict(context)
    # No finger feedback was captured for arm-only demonstrations. State the
    # chosen hand model explicitly in the UI rather than claiming measured TCP.
    for hand in ('left', 'right'):
        context.update(model_joints(hand, POSTURES['index_only']))
    if config.get('hands'):
        posture = HAND_POLICY[slot][0] if slot in HAND_POLICY else STAGES[slot][1]
        state = config['hands'].get(posture)
        if state is not None:
            context.update(model_joints(side, state['feedback_q']))
    return context


def available_links(side,context):
    from task_session import ARMS
    return [hand+'_index_tcp' for hand in ('left','right')
            if hand==side or all(name in context for name in ARMS[hand])]


def trace_markers(model, trace, side, context, start, end, config, slot, key_times=(),
                  spacing=.02, maximum=40, arrow_length=.04, arrow_diameter=.0036,geometry=None):
    from task_dring_teach_data import moving
    if geometry is None:
        from task_trace_display_cache import TraceDisplayCache
        geometry=TraceDisplayCache(maximum_bytes=0).geometry(model,trace,side,context,config,slot)
    t,context=geometry['times'],geometry['context']
    links,endpoints,transforms=geometry['links'],geometry['endpoints'],geometry['transforms']
    result = MarkerArray()
    selected = (t >= start) & (t <= end) & (end > start)
    for hand_index, link in enumerate(links):
        pose = transforms[link]
        origin = pose[:, :3, 3]
        sparse = np.zeros(len(t), dtype=bool)
        sparse[sparse_indices(pose, spacing, maximum)] = True
        ends = transforms[endpoints[hand_index]][:, :3, 3]
        path_indices = display_indices(len(t), selected)
        for region, mask in enumerate((~selected, selected)):
            item = Marker(ns=NAMESPACE, id=2*hand_index+region, type=Marker.LINE_LIST)
            item.header.frame_id = 'pelvis'
            item.pose.orientation.w = 1.
            item.scale.x = .0015
            item.color.r, item.color.g, item.color.b, item.color.a = ((1., .85, 0., .85) if region == 0 else (1., .05, .05, 1.))
            a, b = path_indices[:-1], path_indices[1:]
            keep = mask[a] | mask[b]
            vertices = np.stack((origin[a[keep]], origin[b[keep]]), axis=1).reshape(-1, 3)
            item.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in vertices]
            result.markers.append(item)
            result.markers.append(solid_arrow_marker(origin[mask & sparse],ends[mask & sparse],arrow_length,
                12+2*hand_index+region,(1.,.85,0.,1.) if region==0 else (1.,.05,.05,1.),arrow_diameter))
    if key_times:
        from task_dring_trim import sample
        pose = model.batch_transforms([side+'_index_tcp'],context,moving(side),sample(trace,key_times))[side+'_index_tcp']
        item = Marker(ns=NAMESPACE,id=4,type=Marker.SPHERE_LIST)
        item.header.frame_id='pelvis';item.pose.orientation.w=1.
        item.scale.x=item.scale.y=item.scale.z=.008
        item.color.r,item.color.g,item.color.b,item.color.a=.1,1.,1.,1.
        item.points=[Point(x=float(x),y=float(y),z=float(z)) for x,y,z in pose[:,:3,3]]
        result.markers.append(item)
    return replace_markers(result)


def comparison_markers(result, spacing=.02, maximum=40, arrow_length=.04, only_differences=True, arrow_diameter=.0036):
    array = MarkerArray()
    for side_index,link in enumerate(result['first']):
        mask=result['differences'][link]
        for version,color in (('first',(1.,.85,0.,1.)),('second',(.05,.8,1.,1.))):
            pose=result[version][link];origin=pose[:,:3,3]
            indices=sparse_indices(pose,spacing,maximum)
            if only_differences:indices=indices[mask[indices]]
            edges=np.flatnonzero(mask[:-1]|mask[1:]) if only_differences else np.arange(len(origin)-1)
            vertices=np.stack((origin[edges],origin[edges+1]),axis=1).reshape(-1,3)
            item=Marker(ns=NAMESPACE,id=6+side_index*3+(version=='second'),type=Marker.LINE_LIST)
            item.header.frame_id='pelvis';item.pose.orientation.w=1.;item.scale.x=.003
            item.color.r,item.color.g,item.color.b,item.color.a=color
            item.points=[Point(x=float(x),y=float(y),z=float(z)) for x,y,z in vertices]
            array.markers.append(item)
            array.markers.append(solid_arrow_marker(origin[indices],result[version+'_endpoints'][link][indices,:3,3],
                arrow_length,12+side_index*2+(version=='second'),color,arrow_diameter))
        a=result['first'][link][:,:3,3];b=result['second'][link][:,:3,3]
        indices=np.flatnonzero(mask)
        if len(indices)>maximum:indices=indices[np.linspace(0,len(indices)-1,maximum).round().astype(int)]
        vertices=np.stack((a[indices],b[indices]),axis=1).reshape(-1,3)
        item=Marker(ns=NAMESPACE,id=8+side_index*3,type=Marker.LINE_LIST)
        item.header.frame_id='pelvis';item.pose.orientation.w=1.;item.scale.x=.0015
        item.color.r,item.color.g,item.color.b,item.color.a=1.,.15,.45,.85
        item.points=[Point(x=float(x),y=float(y),z=float(z)) for x,y,z in vertices]
        array.markers.append(item)
    return replace_markers(array)
