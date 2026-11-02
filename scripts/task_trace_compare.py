"""Geometry-only comparison of two versions; independent of execution state."""
import numpy as np
from scipy.spatial.transform import Rotation


def aligned_rows(trace, *, mode='path', reverse=False, count=401):
    rows = np.asarray(trace['raw']['positions'],dtype=float)
    times = np.asarray(trace['raw']['time_s'],dtype=float)
    if reverse:
        rows = rows[::-1]
        times = times[-1]-times[::-1]
    if mode == 'path':
        axis = np.r_[0.,np.cumsum(np.linalg.norm(np.diff(rows,axis=0),axis=1))]
    elif mode == 'time':
        axis = times-times[0]
    else:raise ValueError('Unknown comparison alignment')
    if axis[-1] <= 1e-12:return np.repeat(rows[:1],count,axis=0)
    keep = np.r_[True,np.diff(axis)>1e-12]
    progress = np.linspace(0.,axis[-1],count)
    result = np.column_stack([np.interp(progress,axis[keep],values[keep]) for values in rows.T])
    result[0],result[-1] = rows[0],rows[-1]
    return result


def differing_pick(trace, time, mask, mode='path', reverse=False):
    times=np.asarray(trace['raw']['time_s'])
    if mode=='path':
        rows=np.asarray(trace['raw']['positions'])
        progress=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(rows,axis=0),axis=1))]
        fraction=np.interp(time,times,progress)/progress[-1] if progress[-1]>1e-12 else np.zeros_like(time,dtype=float)
    else:fraction=np.asarray(time)/times[-1]
    if reverse:fraction=1.-fraction
    index=np.clip(fraction,0.,1.)*(len(mask)-1)
    return mask[np.floor(index).astype(int)] | mask[np.ceil(index).astype(int)]


def compare(model, first, second, side, context, *, mode='path', reverse=False,
            position_mm=2., orientation_deg=2., count=401):
    from task_dring_teach_data import moving
    a = aligned_rows(first,mode=mode,count=count)
    b = aligned_rows(second,mode=mode,reverse=reverse,count=count)
    from task_trace_markers import available_links
    links = available_links(side,context)
    endpoint_links = [link.split('_',1)[0]+'_tool0' for link in links]
    all_a = model.batch_transforms(links+endpoint_links,context,moving(side),a)
    all_b = model.batch_transforms(links+endpoint_links,context,moving(side),b)
    poses_a = {link:all_a[link] for link in links}
    poses_b = {link:all_b[link] for link in links}
    joint = np.max(np.abs(a-b),axis=1)
    details, differences = dict(left=None,right=None), {}
    for link in links:
        hand=link.split('_',1)[0]
        pa,pb = poses_a[link],poses_b[link]
        mm = 1000*np.linalg.norm(pa[:,:3,3]-pb[:,:3,3],axis=1)
        degrees = np.degrees((Rotation.from_matrix(pa[:,:3,:3]).inv()*Rotation.from_matrix(pb[:,:3,:3])).magnitude())
        differences[link] = (mm>position_mm)|(degrees>orientation_deg)|((joint>.01) if hand==side else False)
        details[hand] = dict(max_position_mm=float(np.max(mm)),rms_position_mm=float(np.sqrt(np.mean(mm**2))),
            max_orientation_deg=float(np.max(degrees)),different_percent=float(100*np.mean(differences[link])))
    return dict(first=poses_a,second=poses_b,differences=differences,
                first_endpoints={link:all_a[end] for link,end in zip(links,endpoint_links)},
                second_endpoints={link:all_b[end] for link,end in zip(links,endpoint_links)},
                report=dict(hands=details,max_joint_delta_rad=float(np.max(joint)),alignment=mode,reverse_second=reverse,
                            first_seconds=first['raw']['time_s'][-1],second_seconds=second['raw']['time_s'][-1]))
