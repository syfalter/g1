"""Conservative RGB-D target separation for uncommissioned planning previews.

A detector bbox is not an object mask. Ambiguous cells remain background;
this inference must never be treated as a calibrated physical contact model.
"""
import numpy as np


def separate_contact_surface(xyz, uv, bbox, center, resolution=.015):
    xyz, uv, center = np.asarray(xyz), np.asarray(uv), np.asarray(center)
    bbox = np.asarray(bbox, dtype=float)
    if (xyz.ndim != 2 or xyz.shape[1:] != (3,) or uv.shape != (len(xyz),2)
            or center.shape != (3,) or bbox.shape != (4,)
            or not all(np.isfinite(a).all() for a in (xyz,uv,center,bbox))
            or not 0 < resolution <= .02 or np.any(bbox[2:] <= bbox[:2])):
        raise ValueError('TARGET_NOT_SEPARATED: invalid RGB-D/bbox geometry')
    inside = np.all((uv >= bbox[:2]) & (uv <= bbox[2:]), axis=1)
    radius = np.linalg.norm(xyz[:,:2]-center[:2], axis=1)
    base = xyz[(radius>.10)&(radius<.30)&(~inside)&(abs(xyz[:,2]-center[2])<.12)]
    if len(base)<500:
        raise ValueError('TARGET_NOT_SEPARATED: insufficient surrounding table observations')
    design = np.c_[base[:,:2],np.ones(len(base))]
    mask = np.ones(len(base), dtype=bool)
    for _ in range(8):
        if sum(mask)<500:raise ValueError('TARGET_NOT_SEPARATED: table plane support collapsed')
        coef, _, rank, _ = np.linalg.lstsq(design[mask],base[mask,2],rcond=None)
        error = base[:,2]-design@coef
        mask = abs(error)<.008
    rms = float(np.sqrt(np.mean(error[mask]**2)))
    if rank!=3 or mask.mean()<.55 or rms>.006 or np.linalg.norm(coef[:2])>.20:
        raise ValueError('TARGET_NOT_SEPARATED: unreliable local table plane')
    if np.any(np.ptp(base[mask,:2],axis=0)<.20):
        raise ValueError('TARGET_NOT_SEPARATED: table plane lacks spatial coverage')
    height = xyz[:,2]-np.c_[xyz[:,:2],np.ones(len(xyz))]@coef
    candidate = inside & (radius<.16) & (height>.018) & (height<.10)
    # If any measured point in a voxel is uncertain, keep that entire cell solid.
    cells, inverse = np.unique(np.floor(xyz/resolution).astype(np.int64),axis=0,return_inverse=True)
    eligible = np.ones(len(cells), dtype=bool)
    np.logical_and.at(eligible,inverse,candidate)
    lower = cells*resolution
    plane_max = ((lower[:,:2]+np.where(coef[:2]>0,resolution,0))*coef[:2]).sum(axis=1)+coef[2]
    eligible &= lower[:,2]-plane_max > .012
    selected = eligible[inverse]
    if sum(selected)<100 or sum(eligible)<8:
        raise ValueError('TARGET_NOT_SEPARATED: no sufficiently elevated target cells')
    # A static sweep conservatively obstructs other links throughout the rotation.
    target = (cells[eligible]+.5)*resolution
    delta = target-center
    max_radius = float(np.linalg.norm(delta[:,:2],axis=1).max())
    angles = np.linspace(-2*np.pi/3,2*np.pi/3,
                         max(3,int(np.ceil((4*np.pi/3)*max_radius/(resolution*.4)))+1))
    swept=[]
    angular_margin=max_radius*(angles[1]-angles[0])/2
    offsets=np.array(np.meshgrid(range(4),range(4),range(4),indexing='ij')).reshape(3,-1).T
    for angle in angles:
        c,s=np.cos(angle),np.sin(angle)
        rotated=delta.copy()
        rotated[:,:2]=delta[:,:2]@np.array([[c,s],[-s,c]])
        rotated+=center
        extent=np.array([.5*resolution*(abs(c)+abs(s))+angular_margin]*2+[resolution*.5])
        lo=np.floor((rotated-extent)/resolution).astype(np.int64)
        hi=np.floor((rotated+extent)/resolution).astype(np.int64)
        if np.any(hi-lo>3):raise ValueError('TARGET_NOT_SEPARATED: sweep voxel bounds exceeded')
        boxes=lo[:,None,:]+offsets[None,:,:]
        covered=boxes[np.all(boxes<=hi[:,None,:],axis=2)]
        swept.append(covered)
    report={'source':'bbox_plus_elevated_voxels_preview_only','hardware_validated':False,
            'resolution_m':resolution,'background_points':int(sum(~selected)),
            'target_points':int(sum(selected)),'target_cells':int(sum(eligible)),
            'plane_z_from_xy':coef.tolist(),'plane_rms_m':rms,'plane_support':float(mask.mean()),
            'contact_links':['index_2','index_rubber_2','index_pad','index_tip'],
            'contact_stages':['insert','rotate','withdraw']}
    return xyz[~selected],(np.unique(np.vstack(swept),axis=0)+.5)*resolution,report
