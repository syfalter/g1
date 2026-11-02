"""Bounded, worker-owned caches of immutable recorded-trace display data."""
from collections import OrderedDict

import numpy as np


class TraceDisplayCache:
    def __init__(self,maximum_bytes=64*1024*1024):
        self.maximum_bytes=maximum_bytes
        self.geometries=OrderedDict();self.bytes=0
        self.frames=OrderedDict()

    def geometry(self,model,trace,side,context,config,slot):
        from task_dring_teach_data import moving
        from task_trace_markers import hand_context,available_links
        context=hand_context(trace,side,context,config,slot)
        names=moving(side)
        key=(id(model),id(trace['raw']),side,
             tuple(sorted((k,float(v)) for k,v in context.items() if k not in names)))
        if key in self.geometries:
            self.geometries.move_to_end(key)
            return self.geometries[key][1]
        rows=np.asarray(trace['raw']['positions'],dtype=float)
        times=np.asarray(trace['raw']['time_s'],dtype=float)
        links=available_links(side,context)
        endpoints=[link.split('_',1)[0]+'_tool0' for link in links]
        poses=model.batch_transforms(links+endpoints,context,names,rows)
        result=dict(rows=rows,times=times,links=links,endpoints=endpoints,transforms=poses,context=context)
        size=rows.nbytes+times.nbytes+sum(p.nbytes for p in poses.values())
        if size<=self.maximum_bytes:
            while self.geometries and (self.bytes+size>self.maximum_bytes or len(self.geometries)>=8):
                _,(_,_,old_size)=self.geometries.popitem(last=False);self.bytes-=old_size
            # Retain raw identity, preventing an id-reuse cache hit after an edit.
            self.geometries[key]=(trace['raw'],result,size);self.bytes+=size
        return result

    def frame(self,key):
        value=self.frames.get(key)
        if value is not None:self.frames.move_to_end(key)
        return value

    def remember_frame(self,key,value):
        if len(value[0])>16*1024*1024:return
        self.frames[key]=value;self.frames.move_to_end(key)
        while len(self.frames)>3:self.frames.popitem(last=False)
