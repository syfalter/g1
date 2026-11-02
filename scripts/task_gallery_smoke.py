"""Pure X11 window and viewport checks for isolated RViz smoke tests."""
import re

import numpy as np


def window_geometry(line):
    match=re.search(r'^\s*(0x[0-9a-fA-F]+) .*? (\d+)x(\d+)[+-]\d+[+-]\d+\s+([+-]\d+)([+-]\d+)\s*$',line)
    if match is None:return None
    return dict(id=int(match[1],16),width=int(match[2]),height=int(match[3]),x=int(match[4]),y=int(match[5]))


def scoped_rviz_window(tree,config):
    title=re.compile(r'"'+re.escape(str(config))+r'\*? - RViz"')
    matches=[window_geometry(line) for line in tree.splitlines() if title.search(line)]
    if len(matches)!=1 or matches[0] is None:raise ValueError('Unique scoped RViz window not found')
    return matches[0]


def render_region(tree,window):
    children=[record for line in tree.splitlines() if (record:=window_geometry(line)) is not None
              and record['id']!=window['id'] and record['width']>=200 and record['height']>=150]
    if not children:raise ValueError('RViz render window not found')
    child=max(children,key=lambda r:r['width']*r['height'])
    x,y=child['x']-window['x'],child['y']-window['y']
    if x<0 or y<0 or x+child['width']>window['width'] or y+child['height']>window['height']:
        raise ValueError('RViz render window is outside the scoped window')
    return x,y,child['width'],child['height']


def viewport_metrics(image):
    image=np.asarray(image)
    if image.ndim!=3 or image.shape[2]!=3 or min(image.shape[:2])<100:
        raise ValueError('Invalid RViz viewport image')
    difference=np.max(np.abs(image.astype(float)-np.array([38.,34.,32.])),axis=2)
    return dict(width=image.shape[1],height=image.shape[0],
                spatial_pixel_std=float(np.std(image,axis=(0,1)).mean()),
                foreground_fraction=float(np.mean(difference>12.)))


def visible_viewport(metrics):
    return metrics['spatial_pixel_std']>8. and metrics['foreground_fraction']>.01


def changed_fraction(first,second):
    if first.shape!=second.shape:raise ValueError('RViz viewport dimensions changed during smoke test')
    return float(np.mean(np.max(np.abs(first.astype(float)-second.astype(float)),axis=2)>12.))
