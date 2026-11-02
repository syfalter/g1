"""Render offline candidate paths in pelvis coordinates, not camera measurements."""
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from prepare_dring_candidates import ARCHIVE
from task_dring_teach_data import moving
from workbench_model import OfflineModel


def main():
    bundle=json.loads((ARCHIVE/'candidates/demonstration.json').read_text())
    model=OfflineModel(ARCHIVE/'robot.urdf');side=bundle['side'];link=side+'_wrist_yaw_link'
    offset=np.array(json.loads((ARCHIVE/'index_tcp.json').read_text())['sides'][side]['states']['open']
                    ['wrist_yaw_from_index_tcp']['matrix_row_major'])
    fig,axes=plt.subplots(2,2,figsize=(12,10),constrained_layout=True)
    for row,direction in enumerate(('left','right')):
        circle=bundle['traces'][direction+'_2']['optimization']['circle']
        center=np.array(circle['center_xy_pelvis_m'])*1000
        for index in range(1,6):
            slot=direction if index==1 else f'{direction}_{index}';trace=bundle['traces'][slot]
            t=np.array(trace['raw']['time_s']);q=np.array(trace['raw']['positions'])
            mask=(t>=circle['contact_interval_s'][0])&(t<=circle['contact_interval_s'][1])
            p=(model.batch_transforms([link],{},moving(side),q[mask])[link]@offset)[:,:3,3]*1000
            label='Original' if index==1 else 'Smoothed' if index==2 else f'Smoothed +{index-2} deg'
            for ax in axes[row]:ax.plot(p[:,0],p[:,1],label=label,linewidth=1.4)
        axes[row,0].scatter(*center,marker='+',color='black',label='Estimated local center')
        axes[row,0].set_title(direction.title()+' contact arc (nominal TCP FK)')
        axes[row,1].set_title(direction.title()+' end-of-arc detail')
        axes[row,1].set_xlim(p[-1,0]-10,p[-1,0]+10)
        axes[row,1].set_ylim(p[-1,1]-10,p[-1,1]+10)
        for ax in axes[row]:
            ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.3)
            ax.set_xlabel('Pelvis X (mm)');ax.set_ylabel('Pelvis Y (mm)')
        axes[row,0].legend(fontsize=8)
    fig.savefig(ARCHIVE/'candidates/comparison.png',dpi=160)


if __name__=='__main__':main()
