#!/usr/bin/env python3
"""Render every raw RGB pair for an auditable visual applicability review."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from task_history_manifest import digest,records,inside


def annotate(dataset,atlas,annotations,output):
    rows=json.loads((atlas/'index.json').read_text())
    labels=json.loads(annotations.read_text())
    if labels['reviewed_count']!=len(rows):raise ValueError('Incomplete review')
    categories={}
    for name in ('no_target_calibration_board','no_target_floor_or_people','uncertain_board_or_occlusion'):
        for index in labels[name]:
            if index in categories or type(index) is not int or not 0<=index<len(rows):
                raise ValueError('Duplicate or invalid review index')
            categories[index]=name
    canonical=sorted(records(dataset/'pairs.jsonl'),key=lambda p:p['id'])
    reviewed=[]
    for index,(row,pair) in enumerate(zip(rows,canonical)):
        if row['pair_id']!=pair['id'] or row['number']!=index:raise ValueError('Review order mismatch')
        category=categories.get(index,'ring_visible')
        reviewed.append(dict(pair_id=pair['id'],number=index,visual_category=category,
                             raw_rgb_sha256=pair['sha256']['rgb.png'],
                             evidence_sheet=row['sheet'],evidence_sheet_sha256=digest(atlas/row['sheet']),
                             task_excluded=False,
                             note='Visual evidence only; exclusions require context consistency audit'))
    result=dict(method=labels['method'],annotations_sha256=digest(annotations),pairs=reviewed)
    with output.open('x') as stream:json.dump(result,stream,indent=2)
    from collections import Counter
    print(dict(Counter(r['visual_category'] for r in reviewed)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--annotations',type=Path)
    parser.add_argument('--atlas',type=Path)
    args=parser.parse_args()
    if args.output.resolve().is_relative_to(args.dataset.resolve()):
        parser.error('Review output must not modify dataset')
    if args.annotations:
        if not args.atlas:parser.error('--annotations requires --atlas')
        annotate(args.dataset,args.atlas,args.annotations,args.output)
        return
    args.output.mkdir(parents=True,exist_ok=False)
    rows=sorted(records(args.dataset/'pairs.jsonl'),key=lambda p:p['id'])
    index=[]
    for offset in range(0,len(rows),30):
        sheet=np.full((5*205,6*320,3),245,dtype=np.uint8)
        name=f'sheet_{offset//30:02d}.jpg'
        for local,row in enumerate(rows[offset:offset+30]):
            path=inside(args.dataset,row['directory'])/'rgb.png'
            if digest(path)!=row['sha256']['rgb.png']:raise ValueError('RGB checksum mismatch')
            image=cv2.imread(str(path),cv2.IMREAD_COLOR)
            if image is None:raise ValueError('Unreadable RGB: '+str(path))
            x,y=local%6*320,local//6*205
            sheet[y:y+180,x:x+320]=cv2.resize(image,(320,180),interpolation=cv2.INTER_AREA)
            cv2.putText(sheet,f'{offset+local:03d} {row["id"][:12]}',(x+4,y+197),
                        cv2.FONT_HERSHEY_SIMPLEX,.46,(0,0,0),1,cv2.LINE_AA)
            index.append(dict(number=offset+local,pair_id=row['id'],rgb=str(path.resolve()),sheet=name,
                              applicability='unreviewed'))
        if not cv2.imwrite(str(args.output/name),sheet,[cv2.IMWRITE_JPEG_QUALITY,94]):
            raise RuntimeError('Could not save contact sheet')
    (args.output/'index.json').write_text(json.dumps(index,indent=2)+'\n')
    print(f'{len(rows)} raw pairs in {len(range(0,len(rows),30))} sheets')


if __name__=='__main__':main()
