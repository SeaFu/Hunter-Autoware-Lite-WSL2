#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate Autoware-compatible 6-column-prefix CSV + per-segment preview.

When supplied a completed live preview JSON the EXACT preview samples are
exported to preserve the recorded RViz colors and segment identities. No
additional report is generated. The state JSON lives in /tmp and may be deleted
once the Waypoint CSV is committed.
"""
import argparse
import csv
import json
import math
import os
from nav_trajectory_feasibility import (EXTRA_FIELDS, profile_for, profile_id,
    evaluate_segments, sample_trajectory)


def _pose_samples(source):
    with open(source,encoding='utf-8-sig',newline='') as f:
        reader=csv.DictReader(f)
        required=('x','y','z','yaw')
        if not reader.fieldnames or not all(x in reader.fieldnames for x in required):
            raise ValueError('輸入 CSV 必須有 x,y,z,yaw 欄位')
        samples=[]
        for row in reader:
            try: v=dict((k,float(row[k])) for k in required)
            except (TypeError,ValueError,KeyError):continue
            if all(math.isfinite(x) for x in v.values()): samples.append(v)
        return samples


def generate(source,target,spacing=.5,speed_kmh=1.0,max_jump=1.5,
             state_json=None,profile=None):
    if not (.05<=spacing<=10 and 0<speed_kmh<=15 and .2<=max_jump<=100):
        raise ValueError('不合理的 waypoint 參數')
    if os.path.exists(target):
        raise ValueError('輸出已存在，為避免覆寫請另選檔名: '+target)
    live_state=None
    if state_json:
        with open(state_json,encoding='utf-8') as f:live_state=json.load(f)
        if (live_state.get('schema')!='hunter_feasibility_v1' or
                not live_state.get('completed')):
            raise ValueError('線上預檢尚未完成；先停止錄製並等待預檢節點結束')
        selected=live_state['samples']
        assessment=live_state['segments']
        profile=live_state['profile']
        if len(assessment)!=len(selected):
            raise ValueError('預檢快照路段數與 Waypoint 數不一致')
        if abs(spacing-float(profile['spacing_m'])) > 1e-7:
            raise ValueError('錄製間距與輸出間距不一致；為保留顏色請使用錄製時設定')
    else:
        samples=_pose_samples(source)
        if len(samples)<3:raise ValueError('有效 NDT 位姿少於 3 筆')
        selected=[samples[0]]
        for item in samples[1:]:
            gap=math.hypot(item['x']-selected[-1]['x'],item['y']-selected[-1]['y'])
            if gap>max_jump:raise ValueError('NDT 位姿跳動 {:.2f} m，超過上限 {:.2f} m'.format(gap,max_jump))
            if gap>=spacing:selected.append(item)
        if selected[-1]!=samples[-1]:
            gap=math.hypot(samples[-1]['x']-selected[-1]['x'],samples[-1]['y']-selected[-1]['y'])
            if gap>max_jump:raise ValueError('末段跳點超過上限')
            if gap>=.05:selected.append(samples[-1])
        if profile is None:
            profile=profile_for(1.17,2.,.80,.55,1.,2.4,max(.05,speed_kmh/3.6),spacing=spacing)
        assessment=evaluate_segments(selected,profile,complete=True)
    if len(selected)<5:raise ValueError('有效 Waypoint 少於 5 點；請減少取樣距離')
    for i in range(1,len(selected)):
        gap=math.hypot(selected[i]['x']-selected[i-1]['x'],
                       selected[i]['y']-selected[i-1]['y'])
        if gap<.05 or gap>1.5:
            raise ValueError('第 {}~{} 點間距 {:.3f}m；CSV 格式檢查無法通過'.format(i,i+1,gap))
    parent=os.path.dirname(os.path.abspath(target));os.makedirs(parent,exist_ok=True)
    pid=profile_id(profile)
    columns=['x','y','z','yaw','velocity','change_flag']+list(EXTRA_FIELDS)
    with open(target,'x',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=columns);w.writeheader()
        for i,p in enumerate(selected):
            row={k:'{:.8f}'.format(float(p[k])) for k in ('x','y','z','yaw')}
            row.update(velocity='0.0' if i==len(selected)-1 else '{:.3f}'.format(speed_kmh),
                       change_flag='0')
            row.update(assessment[i])
            row['check_profile']=pid
            w.writerow(row)
    return len(selected)

if __name__=='__main__':
    a=argparse.ArgumentParser()
    a.add_argument('--input',required=True);a.add_argument('--output',required=True)
    a.add_argument('--spacing-m',type=float,default=.5)
    a.add_argument('--speed-kmh',type=float,default=1.)
    a.add_argument('--max-jump-m',type=float,default=1.5)
    a.add_argument('--state-json')
    a.add_argument('--profile-json')
    opts=a.parse_args()
    profile=json.loads(opts.profile_json) if opts.profile_json else None
    n=generate(opts.input,opts.output,opts.spacing_m,opts.speed_kmh,
               opts.max_jump_m,opts.state_json,profile)
    print('Waypoint 彩色預檢 CSV 已建立 {}，{} 點（只供預檢，無馬達控制）'.format(opts.output,n))
