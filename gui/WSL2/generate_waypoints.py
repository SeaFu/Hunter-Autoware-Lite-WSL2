#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert NDT pose CSV to Autoware waypoint ver3 CSV, offline/pure output only."""
import argparse
import csv
import math
import os


def generate(source, target, spacing=0.5, speed_kmh=1.0, max_jump=1.5):
    if not (0.05 <= spacing <= 10 and 0 < speed_kmh <= 15 and 0.2 <= max_jump <= 100):
        raise ValueError('不合理的 waypoint 參數')
    if os.path.exists(target):
        raise ValueError('輸出已存在，為避免覆寫請另選檔名: '+target)
    with open(source, encoding='utf-8-sig', newline='') as f:
        reader=csv.DictReader(f)
        required=('x','y','z','yaw')
        if not reader.fieldnames or not all(x in reader.fieldnames for x in required):
            raise ValueError('輸入 CSV 必須有 x,y,z,yaw 欄位（NDT 整理後 CSV）')
        samples=[]
        for row in reader:
            try: values=tuple(float(row[k]) for k in required)
            except (TypeError,ValueError,KeyError):continue
            if all(math.isfinite(a) for a in values):samples.append(values)
    if len(samples)<3:raise ValueError('有效 NDT 位姿少於 3 筆')
    selected=[samples[0]]
    for item in samples[1:]:
        gap=math.hypot(item[0]-selected[-1][0],item[1]-selected[-1][1])
        if gap>max_jump:raise ValueError('NDT 軌跡位姿跳動 {:.2f} m，超過上限 {:.2f} m'.format(gap,max_jump))
        if gap>=spacing:selected.append(item)
    if selected[-1]!=samples[-1]:
        gap=math.hypot(samples[-1][0]-selected[-1][0],samples[-1][1]-selected[-1][1])
        if gap>max_jump:raise ValueError('末段跳點超過上限')
        selected.append(samples[-1])
    if len(selected)<3:raise ValueError('有效 Waypoint 少於 3 點，請減少取樣距離')
    parent=os.path.dirname(os.path.abspath(target));os.makedirs(parent,exist_ok=True)
    with open(target,'x',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(['x','y','z','yaw','velocity','change_flag'])
        for i,(x,y,z,yaw) in enumerate(selected):
            w.writerow(['{:.8f}'.format(v) for v in (x,y,z,yaw)]+['0.0' if i==len(selected)-1 else '{:.3f}'.format(speed_kmh),'0'])
    return len(selected)

if __name__=='__main__':
    a=argparse.ArgumentParser()
    a.add_argument('--input',required=True);a.add_argument('--output',required=True)
    a.add_argument('--spacing-m',type=float,default=0.5)
    a.add_argument('--speed-kmh',type=float,default=1.0)
    a.add_argument('--max-jump-m',type=float,default=1.5)
    opts=a.parse_args()
    print('Waypoint v3 已寫入 {}，{} 點（只供預覽，無馬達控制）'.format(opts.output,generate(opts.input,opts.output,opts.spacing_m,opts.speed_kmh,opts.max_jump_m)))
