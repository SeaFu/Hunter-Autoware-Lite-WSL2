#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Re-evaluate a recorded waypoint CSV without changing motor controls or source.

This RE-CHECK has no access to current map/cloud: geometry is checked, but a
PASS without map evidence becomes WARN (NO_MAP_CLEARANCE_CHECK). The original
CSV and its previous color decisions remain unchanged for traceability.
"""
import argparse
import csv
import json
import math
import os
from nav_trajectory_feasibility import EXTRA_FIELDS, evaluate_segments, profile_id
from route_safety import load_route, validate


def recheck(path, profile):
    if not os.path.isfile(path):
        raise ValueError('找不到輸入 Waypoint CSV：'+path)
    points=load_route(path)
    validate(points)
    # Strictly preserve original route samples, pose yaw, speed, and stop flag.
    assessment=evaluate_segments(points, profile, map_xy=None,complete=True)
    if len(assessment)!=len(points):
        raise ValueError('預檢路段數不符合 Waypoint 數量')
    root,ext=os.path.splitext(os.path.abspath(path))
    dst='{}_rechecked_{}{}'.format(root,profile_id(profile),ext)
    if os.path.exists(dst):
        raise ValueError('新檔已存在；請移動舊檔或更換預檢參數：'+dst)
    keys=['x','y','z','yaw','velocity','change_flag']+list(EXTRA_FIELDS)
    with open(dst,'x',newline='',encoding='utf-8') as out:
        writer=csv.DictWriter(out,fieldnames=keys)
        writer.writeheader()
        for i,p in enumerate(points):
            row={key:'{:.8f}'.format(p[key]) for key in keys[:6]}
            row.update(assessment[i]);row['check_profile']=profile_id(profile)
            writer.writerow(row)
    return dst, assessment


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--input',required=True)
    parser.add_argument('--profile-json',required=True)
    args=parser.parse_args()
    profile=json.loads(args.profile_json)
    dst,assessment=recheck(args.input, profile)
    breakdown={key:sum(1 for s in assessment[:-1] if s['segment_status']==key)
               for key in ('PASS','WARN','FAIL','PENDING')}
    print('NEW_CSV={} 狀態={}（僅離線預檢；無地圖淨空覆核，原 CSV 未覆寫）'.format(dst,breakdown))

if __name__=='__main__':main()
