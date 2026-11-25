#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Short, rate-limited Hunter /cmd_vel verification helper for ROS1 Melodic.

IMPORTANT: This sender does not provide a receiver-side watchdog or physical
emergency stop. Do not use on a real robot until those protections are verified.
"""
from __future__ import print_function

import argparse
import sys
import time

import rospy
from geometry_msgs.msg import Twist


def make_twist(linear=0.0, angular=0.0):
    msg = Twist()
    msg.linear.x = float(linear)
    msg.angular.z = float(angular)
    return msg


def publish_for(pub, msg, duration, hz):
    period = 1.0 / float(hz)
    end_time = time.time() + duration
    while not rospy.is_shutdown() and time.time() < end_time:
        pub.publish(msg)
        time.sleep(period)


def main():
    parser = argparse.ArgumentParser(description="Short Hunter /cmd_vel motion test (ROS1).")
    parser.add_argument("mode", choices=("forward", "left", "right"),
                        help="One action per invocation; no reverse motion")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    print("\n*** 實體 Hunter 低速測試：{} ***".format(args.mode))
    print("僅在已驗證硬體急停、遙控器接管、底盤命令逾時停止後使用。")
    print("車輛需位於平坦、開闊、無人靠近的測試區，並有專人持遙控器。")
    print("本程式只能在正常執行時送出停止命令；無法防止 WSL / ROS / CAN 中斷。")
    print("/cmd_vel 必須沒有其他控制者，且需確認車型/方向與速度單位。")
    answer = input_compat("以上條件全部通過才輸入 RUN，否則直接 Enter 取消： ").strip()
    if answer != "RUN":
        print("取消測試。")
        return 0

    rospy.init_node("hunter_short_motion_test", anonymous=True)
    if rospy.get_param("/use_sim_time", False):
        rospy.logerr("/use_sim_time=true; refuse real-robot command test.")
        return 2

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    connect_deadline = time.time() + 5.0
    while (not rospy.is_shutdown() and pub.get_num_connections() == 0
           and time.time() < connect_deadline):
        time.sleep(0.05)
    if pub.get_num_connections() == 0:
        rospy.logerr("No /cmd_vel subscriber found; abort.")
        return 3

    # Conservative test values, not performance specifications.
    # 0.10 m/s, 0.04 rad/s: radius 2.5m and inner wheel steering angle ~0.29rad
    # with user-provided Hunter V1/V2 wheelbase and track settings.
    speed = 0.10
    yaw_rate = {"forward": 0.0, "left": 0.04, "right": -0.04}[args.mode]
    cmd = make_twist(speed, yaw_rate)
    stop = make_twist(0.0, 0.0)
    duration = 1.0
    rate = 60.0  # SDK notes command frequency should be >=50Hz; allow scheduling margin.

    try:
        print("先以約 60Hz 送出 1.0 秒零速度，確認車輛保持停止。")
        publish_for(pub, stop, 1.0, rate)
        if rospy.is_shutdown():
            return 4
        answer = input_compat("確認現場安全、車輛已進入受控自駕模式，輸入 GO 執行短脈衝： ").strip()
        if answer != "GO":
            print("取消運動；繼續送出零速度。")
            return 0
        print("發布：linear.x={:.2f} m/s, angular.z={:+.2f} rad/s，持續 {:.1f} 秒".format(speed, yaw_rate, duration))
        publish_for(pub, cmd, duration, rate)
    except KeyboardInterrupt:
        print("收到 Ctrl+C；嘗試發送零速度。")
    finally:
        # Best-effort stop only. Does not replace a receiver watchdog or E-stop.
        print("發布零速度 2 秒（僅屬軟體停止嘗試）。")
        try:
            publish_for(pub, stop, 2.0, rate)
        except Exception as e:
            print("無法完成零速度發布：{}。立即使用實體急停/遙控器接管。".format(e))
    print("測試完成。觀察底盤實際停止、/hunter_status，並切回安全模式。")
    return 0


def input_compat(prompt):
    try:
        return raw_input(prompt)
    except NameError:
        return input(prompt)


if __name__ == "__main__":
    sys.exit(main())
