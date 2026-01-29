#!/usr/bin/env python3
#
# Copyright 2026 RFC-Tsudanuma
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software
# and associated documentation files (the “Software”), to deal in the Software without restriction,
# including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# [Contributors]
#
# - Satoshi Inoue
# - Yugo Nishio


import rclpy
import time

import strategy.ros_publishers as publisher

if __name__ == "__main__":
    rclpy.init(args=None)
    pub = publisher.WalkCommandPublisher()
    # ここではセンチ単位になる
    # pub.send_walk_command(publisher.WalkCommand(50, 0.0, 0.0))  # 停止コマンドを送信
    # time.sleep(3)

    cmd = publisher.WalkCommand(0.0, 0.0, 0.0)
    pub.send_walk_command(cmd)
    time.sleep(0.2)
    cmd = publisher.WalkCommand(0.0, 0.0, 0.0)
    pub.send_walk_command(cmd)
    time.sleep(0.2)
    cmd = publisher.WalkCommand(0.0, 0.0, 0.0)
    pub.send_walk_command(cmd)
    time.sleep(0.2)

    # パラメータ設定
    d = 3.0
    v0 = 0.41
    t0 = 1.47
    t1 = 1.733
    t3 = 1.676
    vx_r = 0.94

    # 距離から送信時間計算
    # t2 = (d - 0.5 * vx_r * (t1 + t3)) / vx_r
    d0 = 0.5 * (v0 * t0)
    d1 = 0.5 * t1 * (vx_r ** 2 - v0 ** 2) / vx_r
    d3 = 0.5 * vx_r * t3
    d2 = d - (d0 + d1 + d3)
    t2 = d2 / vx_r
    t1_ = d1 / (vx_r - v0)
    t_send = t1_ + t2

    cmd = publisher.WalkCommand(40.0, 0.0, 0.0)
    pub.send_walk_command(cmd)

    time.sleep(t0)

    cmd = publisher.WalkCommand(90.0, 0.0, 0.0)
    pub.send_walk_command(cmd)

    time.sleep(t_send)


    cmd = publisher.WalkCommand(0.0, 0.0, 0.0)
    pub.send_walk_command(cmd)

    time.sleep(3)