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
# - Haruki Ogawa

import gtpyhop
from constants import BALL_POSITION, ROBOT_FIELD
from print_utils import print_in_real_action
from real_action import action_functions
from state import Position
from decision_logger import DecisionLogger
from utils import (
    calculate_angle,
    get_shared_ally_positions,
    calculate_distance_global,
    convert_local_to_global,
    convert_hardcoded_pos_to_our_field
)

from strategy.ros_publishers import WalkCommand, WalkCommandPublisher


# そもそもfollower_taskに切り替わる条件として情報共有で誰かattackerでいることが前提
def follower_task_method(s: gtpyhop.State, pub, logger: DecisionLogger):
    """
    Define the actions for the follower role.
    """
    # 味方ロボットが検出されない場合はattackerに切り替わる
    tmp = get_shared_ally_positions(s)
    shared_allys = [x for x in tmp if x is not None]
    if not s.allies_position_local and len(shared_allys) == 0:
        print_in_real_action(
            "follower_task_method",
            "味方が見つかりませんでした。アタッカーの役割に切り替えます",
        )
        logger.task_executed(details="No allies found, switching to attacker role")
        return [("attacker_task", pub, logger)]
    if s.ball_position_local is None:
        # ボールが見えない場合
        logger.task_executed(details="Ball not detected, executing search and turn to ball task")
        return [("task_search_and_turn_to_ball", pub, logger)]
    else:
        # ボールが見えており、味方もいる場合は追従アクションを実行
        logger.task_executed(details="Ball detected, executing approach_attacker_follower action")
        return [("approach_attacker_follower", pub, logger)]


# アクション
def approach_attacker_follower(s: gtpyhop.State, pub: WalkCommandPublisher, logger: DecisionLogger):
    # 味方local、ボールlocal、自分のglobalがある場合
    if s.allies_position_local and s.ball_position_local is not None:
        # s.allies_position_localはPositionオブジェクトのリストなので、最初の要素を取得する
        first_ally_local_position = s.allies_position_local[0]

        # 自分とボールの差分、味方とボールの差分を比較して
        # ボールのグローバル座標
        ball_gl = convert_local_to_global(s.self_position_global, s.ball_position_local)
        allies_gl = convert_local_to_global(
            s.self_position_global, first_ally_local_position
        )

        diff_ball_alies = calculate_distance_global(ball_gl, allies_gl)
        diff_ball_self_position = calculate_distance_global(
            ball_gl, s.self_position_global
        )

        if diff_ball_self_position > diff_ball_alies:
            # 味方と一定の距離を保って、追従する
            # ペナルティエリアよりボールが後ろにあるとそれ以上下がらない
            if ball_gl.x > -450.0:
                allies_follower_pos_x = ball_gl.x - 250.0
            else:
                allies_follower_pos_x = -450.0

            # ボールのy座標から離れる
            if ball_gl.y > 0.0:
                allies_follower_pos_y = ball_gl.y - 250.0
            else:
                allies_follower_pos_y = ball_gl.y + 250.0

            follower_position_global = Position(
                Position.COORD_GLOBAL, allies_follower_pos_x, allies_follower_pos_y, 0.0
            )

            # 目標位置とロボットの現在の位置のグローバル座標間の距離を計算
            distance_to_target = calculate_distance_global(
                s.self_position_global, follower_position_global
            )

            # ロボットが目標位置に十分に近づいたら、ボールの方向を向く
            if distance_to_target < 150.0:  # 距離が150mm以内なら到着したとみなす
                # ローカル座標のボールの角度を計算
                ball_angle = calculate_angle(s.ball_position_local)

                # 首の動きと体の旋回を調整
                # 首をボールの方向へ
                # pub.move_neck_yaw(yaw_angle=-ball_angle)
                # 体の向きを調整するための旋回コマンドを生成
                cmd = action_functions.generate_turn_command(s.ball_position_local)
                pub.send_walk_command(cmd)
                logger.action_executed(details="Turning to face the ball")

                return s

            else:
                cmd = action_functions.move_to_target_global_pose(
                    s, pub, follower_position_global
                )
                if cmd is not None:
                    pub.send_walk_command(cmd)
                logger.action_executed(details="Moving to follower position")
                return s
        else:
            # 自分の方がボールに近い場合、followerからattackerに切り替えるなど、
            # 新しいタスクへ遷移するロジックをここに追加することを検討
            print_in_real_action(
                "approach_attacker_follower",
                "Self is closer to the ball. Transitioning to a new task.",
            )
            logger.action_postcondition_already_satisfied(details="Self is closer to ball than ally, no action taken")
            return s  # シンプルに現在の状態を維持

    # ボールが見えない場合
    else:
        print_in_real_action(
            "approach_attacker_follower",
            "Ball is not visible. Moving to a default defense position.",
        )

        # ボールが見えない場合の自陣でボールが探しやすい場所に移動
        default_follower_position = convert_hardcoded_pos_to_our_field(s,Position(
            Position.COORD_GLOBAL,
            -300.0,
            0.0,
            0.0,
        ))
        cmd = action_functions.move_to_target_global_pose(
            s, pub, default_follower_position
        )
        if cmd is not None:
            pub.send_walk_command(cmd)
        logger.action_executed(details="Moving to default defense position as ball is not visible")
        return s


gtpyhop.declare_actions(approach_attacker_follower)

gtpyhop.declare_task_methods("follower_task", follower_task_method)
