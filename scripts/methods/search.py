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
# - Suzuha Kiuchi
# - Yugo Nishio


import math

import gtpyhop
import setting
import utils
from state import Position
from constants import BALL_POSITION, ROBOT_FIELD
from print_utils import print_in_real_action, print_in_task_method
from strategy_time import get_strategy_time
from strategy.ros_publishers import WalkCommand
from real_action.action_functions import move_to_target_global_pose, generate_turn_command_by_angle
from decision_logger import DecisionLogger

# ここからアクション ---------------------------------------------------------

# turn
def search_ball_by_turning(s: gtpyhop.State, pub, logger: DecisionLogger):
    """
    回転することで、ボールを探すアクション。
    """
    if (
        s.world_states["ball_position_relative"] == BALL_POSITION.UNKNOWN
    ):
        if s.last_visible_ball[0] is not None and s.self_position_global is not None:
            print_in_real_action("search_ball_by_turning", "Turn to last seen ball")
            # 最後に見たボールの記憶があるならそちらに回転する
            angle = utils.calculate_angle(utils.convert_global_to_local(s.self_position_global, s.last_visible_ball[0]))
            if angle > 0:
                cmd = WalkCommand(0.0, 0.0, theta_velocity=setting.ROBOT_MAX_SPEED_PI)
            else:
                cmd = WalkCommand(0.0, 0.0, theta_velocity=setting.ROBOT_MAX_SPEED_PI * -1.0)
        else:
            print_in_real_action("search_ball_by_turning", "Turn clockwise...")
            # 場所の予想がつかない場合は右回りで探索
            cmd = WalkCommand(0.0, 0.0, theta_velocity=setting.ROBOT_MAX_SPEED_PI * 0.8)
        if not pub.is_turning_with_only_theta_vel_now():
            # 角度の符号の変わり目付近で、今ちゃんと回転しているならそのままの回転を続ける
            pub.send_walk_command(cmd)
        logger.action_executed(details="Turning to search for the ball")
        return s # これを実行したらSearchは終了
    logger.action_precondition_not_satisfied(details="Ball already detected, no need to turn")
    return s

def search_ball_by_walking_around(s: gtpyhop.State, pub, logger: DecisionLogger):
    """
    自陣まで帰ることでボールを探すアクション
    """
    if (
        s.world_states["ball_position_relative"] == BALL_POSITION.UNKNOWN
        and s.self_position_global is not None
    ):
        # 自陣側の場合はとりあえず自陣の中央付近に移動する
        target_global_pose = utils.convert_hardcoded_pos_to_our_field(s, utils.get_start_position())
        print_in_real_action("search_ball_by_walking_around", "Move to our field center")
        cmd = move_to_target_global_pose(s, pub, target_global_pose)
        if cmd is not None:
            pub.send_walk_command(cmd)
        else:
            pub.send_stop_command()
        logger.action_executed(details="Walking around to search for the ball")
        return s
    logger.action_precondition_not_satisfied(details="Ball already detected or self position unknown")
    return s

def task_select_search_action_method(s: gtpyhop.State, pub, logger: DecisionLogger):
    # この探索はボールが2秒以上見えていない場合に実行される
    # まず、味方がボール位置を知っているならそちらを向く
    # 知らないなら、最後の自分の記憶の方向に向かって回転を開始する
    # 👆の条件で10秒以上探索して何も見つからなかったら、既定の位置に戻る
    # しかし、味方がボール位置を知っているならそちらに向かう
    # 既定の位置は自陣の中央付近
    if s.world_states["current_robot_field"] == ROBOT_FIELD.UNKNOWN:
        # 自己位置が不明な場合はその場回転する
        print_in_task_method("task_select_search", "Selfpos unknown, turning...")
        logger.task_executed(details="Self position unknown, executing turning search")
        return [("search_ball_by_turning", pub, logger)]
    # 共有されたボール位置があるならそちらを向く
    balls = utils.get_other_player_ballpos_from_shared_info(s.shared_msg)
    if len(balls) > 0:
        print_in_task_method("task_select_search", "Turning to shared ball position...")
        logger.task_executed(details="Turning to shared ball position")
        return [("turn_to_shared_ball_position", pub, logger)]
    # 歩行の目標位置に十分近ければ回転探索を行う
    # それはそれとして、自己位置が分からない場合も回転探索する必要がある
    if s.self_position_global is None:
        print_in_task_method("task_select_search", "Selfpos unknown, turning...")
        logger.task_executed(details="Self position unknown, executing turning search")
        return [("search_ball_by_turning", pub, logger)]
    if utils.are_target_poses_near(s.self_position_global, utils.convert_hardcoded_pos_to_our_field(s, utils.get_start_position()), 100.0, 100.0, math.radians(30.0)):
        print_in_task_method("task_select_search", "Near target position, turning...")
        logger.task_executed(details="Near target position, executing turning search")
        return [("search_ball_by_turning", pub, logger)]
    # ボールが見えなくなってから10秒以内なら回転探索を行う
    if (get_strategy_time() - s.last_visible_ball[1] < 10.0):
        print_in_task_method("task_select_search", "Turn to search...")
        logger.task_executed(details="Executing turning search for the ball")
        return [("search_ball_by_turning", pub, logger)]
    else:
        # 10秒以上見えず、かつ目標位置にも十分近くない場合はそこに移動する
        print_in_task_method("task_select_search", "Walk to target position to search...")
        logger.task_executed(details="Walking to target position to search for the ball")
        return [("search_ball_by_walking_around", pub, logger)]

def turn_to_shared_ball_position(s: gtpyhop.State, pub, logger: DecisionLogger):
    """
    共有されたボールの位置に向かって回転する
    """
    if s.world_states["ball_position_relative"] == BALL_POSITION.UNKNOWN:
        balls = utils.get_other_player_ballpos_from_shared_info(s.shared_msg)
        if len(balls) > 0:
            print_in_real_action("turn_to_shared_ball_position", "Turn to shared ball position")
            # 最後に見たボールの記憶があるならそちらに回転する
            if s.self_position_global is None:
                logger.action_precondition_not_satisfied(details="Self position unknown, cannot turn to shared ball position")
                return None
            ballpos = utils.get_other_player_average_ballpos_from_shared_info(s.shared_msg)
            angle = utils.calculate_angle(utils.convert_global_to_local(s.self_position_global, ballpos))
            cmd = generate_turn_command_by_angle(angle)
            pub.send_walk_command(cmd)
            logger.action_executed(details="Turning to shared ball position")
            return s
    logger.action_postcondition_already_satisfied(details="Ball already detected")
    return None

def search_ball_by_neck_movement(s: gtpyhop.State, pub, logger: DecisionLogger):
    """
    Search for the ball by moving the neck.
    """
    # preconditionを確認
    if s.world_states["ball_position_relative"] == BALL_POSITION.UNKNOWN:
        print_in_real_action(
            "search_ball_by_neck_movement", "This function delegate to neck movement."
        )
        logger.action_executed(details="Delegating to neck movement for ball search")
        return s
    else:
        print_in_real_action(
            "search_ball_by_neck_movement", "This function delegate to neck movement."
        )        # ボールが見つかっている場合はその方向を向く
        logger.action_executed(details="Delegating to neck movement")
        return s  # preconditionが満たされているので、アクションは成功したとみなす


# neck and turn
def turn_to_detected_ball(s: gtpyhop.State, pub, logger: DecisionLogger):
    # ボールが見つかっていて、歩いていない時に実行する or
    # ボールが見つかっていて、歩いているが、ボールとの距離が遠いとき
    if (
        s.world_states["ball_position_relative"] != BALL_POSITION.UNKNOWN
        and not pub.is_walking_with_lin_vel_now()
    ) or (
        s.world_states["ball_position_relative"] != BALL_POSITION.UNKNOWN
        and pub.is_walking_with_lin_vel_now()
        and utils.calculate_distance(s.ball_position_local) > 300
    ):
        print_in_real_action("turn_to_detected_ball", "Execute")
        # ボールの位置がわかっている場合は、その方向を向く
        ball_angle = utils.calculate_angle(s.ball_position_local)
        if (
            math.fabs(ball_angle)
            > setting.ROBOT_MAX_SPEED_PI * setting.STRATEGY_SLEEP_TIME
        ):
            cmd = WalkCommand(
                0.0,
                0.0,
                theta_velocity=0.9
                * math.copysign(setting.ROBOT_MAX_SPEED_PI, ball_angle),
            )
            pub.send_walk_command(cmd)
        else:
            time = setting.STRATEGY_SLEEP_TIME - 0.05
            theta_vel = ball_angle / time * 0.9
            cmd = WalkCommand(0.0, 0.0, theta_velocity=theta_vel, continuous_time=time)
            pub.send_walk_command(cmd)
        logger.action_executed(details="Turning to face the detected ball")
        return s
    else:
        logger.action_postcondition_already_satisfied(details="Ball not detected or already facing ball")
        return s

def stop_if_lin_vel_walking(s: gtpyhop.State, pub, logger: DecisionLogger):
    """
    線速度で歩いている場合に停止するアクション。
    """
    print_in_real_action("stop_if_lin_vel_walking", "Executing stop_if_lin_vel_walking().")
    # 線速度で歩いている場合ロボットを停止させる
    if pub.is_walking_with_lin_vel_now() and s.last_action_name != "search_ball_by_walking_around":
        pub.send_walk_command(WalkCommand(0.0, 0.0, 0.0))
        logger.action_executed(details="Stopping linear velocity walking")
    else:
        logger.action_postcondition_already_satisfied(details="Not walking with linear velocity")
    return s

def turn_neck_to_detected_ball(s: gtpyhop.State, pub, logger: DecisionLogger):
    """
    ボールが見つかっている場合に首をその方向に向けるアクション。
    """
    if s.world_states["ball_position_relative"] != BALL_POSITION.UNKNOWN:
        print_in_real_action("turn_neck_to_detected_ball", "This action delegate to neck movement.")
        logger.action_executed(details="Delegating to neck movement to turn to detected ball")
        return s
    else:
        logger.action_precondition_not_satisfied(details="Ball not detected, cannot turn neck")
        return None

gtpyhop.declare_actions(
    search_ball_by_turning,
    search_ball_by_neck_movement,
    turn_to_detected_ball,
    stop_if_lin_vel_walking,
    search_ball_by_walking_around,
    turn_neck_to_detected_ball,
    turn_to_shared_ball_position,
)

# ここからタスク ---------------------------------------------------------


def task_search_and_turn_to_ball_method(s: gtpyhop.State, pub, logger: DecisionLogger):
    """
    ボールを探して向きを変えるためのメソッド。
    """
    print_in_task_method("task_search_and_turn_to_ball", "Searching for the ball and turning...")
    # ボールが2秒以上見つからない場合に探索行動を実行する
    if (s.world_states["ball_position_relative"] == BALL_POSITION.UNKNOWN
        and (get_strategy_time() - s.last_visible_ball[1] >= 2.0)):
        logger.task_executed(details="Ball not detected for over 2 seconds, executing search")
        return [("task_select_search", pub, logger)]
    elif (
        s.ball_position_local is not None
        and math.fabs(utils.calculate_angle(s.ball_position_local)) > 1.0472 # 60度以上向きが異なる場合
    ):
        logger.task_executed(details="Ball detected with large angle, turning to face ball")
        return [("turn_neck_to_detected_ball", pub, logger), ("turn_to_detected_ball", pub, logger)]
    else:
        # ボールが見つかっていて、角度が小さい場合はそちらに首を向ける
        logger.task_executed(details="Ball detected with small angle, turning neck to face ball")
        return [("turn_neck_to_detected_ball", pub, logger)]


gtpyhop.declare_task_methods(
    "task_search_and_turn_to_ball", task_search_and_turn_to_ball_method,
)
gtpyhop.declare_task_methods(
    "task_select_search", task_select_search_action_method,
)