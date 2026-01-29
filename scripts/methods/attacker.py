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
# - Suzuha Kiuchi
# - Yugo Nishio


import gtpyhop
import setting
from constants import BALL_POSITION, ROBOT_FIELD, SECONDARY_STATE
from print_utils import print_in_task_method
from utils import calculate_distance_global, convert_local_to_global, calculate_distance, get_shared_ally_ballpos_local
from strategy_time import get_strategy_time
from utils import get_shared_ally_attackers_positions
from decision_logger import DecisionLogger

from methods.follower import follower_task_method


def attacker_task_method(s, pub, logger: DecisionLogger):
    """
    Define the actions for the attacker role.
    """

    # ペナライズから復帰して4秒間は指定位置に移動する
    if get_strategy_time() - s.last_unpenalized_time < 4.0:
        print_in_task_method(
            "attacker_task",
            "Recently unpenalized. Walking to default position.",
        )
        logger.task_executed(details="Recently unpenalized, moving to default position")
        return [("walk_to_default_position", pub, logger)]

    # ボールが見つかっていない場合は、ボールを探すタスクを実行
    if s.ball_position_local is None or s.self_position_global is None:
        print_in_task_method("attacker_task", "No ball or position data. Searching.")
        logger.task_executed(details="Ball or self position unknown, executing search and turn to ball task")
        return [("task_search_and_turn_to_ball", pub, logger)]

    # ボールが見つかっている場合はattackerになる
    if s.ball_position_local is not None:
        ball_gl = convert_local_to_global(s.self_position_global, s.ball_position_local)

        # フォロワーの役割に切り替える条件を追加
        # s.allies_position_local が存在し、かつ自分とボールの距離より味方とボールの距離が近い場合
        shared_balls = get_shared_ally_ballpos_local(s)
        ally_id = 0
        for ally_ballpos_local in shared_balls:
            ally_id += 1
            if ally_ballpos_local is None:
                continue
            diff_ball_allies = calculate_distance(ally_ballpos_local)
            diff_ball_self_position = calculate_distance(s.ball_position_local)
            # idが低い方が優先
            id_priority_cm = ally_id * 100
            self_priority_cm = setting.DEFAULT_STRATEGY_CONFIG["common_config"]["robot_id"] * 100

            if (diff_ball_allies + id_priority_cm) < (diff_ball_self_position + self_priority_cm):
                # 自分よりも味方がボールに近い場合は、フォロワーとして振る舞う
                logger.task_executed(details=f"Ally {ally_id} is closer to ball, switching to follower role")
                return [("follower_task", pub, logger)]

        if (
            s.world_states["current_robot_field"] == ROBOT_FIELD.ALLY_FIELD
            and ball_gl.x > s.self_position_global.x + 100
        ):  # 1m以上後ろにいる時
            logger.task_executed(details="Executing tackle_ball task")
            return [("tackle_ball", pub, logger)]
        else:
            logger.task_executed(details="Executing task_kick_goal sequence")
            return [("task_kick_goal", pub, logger)]


def attacker_task_select_method(s: gtpyhop.State, pub, logger: DecisionLogger):
    if s.secondary_time == 0 and s.world_states["secondary_state"] in [
        SECONDARY_STATE.DIRECT_FREEKICK,
        SECONDARY_STATE.INDIRECT_FREEKICK,
        SECONDARY_STATE.PENALTYKICK,
        SECONDARY_STATE.CORNER_KICK,
        SECONDARY_STATE.GOAL_KICK,
        SECONDARY_STATE.THROW_IN,
    ]:
        logger.task_executed(details="Executing do_nothing task due to secondary state and time conditions")
        return [("do_nothing", pub, logger)]
    # secondary.infoで自分ボールか相手ボールか分ける
    # こっちmyball
    elif s.world_states["secondary_state"] in [
        SECONDARY_STATE.DIRECT_FREEKICK,
        SECONDARY_STATE.INDIRECT_FREEKICK,
        SECONDARY_STATE.PENALTYKICK,
        SECONDARY_STATE.CORNER_KICK,
        SECONDARY_STATE.GOAL_KICK,
        SECONDARY_STATE.THROW_IN,
    ]:
        # 自分ボール
        if s.secondary_state_info[0] == setting.get_team_id():
            logger.task_executed(details="My balls")
            return [("secondary_attacker_myball_task", pub, logger)]
        # 相手ボール
        else:
            logger.task_executed(details="Enemy ball")
            return [("secondary_attacker_enemyball_task", pub, logger)]
    else:
        logger.task_executed(details="Execute attacker task")
        return [("attacker_task", pub, logger)]


def secondary_attacker_myball_task_method(s: gtpyhop.State, pub, logger: DecisionLogger):
    # secondary.infoで自分ボールの場合を追加する
    if s.world_states["secondary_state"] in [
        SECONDARY_STATE.DIRECT_FREEKICK,
        SECONDARY_STATE.INDIRECT_FREEKICK,
        SECONDARY_STATE.PENALTYKICK,
        SECONDARY_STATE.CORNER_KICK,
        SECONDARY_STATE.GOAL_KICK,
        SECONDARY_STATE.THROW_IN,
    ]:
        if s.ball_position_local is not None and s.self_position_global is not None:
            if s.allies_position_local:
                ball_gl = convert_local_to_global(s.self_position_global, s.ball_position_local)
                first_ally_local_position = s.allies_position_local[0]
                allies_gl = convert_local_to_global(s.self_position_global, first_ally_local_position)

                diff_ball_alies = calculate_distance_global(ball_gl, allies_gl)
                diff_ball_self_position = calculate_distance_global(ball_gl, s.self_position_global)

                if diff_ball_alies < diff_ball_self_position:
                    # 自分よりも味方がボールに近い場合は、フォロワーとして振る舞う
                    logger.task_executed(details="Ally is closer to ball, switching to follower role")
                    return [("approach_secondary_attacker_follower", pub, logger)]

            logger.task_executed(details="Executing task_kick_goal sequence for my ball")
            return [
                ("task_search_and_turn_to_ball", pub, logger),
                ("goal_kick_ball_position", pub, logger),
                ("goal_kick_ball_around", pub, logger),
                ("adjust_to_kick_position", pub, logger),
                # ("goal_kick_ball_direction", pub, logger),
            ]
        else:
            logger.task_executed(details="Ball or self position unknown, executing search and turn to ball task")
            return  [("task_search_and_turn_to_ball", pub, logger)]
    logger.task_precondition_not_satisfied(details="Secondary state not suitable for my ball task")
    return []


def secondary_attacker_enemyball_task_method(s: gtpyhop.State, pub, logger: DecisionLogger):
    # secondary.infoで敵ボールの場合を追加する
    if s.world_states["secondary_state"] in [
        SECONDARY_STATE.DIRECT_FREEKICK,
        SECONDARY_STATE.INDIRECT_FREEKICK,
        SECONDARY_STATE.PENALTYKICK,
        SECONDARY_STATE.CORNER_KICK,
        SECONDARY_STATE.GOAL_KICK,
        SECONDARY_STATE.THROW_IN,
    ]:
        if s.ball_position_local is not None and s.self_position_global is not None:
            if s.allies_position_local:
                ball_gl = convert_local_to_global(s.self_position_global, s.ball_position_local)
                first_ally_local_position = s.allies_position_local[0]
                allies_gl = convert_local_to_global(s.self_position_global, first_ally_local_position)

                diff_ball_alies = calculate_distance_global(ball_gl, allies_gl)
                diff_ball_self_position = calculate_distance_global(ball_gl, s.self_position_global)

                # if diff_ball_alies < diff_ball_self_position:
                #     # 自分よりも味方がボールに近い場合は、フォロワーとして振る舞う
                #     return [("approach_secondary_attacker_follower", pub)]

            logger.task_executed(details="Executing approach to enemy ball position")
            return [
                ("task_search_and_turn_to_ball", pub, logger),
                ("approach_secondary_attacker_enemyball_position", pub, logger),
            ]
        else:
            logger.task_executed(details="Ball or self position unknown, executing search and turn to ball task")
            return  [("task_search_and_turn_to_ball", pub, logger)]
    logger.task_precondition_not_satisfied(details="Secondary state not suitable for enemy ball task")
    return []


gtpyhop.declare_task_methods("attacker_task", attacker_task_method)
gtpyhop.declare_task_methods("attacker_task_select", attacker_task_select_method)
gtpyhop.declare_task_methods(
    "secondary_attacker_myball_task", secondary_attacker_myball_task_method
)
gtpyhop.declare_task_methods(
    "secondary_attacker_enemyball_task", secondary_attacker_enemyball_task_method
)


# task


def task_kick_goal(s: gtpyhop.State, pub, logger: DecisionLogger):
    """
    ゴールに向かってボールを蹴るタスク
    条件：ボールを知っていること,自己位置を知っていること
    """
    if (
        s.world_states["current_robot_field"] != BALL_POSITION.UNKNOWN
        and s.self_position_global is not None
    ):
        logger.task_executed(details="Executing task_kick_goal sequence")
        return [
            ("task_search_and_turn_to_ball", pub, logger),
            ("goal_kick_ball_position", pub, logger),
            ("goal_kick_ball_around", pub, logger),
            ("adjust_to_kick_position", pub, logger),
            # ("goal_kick_ball_direction", pub, logger),
            ("kick_ball", pub, logger),
        ]
    logger.task_precondition_not_satisfied(details="Ball position or self position unknown, cannot execute task_kick_goal")


gtpyhop.declare_task_methods("task_kick_goal", task_kick_goal)
