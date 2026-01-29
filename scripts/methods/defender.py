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
from constants import SECONDARY_STATE, FIELD_DIMENTIONS
from print_utils import print_in_task_method
from decision_logger import DecisionLogger
from strategy_time import get_strategy_time
import utils
import numpy as np

def is_ball_in_danger_zone(s: gtpyhop.State) -> bool:
    """
    ボールが危険ゾーンにあるかどうかを判定する関数
    危険ゾーンにあればディフェンダーは何らかの対応の必要がある
    """
    # 情報が無い場合はFalse
    if s.ball_position_local is None or s.self_position_global is None:
        return False

    # 自陣ゴール前300cm以内を危険ゾーンとする
    DANGER_ZONE_X = -FIELD_DIMENTIONS.X_LENGTH / 2 + 300.0
    # 中央直径400cmを危険ゾーンとする
    DANGER_ZONE_Y = 200.0
    ball_gl = utils.convert_local_to_global(s.self_position_global, s.ball_position_local)
    ball_x, ball_y = ball_gl.x, ball_gl.y
    if ball_x < DANGER_ZONE_X and abs(ball_y) < DANGER_ZONE_Y:
        return True
    return False

def vector_robot_to_ball_directs_goal(s: gtpyhop.State) -> bool:
    """
    ロボットからボールへのベクトルがゴール方向かどうかを判定する関数
    """
    if s.ball_position_local is None or s.self_position_global is None:
        return False
    # ボールへのベクトル
    vec_ball = np.array([s.ball_position_local.x, s.ball_position_local.y])
    # ロボットからゴールポールへのベクトル
    pos_a,pos_b = utils.get_goalpost_local(s)[0], utils.get_goalpost_local(s)[1]
    true_post_a_vec = np.array([-700.0 - pos_a.x, 130.0 - pos_a.y])
    true_post_b_vec = np.array([-700.0 - pos_b.x, -130.0 - pos_b.y])
    # ボールのベクトルが上の2つのベクトルの間にあるならゴール方向を向かっている
    cross_a = np.cross(true_post_a_vec, vec_ball)
    cross_b = np.cross(vec_ball, true_post_b_vec)
    if (cross_a * cross_b >= 0) and ((cross_a >= 0) == (cross_b >= 0)):
        return True
    return False

def defender_task_method(s, pub, logger: DecisionLogger):
    """
    Define the actions for the defender role.
    """
    # ペナライズから復帰して3秒間は指定位置に移動する
    if get_strategy_time() - s.last_unpenalized_time < 3.0:
        print_in_task_method(
            "defender_task",
            "Recently unpenalized. Walking to default position.",
        )
        logger.task_executed(details="Recently unpenalized, moving to default position")
        return [("walk_to_default_position", pub, logger)]
    # ボールが近くにあって、ゴール内に向かう方向でなければタックルする
    if (is_ball_in_danger_zone(s) and not vector_robot_to_ball_directs_goal(s)):
        print_in_task_method("defender_task", "Ball in danger zone. Tackling.")
        logger.task_executed(details="Ball in danger zone, executing tackle_ball task")
        return [("tackle_ball", pub, logger)]
    # 特に問題ない場合は、移動して守りやすいポジションを取り続ける
    logger.task_executed(details="Approaching defense position")
    return [
        ("task_search_and_turn_to_ball", pub, logger),
        ("approach_defense_position", pub, logger),
    ]


def defender_task_select_method(s: gtpyhop.State, pub, logger: DecisionLogger):
    if s.secondary_time == 0 and s.world_states["secondary_state"] in [
        SECONDARY_STATE.DIRECT_FREEKICK,
        SECONDARY_STATE.INDIRECT_FREEKICK,
        SECONDARY_STATE.PENALTYKICK,
        SECONDARY_STATE.CORNER_KICK,
        SECONDARY_STATE.GOAL_KICK,
        SECONDARY_STATE.THROW_IN,
    ]:
        logger.task_executed(details="Executing do_nothing")
        return [("do_nothing", pub, logger)]
    elif s.world_states["secondary_state"] in [
        SECONDARY_STATE.DIRECT_FREEKICK,
        SECONDARY_STATE.INDIRECT_FREEKICK,
        SECONDARY_STATE.PENALTYKICK,
        SECONDARY_STATE.CORNER_KICK,
        SECONDARY_STATE.GOAL_KICK,
        SECONDARY_STATE.THROW_IN,
    ]:
        logger.task_executed(details="Executing secondary_defender_task due to secondary state")
        return [("secondary_defender_task", pub, logger)]
    else:
        logger.task_executed(details="Executing defender_task")
        return [("defender_task", pub, logger)]


def secondary_defender_task_method(s: gtpyhop.State, pub, logger: DecisionLogger):
    if s.world_states["secondary_state"] in [
        SECONDARY_STATE.DIRECT_FREEKICK,
        SECONDARY_STATE.INDIRECT_FREEKICK,
        SECONDARY_STATE.PENALTYKICK,
        SECONDARY_STATE.CORNER_KICK,
        SECONDARY_STATE.GOAL_KICK,
        SECONDARY_STATE.THROW_IN,
    ]:
        logger.task_executed(details="Approaching secondary defense position")
        return [("approach_secondary_defense_position", pub, logger)]
    logger.task_precondition_not_satisfied(details="Secondary state not suitable for secondary defender task")
    return []


# 今の所全部同じ場所
def secondary_defender_approuch_pos_method(s, pub, logger: DecisionLogger):
    """
    フリーキックに指定の場所に移動するタスク
    """
    logger.task_executed(details="Approaching secondary defense position")
    return [
        ("approach_secondary_defense_position", pub, logger),
    ]


gtpyhop.declare_task_methods("defender_task", defender_task_method)
gtpyhop.declare_task_methods("secondary_defender_task", secondary_defender_task_method)
gtpyhop.declare_task_methods("defender_task_select", defender_task_select_method)