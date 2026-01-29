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

import math

import gtpyhop
import utils
from constants import GAME_STATE, ROLE
from print_utils import wrapp_notice
from setting import DEFAULT_STRATEGY_CONFIG
from decision_logger import DecisionLogger


def gc_initial_to_set(s: gtpyhop.State, pub, logger: DecisionLogger):
    """
    GCに応じてREADYまでやる
    """
    if s.world_states["game_state"] in [
        GAME_STATE.INITIAL,
        GAME_STATE.SET,
        GAME_STATE.FINISHED,
    ]:
        logger.task_executed(details="Game INITIAL/SET/FINISHED, executing search_ball_by_neck_movement and gc_state_pause tasks")
        return [("search_ball_by_neck_movement", pub, logger), ("gc_state_pause", pub, logger)]
    elif s.world_states["game_state"] == GAME_STATE.READY:
        logger.task_executed(details="Game READY, executing gc_state_ready task")
        return [("gc_state_ready", pub, logger)]
    else:
        logger.task_precondition_not_satisfied(details="Game state not suitable for gc_initial_to_set")
        return []


def gc_state_pause(s: gtpyhop.State, pub, logger: DecisionLogger):
    """
    GAME_STATEがINITIAL, SET, FINISHEDの場合に実行するメソッド
    """
    if s.world_states["game_state"] in [
        GAME_STATE.INITIAL,
        GAME_STATE.SET,
        GAME_STATE.FINISHED,
    ]:
        logger.task_executed(details="Executing gc_set_command_wait task")
        return [("gc_set_command_wait", pub, logger)]
    logger.task_precondition_not_satisfied(details="Game state not suitable for gc_state_pause")
    return []


# gc_state_readyの修正
def gc_state_ready(s: gtpyhop.State, pub, logger: DecisionLogger):
    """
    GAME_STATEがREADYの場合に実行するメソッド
    """
    if s.world_states["game_state"] == GAME_STATE.READY:
        # 最初の5秒間は目標位置を決めずに直進する
        if s.secondary_time > 40:
            logger.task_executed(details="Executing walk_straight_to_enter_field task")
            return [("walk_straight_to_enter_field", pub, logger)]
        else:
        # その後は目標位置に向かって移動する
            logger.task_executed(details="Executing gc_ready_command_move task")
            return [("gc_ready_command_move", pub, logger)]
    logger.task_precondition_not_satisfied(details="Game state not suitable for gc_state_ready")
    return []


gtpyhop.declare_task_methods("gc_initial_to_set", gc_initial_to_set)
gtpyhop.declare_task_methods("gc_state_pause", gc_state_pause)
gtpyhop.declare_task_methods("gc_state_ready", gc_state_ready)
