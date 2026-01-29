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
# - Masafumi Horiguchi

import gtpyhop
import state
from print_utils import print_in_strategy
from real_action.action_functions import change_role
from decision_logger import DecisionLogger


def select_mode_method(s: gtpyhop.State, pub, logger: DecisionLogger):
    """
    Select the mode based on the game state.
    """
    print_in_strategy(f"[Current role]: {s.role}")
    if s.robot_mode == state.ROBOT_MODE.STOP:
        logger.task_executed(details="Robot mode STOP")
        return [("do_nothing", pub, logger)]
    elif s.robot_mode == state.ROBOT_MODE.SOCCER_GAME:
        logger.task_executed(details="Robot mode SOCCER_GAME")
        return [("select_role", pub, logger)]
    elif s.robot_mode == state.ROBOT_MODE.NO_GC:
        if s.role != state.ROLE.ATTACKER:
            change_role(state.ROLE.ATTACKER)
            logger.task_executed(details="Change role to ATTACKER due to NO_GC mode")
        else:
            logger.task_executed(details="Robot mode NO_GC")
        return [("attacker_task", pub, logger)]
    elif s.robot_mode == state.ROBOT_MODE.PENALTY_KICK:
        logger.task_executed(details="Robot mode PENALTY_KICK")
        return [("penalty_kick_mode", pub, logger)]


gtpyhop.declare_task_methods("select_mode", select_mode_method)
