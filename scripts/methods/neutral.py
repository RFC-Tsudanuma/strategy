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

import gtpyhop
from decision_logger import DecisionLogger
import constants

def role_neutral_method(s: gtpyhop.State, pub, logger: DecisionLogger):
    """
    Neutral method for the robot.
    This method is called when the robot is in a neutral state.
    """
    if s.world_states["game_state"] == constants.GAME_STATE.FINISHED:
        logger.task_executed(details="Game finished, executing do_nothing task")
        return [("do_nothing",pub, logger)]
    # ペナルティを受けている場合は、解除されるまで待機
    elif s.world_states["penalty"] != constants.PENALTY.NONE:
        logger.task_executed(details="Robot penalized, executing wait_until_unpenalized task")
        return [("wait_until_unpenalized", pub, logger)]
    logger.task_executed(details="Game INITIAL/SET/READY, executing gc_initial_to_set task")
    return [("gc_initial_to_set", pub, logger)]


gtpyhop.declare_task_methods("role_neutral", role_neutral_method)
