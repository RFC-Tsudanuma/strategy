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
# - Masafumi Horiguchi


import gtpyhop
from constants import GAME_STATE, KICKOFF_STATE, PENALTY, ROLE
from real_action.action_functions import change_role, need_waiting_at_kickoff
from setting import DEFAULT_STRATEGY_CONFIG
from decision_logger import DecisionLogger


def select_role_method(s: gtpyhop.State, pub, logger: DecisionLogger):
    """
    ゲームの状態に応じてロボットの役割（ROLE）を選択するメソッド。
    """
    if (
        s.world_states["game_state"] == GAME_STATE.INITIAL
        or s.world_states["game_state"] == GAME_STATE.READY
        or s.world_states["game_state"] == GAME_STATE.SET
        or s.world_states["game_state"] == GAME_STATE.FINISHED
        or s.world_states["game_state"] == GAME_STATE.IMPOSSIBLE
        or s.world_states["kickoff_state"] == KICKOFF_STATE.WAIT
    ):
        if s.role != ROLE.NEUTRAL:
            change_role(ROLE.NEUTRAL)  # roleを切り替える必要があるなら切り替える
            logger.task_executed(details="Change role to NEUTRAL")
        else:
            logger.task_executed(details="Role select => NEUTRAL")
        return [("role_neutral", pub, logger)]

    else:
        # ペナルティが掛かっている時はニュートラルにする
        if s.world_states["penalty"] != PENALTY.NONE:
            if s.role != ROLE.NEUTRAL:
                change_role(ROLE.NEUTRAL)
                logger.task_executed(details="Change role to NEUTRAL")
            else:
                logger.task_executed(details="Role select => NEUTRAL")
            return [("role_neutral", pub, logger)]

        # キックオフの際に待機が必要な場合は待機する
        if need_waiting_at_kickoff(s):
            if s.role != ROLE.NEUTRAL:
                change_role(ROLE.NEUTRAL)
                logger.task_executed(details="Change role to NEUTRAL")
            else:
                logger.task_executed(details="Role select => NEUTRAL")
            return [("do_nothing", pub, logger)]

        if DEFAULT_STRATEGY_CONFIG["common_config"]["is_keeper"]:
            # if s.role != ROLE.KEEPER:
            #     change_role(ROLE.KEEPER)
            # return [("keeper_task", pub, logger)]
            # キーパーとディフェンダーは実質同じなので、キーパーの時にディフェンダーになる
            if s.role != ROLE.DEFENDER:
                change_role(ROLE.DEFENDER)
                logger.task_executed(details="Change role to DEFENDER")
            else:
                logger.task_executed(details="Role select => DEFENDER")
            return [("defender_task_select", pub, logger)]
        else:
            if s.role != ROLE.ATTACKER:
                change_role(ROLE.ATTACKER)
                logger.task_executed(details="Change role to ATTACKER")
            else:
                logger.task_executed(details="Role select => ATTACKER")
            return [("attacker_task_select", pub, logger)]
        # TODO: defenderとattackerの切り替えを実装する


gtpyhop.declare_task_methods("select_role", select_role_method)
