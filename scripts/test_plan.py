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

import time

import gtpyhop
import state as state_module
import strategy_runner

# 自分で設定したstateを使って、プランニングの結果を見るためのスクリプト


def main():
    robot_id = 1
    strategy_instance = strategy_runner.Strategy(robot_id)
    # ドメインを設定した後にアクション等をimportする

    # ここ👇はテストのために適当に自分で書く。-------
    state = state_module.default_init_state.copy()
    state.display("[Strategy] Initial state for robot {robot_id}:")

    # 👆ここまで ----------------------------------
    # stateが有効なものかを確認
    if state_module.verify_state(state) is False:
        print(f"[Strategy] Invalid state for robot {robot_id}.")
        print(f"[Strategy] State: {state}")
        strategy_instance.finish_strategy()
        return
    else:
        print(f"[Strategy] Valid state for robot {robot_id}.")
    start = time.perf_counter()
    plan_list = strategy_instance.find_plan(state)
    end = time.perf_counter()

    print(f"[Strategy] Planning took {end - start:.2f} seconds for robot {robot_id}.")
    if plan_list is None or len(plan_list) == 0:
        print(f"[Strategy] No plan found for robot {robot_id}.")
        return
    else:
        print(f"[Strategy] Plan found for robot {robot_id}: {plan_list}")
        for action in plan_list:
            print(f"[Strategy] Actions : {action}")
    strategy_instance.finish_strategy()


if __name__ == "__main__":
    main()
