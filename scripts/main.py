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


import argparse
import time
import traceback
import rclpy
import setting
from print_utils import print_in_strategy, debug_print
from strategy_time import get_strategy_time

# コンフィグ読み込みとの兼ね合いがあるため、ここではsettingとprint_utils以外はインポートしない。

def main(sim_robot_id=None):
    if sim_robot_id is not None:
        # シミュレーションの場合は読み込むコンフィグを手動で設定
        setting.sim_load_strategy_config(sim_robot_id)
    # コンフィグ読み込みとの兼ね合いで遅延インポートする
    import strategy_runner
    strategy = strategy_runner.Strategy(sim_robot_id=sim_robot_id)
    plan_list = None
    prev_time = get_strategy_time()
    last_action_name = ""
    try:
        while rclpy.ok() and not strategy.finish:
            do_sleep = True
            strategy.check_standup()
            state = (
                strategy.get_world_state(last_action_name=last_action_name)
            )  # ここで実数値も含めてステートが更新される。
            plan_list = strategy.find_plan(state)
            if (plan_list is not None) and (plan_list[-1] is not None):
                last_action_name = plan_list[-1][0]
            strategy.send_decision_log()  # 意思決定ログの送信
            elapsed_time = get_strategy_time() - prev_time
            # スリープ処理
            if (
                do_sleep
                and elapsed_time < setting.STRATEGY_SLEEP_TIME
            ):
                sleep_time = max(0.0, setting.STRATEGY_SLEEP_TIME - elapsed_time - 0.0002)
                time.sleep(sleep_time) # 少し余裕を持たせる
            prev_time = get_strategy_time()
    except KeyboardInterrupt:
        print_in_strategy("Received KeyboardInterrupt (SIGINT). Shutting down...")
    except Exception as e:
        print_in_strategy(f"Unexpected error occurred: {e}")
        traceback.print_exc()
    finally:
        print_in_strategy("Strategy finished.")
        strategy.finish_strategy()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Strategy runner")
    parser.add_argument(
        "--sim-robot-id",
        type=int,
        default=None,
        help="Simulation Robot ID (default: None)",
    )
    args = parser.parse_args()
    main(args.sim_robot_id)
