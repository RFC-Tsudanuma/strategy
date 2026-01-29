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
import setting
not_a_real_robot = False   #実験する時はここを変えてね
if not_a_real_robot:
    setting.sim_load_strategy_config(1)
from game_communications.brain_communication import BrainCommunication


if __name__ == "__main__":
    # BrainCommunicationのインスタンスを作成
    brain_comm = BrainCommunication(is_simulation=not_a_real_robot)
    brain_comm.init_udp_broadcast()
    brain_comm.init_game_controller_broadcast()

    try:
        # メインループを実行
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Shutting down BrainCommunication...")
    finally:
        brain_comm.cleanup()
        print("BrainCommunication has been shut down.")