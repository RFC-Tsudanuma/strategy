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

import yaml
from ament_index_python.packages import get_package_share_directory

DEFAULT_STRATEGY_DOMAIN = "robocup_strategy"

# 実ロボット上で動作しているかどうかのフラグ
ON_REAL_ROBOT = False

def get_robot_id() -> int:
    """
    実ロボット環境の際に、ロボットIDを取得する。
    :return: ロボットID(1-indexed)
    """
    with open( "/etc/rfc_robot_id", "r") as f:
        id_str = f.readline().strip()
        robot_id = int(id_str)
    return robot_id

try:
    __robot_id = get_robot_id()
    with open(
        get_package_share_directory("strategy") + f"/config/{__robot_id}_strategy_config.yaml", "r"
            ) as f:
        print(
            f"Loading strategy config: {get_package_share_directory('strategy')}/config/{__robot_id}_strategy_config.yaml"
        )
        DEFAULT_STRATEGY_CONFIG = yaml.load(f, Loader=yaml.CLoader)
        ON_REAL_ROBOT = True
except Exception as e:
    print(f"Here is not in real robot. You have to set DEFAULT_STRATEGY_CONFIG manually. Error: {e}")
    DEFAULT_STRATEGY_CONFIG = {}

with open(
    get_package_share_directory("strategy") + "/config/strategy_shared_config.yaml", "r"
        ) as f:
    STRATEGY_SHARED_CONFIG = yaml.load(f, Loader=yaml.CLoader)

def sim_load_strategy_config(sim_robot_id: int):
    """
    シミュレーション用に戦略設定を手動で読み込む。
    :param sim_robot_id: sim用ロボットID(1-indexed)
    """
    global DEFAULT_STRATEGY_CONFIG
    with open(
        get_package_share_directory("strategy") + f"/config/{sim_robot_id}_strategy_config.yaml", "r"
            ) as f:
        print(
            f"Loading strategy config: {get_package_share_directory('strategy')}/config/{sim_robot_id}_strategy_config.yaml"
        )
        DEFAULT_STRATEGY_CONFIG = yaml.load(f, Loader=yaml.CLoader)

# ロボットの速度に関連する定数
ROBOT_MAX_SPEED_X_PLUS = 400 / 4.138  # 4.138秒で4m進むのが最大速度
ROBOT_MAX_SPEED_X_MINUS = -400 / 4.322  # 4.322秒で4m下がるのが最大速度
ROBOT_MAX_SPEED_Y = 400 / 10.55  # 10.55秒で4m進むのが最大速度
ROBOT_MAX_SPEED_PI = 2 * math.pi / 4.492  # 4.492秒で360度回転が最大速度


# 戦略に関する定数
STRATEGY_SLEEP_TIME = 0.1  # 戦略の更新間隔（秒）

# ボールの履歴の長さ(秒数)
POSITION_LOCAL_HISTORY_LENGTH_S = 2
# ボールの履歴の長さ（フレーム数）
POSITION_LOCAL_HISTORY_LENGTH = math.ceil(
    (1.0 / STRATEGY_SLEEP_TIME) * POSITION_LOCAL_HISTORY_LENGTH_S
)

# 実機はm単位なのでそれを変換する係数
CONVERT_SCALE = 100.0

NECK_YAW_MAX_RAD = 1.01229  # 58度


def get_team_id() -> int:
    """
    チームIDを取得する。赤チームは0、青チームは1
    :return: チームID
    """
    if DEFAULT_STRATEGY_CONFIG["common_config"]["team_color"] == "red":
        return 1
    else:
        return 0
    
def get_opponent_team_color() -> str:
    """
    相手チームの色を取得する。
    :return: 相手チームの色 ("red" or "blue")
    """
    if DEFAULT_STRATEGY_CONFIG["common_config"]["team_color"] == "red":
        return "blue"
    else:
        return "red"