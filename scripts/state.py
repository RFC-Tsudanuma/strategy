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

# - Haruki Ogawa

from typing import Optional

import gtpyhop
from constants import (
    BALL_POSITION,
    CONTROLLER_BUTTON,
    GAME_STATE,
    KICKOFF_STATE,
    OUR_FIELD,
    PENALTY,
    ROBOT_FIELD,
    ROBOT_MODE,
    ROLE,
    SECONDARY_STATE,
    TEAM_COLOR,
)
from print_utils import print_in_strategy


class RoleChangeException(Exception):
    """
    ロールを変更する時にアクションが投げる例外
    """

    def __init__(self, new_role: ROLE):
        super().__init__(f"Role change requested to {new_role}")
        self.new_role: ROLE = new_role


class Position:
    COORD_GLOBAL = "global"
    COORD_LOCAL = "local"
    """
    Represents a position in the field.
    """

    def __init__(self, coord: str, x: float, y: float, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.coord = coord

    def __repr__(self):
        return f"Position({self.x}, {self.y}, {self.theta}, {self.coord})"


class RobotLowState:
    def __init__(
        self, neck_angle=0.0, neck_pitch_angle=0.0, roll=0.0, pitch=0.0, yaw=0.0
    ):
        self.neck_angle: float = neck_angle  # 首の角度
        self.neck_pitch_angle: float = neck_pitch_angle
        self.roll: float = roll  # ロールの角度
        self.pitch: float = pitch  # ピッチの角度
        self.yaw: float = yaw  # ヨーの角度


class SharedMessage:
    """
    情報共有のデータを表すクラス
    """

    def __init__(self, robot_id: int, ballpos, selfpos, role, ballpos_local=None):
        self.ballpos: Optional[Position] = ballpos
        self.ballpos_local: Optional[Position] = ballpos_local
        self.selfpos: Optional[Position] = selfpos
        self.role: ROLE = role
        self.robot_id = robot_id

    def __repr__(self) -> str:
        return f"SharedMessage(robot_id={self.robot_id}, ballpos={self.ballpos}, selfpos={self.selfpos}, role={self.role})"


class ObjectGlobalPositions:
    """
    履歴として利用するための、一度の観測で取得したオブジェクトのグローバル座標のリスト。
    """

    def __init__(self):
        self.ball_pos_global: list[Position] = []
        self.goalpost_pos_global: list[Position] = []


possible_world_states = gtpyhop.State("Possible_world_states")
# 世界が取り得る状態を定義
possible_world_states.world_states = {
    # ロボットに対するボールの位置関係。キックレンジ内、近い、遠い、未知
    "ball_position_relative": [
        BALL_POSITION.IN_KICK_RANGE,
        BALL_POSITION.NEAR,
        BALL_POSITION.DISTANT,
        BALL_POSITION.UNKNOWN,
    ],
    # ロボットのフィールド上の位置。味方フィールド、相手フィールド
    "current_robot_field": [
        ROBOT_FIELD.ALLY_FIELD,
        ROBOT_FIELD.OPPONENT_FIELD,
        ROBOT_FIELD.UNKNOWN,
    ],
    #  試合の状態。試合中、準備中、セットプレー、フリーキック、ペナルティキック
    "game_state": [
        GAME_STATE.IMPOSSIBLE,
        GAME_STATE.INITIAL,
        GAME_STATE.READY,
        GAME_STATE.SET,
        GAME_STATE.PLAYING,
        GAME_STATE.FINISHED,
    ],
    # セカンダリーステート
    "secondary_state": [
        SECONDARY_STATE.UNKNOWN,
        SECONDARY_STATE.NORMAL,
        SECONDARY_STATE.PENALTYSHOOT,
        SECONDARY_STATE.OVERTIME,
        SECONDARY_STATE.TIMEOUT,
        SECONDARY_STATE.DIRECT_FREEKICK,
        SECONDARY_STATE.INDIRECT_FREEKICK,
        SECONDARY_STATE.PENALTYKICK,
        SECONDARY_STATE.CORNER_KICK,
        SECONDARY_STATE.GOAL_KICK,
        SECONDARY_STATE.THROW_IN,
    ],
    "penalty": [
        PENALTY.UNKNOWN,
        PENALTY.NONE,
        PENALTY.SUBSTITUTE,
        PENALTY.MANUAL,
        PENALTY.BALL_MANIPULATION,
        PENALTY.PUSHING,
        PENALTY.PICKUP_UNCAPABLE,
        PENALTY.SERVICE,
    ],
    "kickoff_state": [
        KICKOFF_STATE.WAIT,
        KICKOFF_STATE.START,
    ],
    # 自分たちのフィールドに対する認識。右側か左側か。
    "our_field": [OUR_FIELD.RIGHT, OUR_FIELD.LEFT],
    # チームカラー
    "team_color": [TEAM_COLOR.RED, TEAM_COLOR.BLUE],
}
possible_world_states.secsTillUnpenalised = 0.0  # ペナルティが解除されるまでの秒数
# キックオフするチームの番号
possible_world_states.kickOffTeam = 3
# 戦略が取り得るロール。アタッカー、ディフェンダー、ニュートラル
possible_world_states.role = [ROLE.ATTACKER, ROLE.DEFENDER, ROLE.NEUTRAL, ROLE.KEEPER]
# ロボットの切り替え可能なモード。試合はsoccer_gameモードで、
# それ以外にデバッグやペナルティキックなどをするならモードを切り替える。
# デバッグモードはロールを変更せず、初期状態で設定されたロールを維持する。
# これは最上位のステート
possible_world_states.robot_mode = [
    ROBOT_MODE.SOCCER_GAME,
    ROBOT_MODE.NO_GC,
    ROBOT_MODE.PENALTY_KICK,
    ROBOT_MODE.STOP,
]
# 最後に受けたペナルティの種類
possible_world_states.last_penalty = PENALTY.NONE
# 最後にペナルティが解除された時の時間（現在時刻からこれを引くと、経過時間が出る）
possible_world_states.last_unpenalized_time = 0.0
possible_world_states.first_half = True

# 以下はロボットの位置等、実数値で表される状態
possible_world_states.self_position_global = Position(
    Position.COORD_GLOBAL, 0.0, 0.0, 0.0
)
possible_world_states.ball_position_local = Position(
    Position.COORD_LOCAL, 0.0, 0.0, 0.0
)
possible_world_states.shared_ball_global = Position(
    Position.COORD_GLOBAL, 0.0, 0.0, 0.0
)
possible_world_states.opponent_position_local = [
    Position(Position.COORD_LOCAL, 0.0, 0.0, 0.0),
    Position(Position.COORD_LOCAL, 0.0, 0.0, 0.0),
    Position(Position.COORD_LOCAL, 0.0, 0.0, 0.0),
]
possible_world_states.allies_position_local = [
    Position(Position.COORD_LOCAL, 0.0, 0.0, 0.0),
    Position(Position.COORD_LOCAL, 0.0, 0.0, 0.0),
]
# 青でも赤でも無いロボットは単なる障害物とする
possible_world_states.normal_obstacles_position_local = [
    Position(Position.COORD_LOCAL, 0.0, 0.0, 0.0),
    Position(Position.COORD_LOCAL, 0.0, 0.0, 0.0),
    Position(Position.COORD_LOCAL, 0.0, 0.0, 0.0),
    # ここはいくらでも入る可能性がある
]
possible_world_states.opponent_goalpost_local = [
    Position(Position.COORD_LOCAL, 0.0, 0.0, 0.0),
]
possible_world_states.ally_goalpost_local = [
    Position(Position.COORD_LOCAL, 0.0, 0.0, 0.0),
]
possible_world_states.opponent_goalpost_global = [
    Position(Position.COORD_GLOBAL, 0.0, 0.0, 0.0),
]
possible_world_states.neck_yaw_rad = 0.0  # 首の角度（ラジアン）
possible_world_states.secondary_time = 0.0  # セカンダリーステートの時間（秒）
possible_world_states.secs_remaining = 600  # ハーフの残り時間（秒）
possible_world_states.object_position_history = (
    None  # オブジェクトの位置履歴（ローカル座標）これはNoneも入るので注意！
)
possible_world_states.last_visible_ball = (None,0.0)  # 最後に見えたボールの位置（グローバル座標）とその時の時間
# 👆これはどんなに古くても必ず入るので注意！比較的新しいlast_visible_ballが欲しいならget_last_visible_ball_from_history()
# を使うこと
possible_world_states.robot_low_state = (
    RobotLowState()
)  # ロボットの低レベル状態（首の角度など）
possible_world_states.shared_msg = [None, None, None]
possible_world_states.secondary_state_info = [0,0,0,0]  # 4byteのcharのデータ
possible_world_states.last_action_name = ""  # 最後に実行したアクションの名前
possible_world_states.last_strategy_process_time = 0.0  # 最後に戦略処理の1ループに掛かった時間（秒）

default_init_state = gtpyhop.State("current_world_state")
default_init_state.world_states = {
    "ball_position_relative": BALL_POSITION.UNKNOWN,
    "current_robot_field": ROBOT_FIELD.UNKNOWN,
    "game_state": GAME_STATE.INITIAL,  # PLAYING,
    "secondary_state": SECONDARY_STATE.UNKNOWN,
    "our_field": OUR_FIELD.LEFT, # これは常にLEFTである。RIGHTになることは無い
    "team_color": TEAM_COLOR.BLUE,
    "kickoff_state": KICKOFF_STATE.WAIT,
    "penalty": PENALTY.UNKNOWN,
}
default_init_state.kickOffTeam = possible_world_states.kickOffTeam
default_init_state.role = ROLE.NEUTRAL  # attacker
default_init_state.robot_mode = ROBOT_MODE.SOCCER_GAME
default_init_state.last_penalty = PENALTY.NONE
default_init_state.last_unpenalized_time = 0.0
default_init_state.first_half = True
default_init_state.self_position_global = None
default_init_state.ball_position_local = None
default_init_state.shared_ball_global = None
default_init_state.opponent_position_local = []
default_init_state.allies_position_local = []
default_init_state.normal_obstacles_position_local = []
default_init_state.opponent_goalpost_local = []
default_init_state.ally_goalpost_local = []
default_init_state.opponent_goalpost_global = []
default_init_state.neck_yaw_rad = 0.0
default_init_state.secondary_time = 0.0
default_init_state.secs_remaining = 600
default_init_state.object_position_history = None
default_init_state.robot_low_state = None  # 初期状態のロボットの低レベル状態
default_init_state.secsTillUnpenalised = 0
default_init_state.shared_msg = [None, None, None]
default_init_state.secondary_state_info = [0,0,0,0]
default_init_state.last_visible_ball = (None, 0.0)
default_init_state.last_action_name = ""
default_init_state.last_strategy_process_time = 0.0

# 定義した状態が有効かどうかを確認する関数
def verify_state(state):
    """
    Verify if the state is valid according to the defined world states.
    """
    if not isinstance(state, gtpyhop.State):
        return False

    # Check if state has role attribute and validate it
    if hasattr(state, "role"):
        if state.role not in possible_world_states.role:
            print_in_strategy(f"Invalid role: {state.role}")
            return False

    # Check if state has robot_mode attribute and validate it
    if hasattr(state, "robot_mode"):
        if state.robot_mode not in possible_world_states.robot_mode:
            print_in_strategy(f"Invalid robot_mode: {state.robot_mode}")
            return False

    # Check if state has world_states attribute and validate it
    if hasattr(state, "world_states"):
        if not isinstance(state.world_states, dict):
            print_in_strategy("world_states must be a dictionary")
            return False

        # Validate each key-value pair in world_states
        for key, value in state.world_states.items():
            if key not in possible_world_states.world_states:
                print_in_strategy(f"Invalid key in world_states: {key}")
                return False
            if value not in possible_world_states.world_states[key]:
                print_in_strategy(f"Invalid value for {key}: {value}")
                return False

    return True


# 定義したステートに問題が無いかを判別する
if not verify_state(default_init_state):
    raise ValueError("[state.py] Invalid initial state.")
