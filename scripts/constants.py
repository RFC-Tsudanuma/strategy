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

import math
from enum import Enum


class ROBOT_FIELD(Enum):
    ALLY_FIELD = "ally_field"
    OPPONENT_FIELD = "opponent_field"
    UNKNOWN = "unknown"


class GAME_STATE(Enum):
    IMPOSSIBLE = "impossible"
    INITIAL = "initial"
    READY = "ready"
    SET = "set"
    PLAYING = "playing"
    FINISHED = "finished"


class SECONDARY_STATE(Enum):
    UNKNOWN = "unknown"
    NORMAL = "normal"
    PENALTYSHOOT = "penaltyshoot"
    OVERTIME = "overtime"
    TIMEOUT = "timeout"
    DIRECT_FREEKICK = "direct_freekick"
    INDIRECT_FREEKICK = "indirect_freekick"
    PENALTYKICK = "penalty_kick"
    CORNER_KICK = "corner_kick"
    GOAL_KICK = "goal_kick"
    THROW_IN = "throw_in"


class PENALTY(Enum):
    UNKNOWN = "unknown"
    NONE = "none"
    SUBSTITUTE = "substitute"
    MANUAL = "manual"
    BALL_MANIPULATION = "ball_manipulation"
    PUSHING = "pushing"
    PICKUP_UNCAPABLE = "pickup_uncapable"
    SERVICE = "service"


class BALL_POSITION(Enum):
    IN_KICK_RANGE = "in_kick_range"
    NEAR = "near"
    DISTANT = "distant"
    UNKNOWN = "unknown"


class ROLE(Enum):
    ATTACKER = "attacker"
    DEFENDER = "defender"
    NEUTRAL = "neutral"
    KEEPER = "keeper"


class ROBOT_MODE(Enum):
    SOCCER_GAME = "soccer_game"
    NO_GC = "no_gc_mode"
    PENALTY_KICK = "penalty_kick"
    STOP = "stop_mode"


class OUR_FIELD(Enum):
    RIGHT = lambda x_pos: x_pos > 0
    LEFT = lambda x_pos: x_pos < 0
    UNKNOWN = None


class KICKOFF_STATE(Enum):
    WAIT = "wait"
    START = "start"


class CONTROLLER_BUTTON(Enum):
    NONE = "none"
    BUTTON_A_PUSHED = "a"
    BUTTON_B_PUSHED = "b"
    BUTTON_X_PUSHED = "x"
    BUTTON_Y_PUSHED = "y"
    BUTTON_R1_PUSHED = "r1"
    BUTTON_R2_PUSHED = "r2"
    BUTTON_L1_PUSHED = "l1"
    BUTTON_L2_PUSHED = "l2"
    BUTTON_START_PUSHED = "start"
    BUTTON_BACK_PUSHED = "back"


class TEAM_COLOR(Enum):
    RED = "red"
    BLUE = "blue"


# GCからくるデータを表す定数
class GC_CONSTANTS:
    STATE = {
        -1: GAME_STATE.IMPOSSIBLE,
        0: GAME_STATE.INITIAL,
        1: GAME_STATE.READY,
        2: GAME_STATE.SET,
        3: GAME_STATE.PLAYING,
        4: GAME_STATE.FINISHED,
    }
    SECONDARY_STATE = {
        255: SECONDARY_STATE.UNKNOWN,
        0: SECONDARY_STATE.NORMAL,
        1: SECONDARY_STATE.PENALTYSHOOT,
        2: SECONDARY_STATE.OVERTIME,
        3: SECONDARY_STATE.TIMEOUT,
        4: SECONDARY_STATE.DIRECT_FREEKICK,
        5: SECONDARY_STATE.INDIRECT_FREEKICK,
        6: SECONDARY_STATE.PENALTYKICK,
        7: SECONDARY_STATE.CORNER_KICK,
        8: SECONDARY_STATE.GOAL_KICK,
        9: SECONDARY_STATE.THROW_IN,
    }
    PENALTY = {
        255: PENALTY.UNKNOWN,
        0: PENALTY.NONE,
        14: PENALTY.SUBSTITUTE,
        15: PENALTY.MANUAL,
        30: PENALTY.BALL_MANIPULATION,
        31: PENALTY.PUSHING,
        34: PENALTY.PICKUP_UNCAPABLE,
        35: PENALTY.SERVICE,
    }


# 検出したオブジェクトのラベル (文字列型と比較したいならEnumにしない方が良い)
class OBJECT_LABEL:
    BALL = "Ball"
    OPPONENT = "Opponent"
    PERSON = "Person"
    GOALPOST = "Goalpost"


# アクションの結果を表す定数
class ACTION_RESULT(Enum):
    # 事前条件が満たされていない場合
    PRECONDITON_FAILED = -1
    # 事後条件が既に満たされていた場合
    POSTCONDITION_ALREADY_SATISFIED = 0
    # アクションが成功した場合
    SUCCESS = 1


class FIELD_DIMENTIONS:
    X_LENGTH = 1400
    Y_LENGTH = 900