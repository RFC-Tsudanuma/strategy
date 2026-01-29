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
# - Suzuha Kiuchi
# - Yugo Nishio
# - Masafumi Horiguchi

import math
import typing
from typing import Optional

import gtpyhop
import numpy as np
import state
from constants import OUR_FIELD, FIELD_DIMENTIONS
from setting import CONVERT_SCALE, STRATEGY_SHARED_CONFIG, DEFAULT_STRATEGY_CONFIG
from vision_interface.msg import DetectedObject, Detections
from game_controller_interface.msg import TeamInfo
import print_utils

def is_ball_in_outside_the_field(ball_position: state.Position) -> bool:
    """
    ボールがフィールド外にあるかを判定する。
    フィールド外にあればTrue、そうでなければFalseを返す。
    """
    if ball_position.coord != state.Position.COORD_GLOBAL:
        raise ValueError(
            "utils.is_ball_in_outside_the_field() :: ball_position must be in global coordinates"
        )
    field_x_len = FIELD_DIMENTIONS.X_LENGTH / 2.0 + STRATEGY_SHARED_CONFIG["ball_outside_config"]["x_threshould"]
    field_y_len = FIELD_DIMENTIONS.Y_LENGTH / 2.0 + STRATEGY_SHARED_CONFIG["ball_outside_config"]["y_threshould"]
    if (
        abs(ball_position.x) > field_x_len
        or abs(ball_position.y) > field_y_len
    ):
        return True
    return False

def convert_local_to_global(
    self_position_global: state.Position, target_local_position: state.Position
) -> state.Position:
    """
    ロボットのグローバル位置と姿勢からローカル位置をグローバル位置に変換する
    """
    if self_position_global.coord != state.Position.COORD_GLOBAL:
        raise ValueError(
            "utils.convert_local_to_global() :: self_position_global must be in global coordinates"
        )
    if target_local_position.coord != state.Position.COORD_LOCAL:
        raise ValueError(
            "utils.convert_local_to_global() :: target_local_position must be in local coordinates"
        )

    # ロボットの向きを使用して座標変換
    cos_theta = np.cos(self_position_global.theta)
    sin_theta = np.sin(self_position_global.theta)

    # グローバル座標の計算
    global_x = (
        self_position_global.x
        + target_local_position.x * cos_theta
        - target_local_position.y * sin_theta
    )
    global_y = (
        self_position_global.y
        + target_local_position.x * sin_theta
        + target_local_position.y * cos_theta
    )

    return state.Position(
        state.Position.COORD_GLOBAL, global_x, global_y, target_local_position.theta
    )


def convert_global_to_local(
    self_position_global: state.Position, target_global_position: state.Position
) -> state.Position:
    """
    ロボットのグローバル位置と姿勢からグローバル位置をローカル位置に変換する
    """
    if self_position_global.coord != state.Position.COORD_GLOBAL:
        raise ValueError(
            "utils.convert_global_to_local() :: self_position_global must be in global coordinates"
        )
    if target_global_position.coord != state.Position.COORD_GLOBAL:
        raise ValueError(
            "utils.convert_global_to_local() :: target_global_position must be in global coordinates"
        )

    # ロボットの向きを使用して座標変換
    cos_theta = np.cos(self_position_global.theta)
    sin_theta = np.sin(self_position_global.theta)

    # ローカル座標の計算
    local_x = (target_global_position.x - self_position_global.x) * cos_theta + (
        target_global_position.y - self_position_global.y
    ) * sin_theta
    local_y = (
        -(target_global_position.x - self_position_global.x) * sin_theta
        + (target_global_position.y - self_position_global.y) * cos_theta
    )

    target_theta = target_global_position.theta - self_position_global.theta

    return state.Position(
        state.Position.COORD_LOCAL, local_x, local_y, normalize_angle(target_theta)
    )


def convert_detectedobject_to_position(
    detected_object: DetectedObject,
) -> state.Position:
    return state.Position(
        coord=state.Position.COORD_LOCAL,
        x=detected_object.position_projection[0] * CONVERT_SCALE,
        y=detected_object.position_projection[1] * CONVERT_SCALE,
    )


def normalize_angle(angle: float) -> float:
    result = (angle + math.pi) % (math.pi * 2.0)
    if result < 0.0:
        result += math.pi * 2
    result -= math.pi
    return result


def calculate_relative_angle(
    object: state.Position, self_position: state.Position = None
) -> float:
    """
    自分とオブジェクトの相対角度を計算する
    """
    if object.coord == state.Position.COORD_LOCAL:
        dx = object.x
        dy = object.y
        return np.arctan2(dy, dx)
    elif object.coord == state.Position.COORD_GLOBAL:
        if self_position is None:
            raise ValueError(
                "utils.calculate_relative_angle() :: self_position must be provided for global coordinates"
            )
        dx = object.x - self_position.x
        dy = object.y - self_position.y
        return np.arctan2(dy, dx) - self_position.theta
    else:
        raise ValueError(f"Unknown coordinate system: {object.coord}")


def calculate_distance(object_pos: state.Position) -> float:
    """
    オブジェクトと自分の距離を計算する
    ここでは、オブジェクトの座標系はローカル座標
    """
    if object_pos.coord != state.Position.COORD_LOCAL:
        raise ValueError(
            "utils.calculate_distance() :: object_pos must be in local coordinates"
        )
    return np.sqrt(object_pos.x**2 + object_pos.y**2)


def calculate_distance_global(
    object1: state.Position, object2: state.Position
) -> float:
    if object1.coord != object2.coord:
        raise ValueError(
            "utils.calculate_distance_global() :: both positions must be in the same coordinate system"
        )
    return np.sqrt((object1.x - object2.x) ** 2 + (object1.y - object2.y) ** 2)


def calculate_angle(object_pos: state.Position) -> float:
    """
    オブジェクトに対する角度を計算する
    正規化された値を返す
    """
    if object_pos.coord != state.Position.COORD_LOCAL:
        raise ValueError(
            "utils.calculate_angle() :: object_pos must be in local coordinates"
        )
    return np.arctan2(object_pos.y, object_pos.x)


def calculate_angle_global(selfpos: state.Position, pos2: state.Position) -> float:
    """
    グローバルの自己位置と他のグローバル位置から、自分から見た角度を計算する。
    正規化された値を返す。
    """
    if (
        selfpos.coord != state.Position.COORD_GLOBAL
        or pos2.coord != state.Position.COORD_GLOBAL
    ):
        raise ValueError(
            "utils.calculate_angle_global() :: both positions must be in global coordinates"
        )
    dx = pos2.x - selfpos.x
    dy = pos2.y - selfpos.y
    result = np.arctan2(dy, dx) - selfpos.theta
    return normalize_angle(result)


def get_real_action_name(virtual_action_tuple: typing.Tuple[str, ...]) -> str:
    """
    仮想アクションのタプルから実アクションの名前を取得する
    """
    if not virtual_action_tuple or len(virtual_action_tuple) < 1:
        raise ValueError(
            "utils.get_real_action_name() :: virtual_action_tuple must not be empty"
        )
    if "va_" not in virtual_action_tuple[0]:
        return virtual_action_tuple[0]  # そのままの名前で良いやつは返す
    else:
        name = virtual_action_tuple[0][3:]  # 先頭の"va_"を除去
    return name


def take_target_out_from_detected_objects(
    label: str, detections: Detections
) -> list[DetectedObject]:
    """
    指定したラベルを持つオブジェクトを取り出す。
    """
    result = []
    for object in detections.detected_objects:
        if object.label == label:
            result.append(object)
    return result


def get_goal_center_pos(s: gtpyhop.State) -> state.Position:
    """
    ゴールポスト2つの情報からその中央となる位置を計算する
    """
    goalpost_local = get_goalpost_local(s)
    goal_position1 = goalpost_local[0]
    goal_position2 = goalpost_local[1]
    y = (goal_position1.y + goal_position2.y) / 2
    x = (goal_position1.x + goal_position2.x) / 2
    return state.Position(state.Position.COORD_LOCAL, x, y)


def get_all_obstacles(current_state: gtpyhop.State) -> list[state.Position]:
    """
    現在の状態からすべての障害物を取得する
    """
    obstacles = []
    for obj in current_state.opponent_position_local:
        if isinstance(obj, state.Position):
            obstacles.append(obj)
        else:
            raise ValueError(f"Unknown obstacle type: {type(obj)}")

    for obj in current_state.allies_position_local:
        if isinstance(obj, state.Position):
            obstacles.append(obj)
        else:
            raise ValueError(f"Unknown obstacle type: {type(obj)}")
    for obj in current_state.opponent_goalpost_local:
        if isinstance(obj, state.Position):
            obstacles.append(obj)
        else:
            raise ValueError(f"Unknown obstacle type: {type(obj)}")
    for obj in current_state.ally_goalpost_local:
        if isinstance(obj, state.Position):
            obstacles.append(obj)
        else:
            raise ValueError(f"Unknown obstacle type: {type(obj)}")
    for obj in current_state.normal_obstacles_position_local:
        if isinstance(obj, state.Position):
            obstacles.append(obj)
        else:
            raise ValueError(f"Unknown obstacle type: {type(obj)}")
    
    return obstacles


def get_behind_position(
    object_pos1: state.Position, object_pos2: state.Position, space
) -> state.Position:
    """
    ロボットがobject_pos1とobject_pos2の順でspace分後ろの座標を返す
    Position:ローカル座標
    """
    unit_vec = get_unit_vec(object_pos1, object_pos2)
    target_position_x = object_pos1.x + unit_vec.x * space
    target_position_y = object_pos1.y + unit_vec.y * space
    return state.Position(
        state.Position.COORD_LOCAL, target_position_x, target_position_y
    )

def get_behind_ball_position(ball_pos: state.Position, space) -> state.Position:
    """
    ボールの手前をアプローチの目標座標にする
    Position:ローカル座標
    """
    target_position_x = (ball_pos.x) - space
    target_position_y = ball_pos.y

    return state.Position(
        state.Position.COORD_LOCAL, target_position_x, target_position_y
    )



def robot_on_line(object_pos1: state.Position, object_pos2: state.Position) -> bool:
    """
    ロボットがobject_pos1とobject_pos2を結ぶ直線の延長線上にいるかつrobot-1-2の順かを判定する
    Position:ローカル座標
    """
    # 外積の絶対値が非常に小さければ、ほぼ一直線上にあるとみなす
    # if calculate_distance_global(object_pos1, object_pos2) < 200:
    #     side_distance_tolerance = 60  # 許容横ずれ
    # else:
    side_distance_tolerance = 25  # 許容横ずれ

    tolerance = calculate_distance(object_pos2) * side_distance_tolerance
    cross_product = object_pos1.x * object_pos2.y - object_pos2.x * object_pos1.y
    collinear = abs(cross_product) < tolerance
    if not collinear:
        return False
    # 内積からrobot-1-2の順かを判断
    vec_1_to_robot_x = 0 - object_pos1.x
    vec_1_to_robot_y = 0 - object_pos1.y
    vec_1_to_2_x = object_pos2.x - object_pos1.x
    vec_1_to_2_y = object_pos2.y - object_pos1.y
    dot_product = vec_1_to_robot_x * vec_1_to_2_x + vec_1_to_robot_y * vec_1_to_2_y
    behind = dot_product < 0
    if behind:
        print_utils.debug_print("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
    return behind


def get_angle_from_center(
    center_pos: state.Position, target_pos: state.Position
) -> float:
    """
    center_posを頂点として、robotとtarget_posがなす角度を計算する。
    center_pos,target_pos:ローカル座標
    """
    # 中心を原点とした2つのベクトルを計算
    vec_center_to_robot_x = 0 - center_pos.x
    vec_center_to_robot_y = 0 - center_pos.y
    vec_center_to_target_x = target_pos.x - center_pos.x
    vec_center_to_target_y = target_pos.y - center_pos.y
    # 各ベクトルの絶対角度をatan2で計算
    angle_self = math.atan2(vec_center_to_robot_y, vec_center_to_robot_x)
    angle_target = math.atan2(vec_center_to_target_y, vec_center_to_target_x)
    # 角度の差を計算し、-πからπ (-180°から180°) の範囲に正規化
    angle_diff_rad = math.atan2(
        math.sin(angle_target - angle_self), math.cos(angle_target - angle_self)
    )
    return angle_diff_rad


def get_goalpost_local(s: gtpyhop.State) -> list[state.Position]:
    """
    ローカル座標のgoalpostが有効か確認する
    すべて有効か確認する
    有効でない場合はグローバル座標から補完してローカル座標を返す
    """
    valid_local_posts = []
    true_global_post_a = convert_hardcoded_pos_to_our_field(s,state.Position(state.Position.COORD_GLOBAL, 700.0, 130.0))
    true_global_post_b = convert_hardcoded_pos_to_our_field(s,state.Position(state.Position.COORD_GLOBAL, 700.0, -130.0))
    tolerance = 50  # 許容半径距離

    # 2. 見えているローカルのポストをグローバルに変換して検証
    which_post_is_seen = {"A": False, "B": False}  # どちらのポストが見えたかを記録

    for local_post in s.opponent_goalpost_local:
        detected_global_post = convert_local_to_global(
            s.self_position_global, local_post
        )
        # print(f"Adistance{calculate_distance_between(detected_global_post, true_global_post_a)}")
        # print(f"Bdistance{calculate_distance_between(detected_global_post, true_global_post_b)}")
        if (
            calculate_distance_between(detected_global_post, true_global_post_a)
            < tolerance
        ):
            valid_local_posts.append(local_post)
            which_post_is_seen["A"] = True
        elif (
            calculate_distance_between(detected_global_post, true_global_post_b)
            < tolerance
        ):
            valid_local_posts.append(local_post)
            which_post_is_seen["B"] = True

    # 最終的なゴールポストリスト
    if len(valid_local_posts) == 2:
        return valid_local_posts
    elif len(valid_local_posts) == 1:
        seen_post = valid_local_posts[0]
        if not which_post_is_seen["A"]:
            calculated_post = convert_global_to_local(
                s.self_position_global, true_global_post_a
            )
            return [calculated_post, seen_post]
        else:
            calculated_post = convert_global_to_local(
                s.self_position_global, true_global_post_b
            )
            return [seen_post, calculated_post]
    else:
        return [
            convert_global_to_local(s.self_position_global, true_global_post_a),
            convert_global_to_local(s.self_position_global, true_global_post_b),
        ]


def get_unit_vec(
    object_pos1: state.Position, object_pos2: state.Position
) -> state.Position:
    """
    ２から１に向かう単位方向ベクトルを計算
    ローカル座標
    """
    vec_x = object_pos1.x - object_pos2.x
    vec_y = object_pos1.y - object_pos2.y
    distance = math.sqrt(vec_x**2 + vec_y**2)
    if distance == 0:
        return state.Position(state.Position.COORD_LOCAL, 0.0, 0.0)
    unit_vec_x = vec_x / distance
    unit_vec_y = vec_y / distance
    return state.Position(state.Position.COORD_LOCAL, unit_vec_x, unit_vec_y)


def calculate_distance_between(
    object_pos1: state.Position, object_pos2: state.Position
) -> float:
    """
    2点間の距離を計算
    """
    vec_x = object_pos1.x - object_pos2.x
    vec_y = object_pos1.y - object_pos2.y
    distance = math.sqrt(vec_x**2 + vec_y**2)
    return distance


def get_last_visible_ball_from_history(
    object_position_history: list[state.ObjectGlobalPositions],
    search_length: typing.Optional[int] = None,
) -> typing.Optional[state.Position]:
    """
    現在の状態から最後に見えたボールの位置を取得する.
    search_length: 履歴を探索する長さ
    """
    if object_position_history is None or len(object_position_history) == 0:
        return None  # ボールが見つからなかった場合

    # 最後に見えたボールの位置を取得
    index = 0
    for objects in reversed(object_position_history):
        ball = objects.ball_pos_global
        index += 1
        if len(ball) == 1:
            return ball[0]
        if search_length is not None and index > search_length:
            break
    # ボールが見つからない場合
    return None


def get_last_visible_goalpost(
    object_position_history: list[state.ObjectGlobalPositions],
) -> list[state.Position]:
    """
    現在の状態から最後に見えたボールの位置を取得する
    """
    if object_position_history is None or len(object_position_history) == 0:
        return []  # ない場合

    # 最後に見えたゴールポストの位置を取得
    for objects in reversed(object_position_history):
        goalposts = objects.goalpost_pos_global
        if len(goalposts) > 0:
            return goalposts

    return []


def is_object_in_our_field(our_field: OUR_FIELD, object_pos: state.Position) -> bool:
    if object_pos.coord != state.Position.COORD_GLOBAL:
        raise ValueError(
            "utils.is_object_in_our_field() :: object_pos must be in global coordinates"
        )
    # 自分たちが右側（x > 0）の場合、右側のゴールが自分たちのゴール
    if our_field == OUR_FIELD.RIGHT:
        return object_pos.x > 0
    # 自分たちが左側（x < 0）の場合、左側のゴールが自分たちのゴール
    elif our_field == OUR_FIELD.LEFT:
        return object_pos.x < 0
    return False  # 不明なフィールドの場合はFalseを返す

def get_most_reliable_shared_ballpos(shared_info: list[state.SharedMessage]):
    """
    共有された情報の中から、最も信頼できるボールの情報を取得する
    """
    if shared_info is None:
        return None
    most_reliable_ballpos = None
    max_distance = 1e-6
    for info in shared_info:
        if info is None:
            continue
        if info.ballpos is not None and info.selfpos is not None:
            if most_reliable_ballpos is None:
                most_reliable_ballpos = info.ballpos
            else:
                current_distance = calculate_distance(info.ballpos_local)
                if current_distance > max_distance:
                    most_reliable_ballpos = info.ballpos
                    max_distance = current_distance
    return most_reliable_ballpos


def get_most_reliable_ballpos_global(
    current_selfpos: Optional[state.Position],
    current_detected_ballpos_local: Optional[state.Position],
    shared_info: list[state.SharedMessage],
) -> typing.Optional[state.Position]:
    """
    共有された情報と今持っている情報の中から、最も信頼できるボールの情報を取得する
    ボールとの距離が遠い = ピッチ角が高い = 自己位置推定が正しい確率が高い、なので、ボールとの距離が一番遠いやつを信頼する。
    """
    if shared_info is None:
        return None
    most_reliable_ballpos = None
    max_distance = 1e-6
    # ひとまず自分の観測にする
    if current_detected_ballpos_local is not None and current_selfpos is not None:
        most_reliable_ballpos = convert_local_to_global(
            current_selfpos, current_detected_ballpos_local
        )
        max_distance = calculate_distance(current_detected_ballpos_local)
    # 受け取った情報と比較する
    for info in shared_info:
        if info is None:
            continue
        if info.ballpos is not None and info.selfpos is not None:
            if most_reliable_ballpos is None:
                most_reliable_ballpos = info.ballpos
            else:
                current_distance = calculate_distance(info.ballpos_local)
                if current_distance > max_distance:
                    most_reliable_ballpos = info.ballpos
                    max_distance = current_distance
    return most_reliable_ballpos


def get_other_player_selfpos_from_shared_info(shared_info: list[state.SharedMessage]):
    """
    SharedMessageのリストから他ロボットの有効な自己位置を取り出す
    """
    if shared_info is None:
        return []
    result = []
    for info in shared_info:
        if info.selfpos is not None:
            result.append(info.selfpos)
    return result


def get_other_player_ballpos_from_shared_info(shared_info: list[state.SharedMessage]):
    """
    SharedMessageのリストから有効なボール位置を取り出す
    """
    result = []
    for info in shared_info:
        if info is not None and info.ballpos is not None:
            result.append(info.ballpos)
    return result

def get_other_player_average_ballpos_from_shared_info(shared_info: typing.Optional[state.SharedMessage]):
    """
    SharedMessageのリストから有効なボール位置を取り出す
    """
    ballpos = state.Position(x=0.0, y=0.0, coord=state.Position.COORD_GLOBAL)
    count = 0
    for info in shared_info:
        if info is not None and info.ballpos is not None:
            ballpos.x += info.ballpos.x
            ballpos.y += info.ballpos.y
            count += 1
    if count > 0:
        ballpos.x /= count
        ballpos.y /= count
        return ballpos
    else:
        return None

def are_target_poses_near(pose1: state.Position, pose2: state.Position, threshold_x: float, threshold_y: float, threshould_theta: float) -> bool:
    """
    2つの姿勢が指定された閾値以内に近いかどうかを判定する
    """
    if pose1.coord != pose2.coord:
        raise ValueError(
            "utils.are_positions_near() :: both positions must be in the same coordinate system"
        )
    return abs(pose1.x - pose2.x) <= threshold_x and abs(pose1.y - pose2.y) <= threshold_y and abs(normalize_angle(pose1.theta - pose2.theta)) <= threshould_theta

def get_start_position() -> state.Position:
    """
    ロボットのフォーメーション位置(グローバル座標)を取得する。
    :return: フォーメーション位置のPositionオブジェクト
    """
    robot_id = DEFAULT_STRATEGY_CONFIG["common_config"]["robot_id"]
    x :float = DEFAULT_STRATEGY_CONFIG["position_config"]["robot_" + str(robot_id)]["ready_x"]
    y :float = DEFAULT_STRATEGY_CONFIG["position_config"]["robot_" + str(robot_id)]["ready_y"]
    return state.Position(coord=state.Position.COORD_GLOBAL, x=x, y=y, theta=0.0)

def retrieve_team_data_from_gc_teams(gc_teams: list[TeamInfo])-> TeamInfo:
    """
    gcから来るデータのteamsから自チームのデータを取り出す
    """
    our_team_id = DEFAULT_STRATEGY_CONFIG["common_config"]["team_id"]
    for team in gc_teams:
        if team.team_number == our_team_id:
            return team
    return gc_teams[0]  # 見つからなかった場合はとりあえず最初のやつを返す


def convert_hardcoded_pos_to_our_field(
    s: gtpyhop.State, target_pos: state.Position
) -> state.Position:
    """
    our_fieldは常に左側とするので、恒等写像にした
    """
    # if target_pos.coord != state.Position.COORD_GLOBAL:
    #     raise ValueError(
    #         "utils.convert_hardcoded_pos_to_our_field() :: target_pos must be in global coordinates"
    #     )
    # # 左側の場合は特に変換しない。
    # if s.world_states["our_field"] == OUR_FIELD.LEFT:
    #     return target_pos
    # elif s.world_states["our_field"] == OUR_FIELD.RIGHT:
    #     return state.Position(
    #         state.Position.COORD_GLOBAL,
    #         -target_pos.x,
    #         -target_pos.y,
    #         normalize_angle(target_pos.theta + math.pi),
    #     )
    # else:
    #     print_utils.debug_print(
    #         "utils.convert_hardcoded_pos_to_our_field() :: our_field is UNKNOWN, cannot convert position."
    #     )
    #     return target_pos
    return target_pos

def get_shared_ally_attackers_positions(s: gtpyhop.State) -> list[state.Position]:
    """
    情報共有された情報から、味方のアタッカーの位置リストを取得する
    固定サイズのリストが返る。対応するIDはリストのインデックス-1となる
    """
    ally_attacker_positions = []
    for ally in s.shared_msg:
        if ally is not None and ally.role == state.ROLE.ATTACKER and ally.selfpos is not None:
            ally_attacker_positions.append(ally.selfpos)
        else:
            ally_attacker_positions.append(None)
    return ally_attacker_positions

def get_shared_ally_positions(s: gtpyhop.State) -> list[state.Position]:
    """
    情報共有された情報から、味方の位置リストを取得する
    固定サイズのリストが返る。対応するIDはリストのインデックス-1となる
    """
    ally_positions = []
    for ally in s.shared_msg:
        if ally is not None and ally.selfpos is not None:
            ally_positions.append(ally.selfpos)
        else:
            ally_positions.append(None)
    return ally_positions

def get_shared_ally_ballpos_local(s: gtpyhop.State) -> list[state.Position]:
    """
    情報共有された情報から、味方のボール位置リストを取得する
    固定サイズのリストが返る。対応するIDはリストのインデックス-1となる
    """
    ally_ball_positions = []
    for ally in s.shared_msg:
        if ally is not None and ally.ballpos_local is not None:
            ally_ball_positions.append(ally.ballpos_local)
        else:
            ally_ball_positions.append(None)
    return ally_ball_positions