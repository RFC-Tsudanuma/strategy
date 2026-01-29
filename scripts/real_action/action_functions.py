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
# - Yugo Nishio
# - Suzuha Kiuchi
# - Masafumi Horiguchi
# - Haruki Ogawa

import math
import typing

import gtpyhop
import pathplan
import print_utils
from constants import ROLE
from setting import (
    DEFAULT_STRATEGY_CONFIG,
    ROBOT_MAX_SPEED_PI,
    ROBOT_MAX_SPEED_X_MINUS,
    ROBOT_MAX_SPEED_X_PLUS,
    ROBOT_MAX_SPEED_Y,
    STRATEGY_SLEEP_TIME,
)
from state import Position, RoleChangeException
from utils import (
    calculate_angle,
    calculate_distance,
    calculate_relative_angle,
    convert_global_to_local,
    get_all_obstacles,
    get_angle_from_center,
    convert_local_to_global
)

from strategy.ros_publishers import (
    WalkCommand,
    WalkCommandPublisher,
)


def move_neck_for_tracking_ball(
    ball_position_local: typing.Optional[Position], pub: WalkCommandPublisher
):
    """
    ボールを追跡するために首を動かす。
    :param s 状態
    :param pub: WalkCommandPublisher
    """
    ball_pos = ball_position_local
    if ball_pos is None:
        return
    # angle = calculate_relative_angle(ball_pos)
    # pub.send_neck_yaw_command(NeckMovementMode(yaw_angle=angle))
    return


def generate_walk_command(target_pos: Position) -> WalkCommand:
    """
    指定された位置に向かうための歩行コマンドを生成する。
    :param target_pos: 目標位置（ローカル座標系）
    :return: WalkCommand
    """
    if target_pos.coord != Position.COORD_LOCAL:
        raise ValueError(
            "generate_walk_command() :: target_pos must be in local coordinates"
        )
    x_vel = 0.0
    y_vel = 0.0
    theta_vel = 0.0
    x_is_small = math.fabs(target_pos.x) < 30.0
    y_is_small = math.fabs(target_pos.y) < 30.0
    angle = calculate_angle(target_pos)
    target_distance = calculate_distance(target_pos)

    if target_distance < 15.0:
        return WalkCommand(0.0, 0.0, 0.0)

    x_time = math.fabs(target_pos.x / ROBOT_MAX_SPEED_X_PLUS)
    y_time = math.fabs(target_pos.y / ROBOT_MAX_SPEED_Y)
    if x_time > y_time:
        time = x_time
    else:
        time = y_time
    if math.fabs(angle) > 0.9:
        theta_vel = math.copysign(ROBOT_MAX_SPEED_PI, angle)
    elif math.fabs(angle) < 0.2 or target_distance < 100.0:
        # 15度くらいより小さい or 1m以内なら回転成分は入れない。
        theta_vel = 0.0
    else:
        theta_vel = angle / math.fabs(time) * 0.9

    if theta_vel > ROBOT_MAX_SPEED_PI:
        linear_vel_scale = math.fabs(time / (angle / ROBOT_MAX_SPEED_PI)) * 0.9
    elif target_distance < 300:
        linear_vel_scale = 0.6  # 距離が近い場合は速度を落とす
    else:
        linear_vel_scale = 1.0

    if math.fabs(target_pos.x) > math.fabs(target_pos.y):
        x_vel = ROBOT_MAX_SPEED_X_PLUS if target_pos.x > 0 else ROBOT_MAX_SPEED_X_MINUS
        if y_is_small:
            y_vel = 0.0
        else:
            y_vel = (
                math.fabs(target_pos.y) / math.fabs(target_pos.x)
            ) * ROBOT_MAX_SPEED_Y
            y_vel = (
                y_vel if target_pos.y > 0 else -y_vel
            )  # TODO: これはロボットの左右の動きに合わせて返る
    else:
        y_vel = ROBOT_MAX_SPEED_Y if target_pos.y > 0 else -ROBOT_MAX_SPEED_Y
        if x_is_small:
            x_vel = 0.0
        else:
            x_vel = (
                math.fabs(target_pos.x) / math.fabs(target_pos.y)
            ) * ROBOT_MAX_SPEED_X_PLUS
            x_vel = x_vel if target_pos.x > 0 else -x_vel
    x_scale = DEFAULT_STRATEGY_CONFIG["common_config"]["walk_x_scale"]
    y_scale = DEFAULT_STRATEGY_CONFIG["common_config"]["walk_y_scale"]
    yaw_scale = DEFAULT_STRATEGY_CONFIG["common_config"]["walk_yaw_scale"]
    return WalkCommand(
        x_vel * linear_vel_scale * x_scale,
        y_vel * linear_vel_scale * y_scale,
        theta_vel * yaw_scale,
    )


def refine_generate_walk_command(target_pos: Position, pub: WalkCommandPublisher) -> WalkCommand:
    """
    指定された位置に向かうための歩行コマンドを生成する。
    :param target_pos: 目標位置（ローカル座標系）
    :return: WalkCommand
    """
    if target_pos.coord != Position.COORD_LOCAL:
        raise ValueError(
            "generate_walk_command() :: target_pos must be in local coordinates"
        )
    x_vel = 0.0
    y_vel = 0.0
    theta_vel = 0.0

    x_middle_vel = 40 # [cm/s]
    x_high_vel = 90 # [cm/s]
    y_middle_vel = 20 # [cm/s]
    y_high_vel = 50 # [cm/s]
    theta_high_vel = 0.4 # [m/rad]
    angle = calculate_angle(target_pos)
    target_distance = calculate_distance(target_pos)

    x_middle_stop_distance = 20   # 停止する司令を出す距離[cm]
    y_middle_stop_distance = 20   # 停止する司令を出す距離[cm]
    y_high_stop_distance = 60     # 停止する司令を出す距離[cm]
    # x_is_small = math.fabs(target_pos.x) < 30.0
    # y_is_small = math.fabs(target_pos.y) < 30.0


    x_middle_vel_cal = 0.41 # [m/s]
    t_middle_acc = 1.47
    t_middle_dcc = 1.44
    t_high_acc = 1.733
    t_high_dec= 1.676
    x_high_vel_cal = 0.94 # [m/s]
    da_middle = (0.5 * x_middle_vel_cal * t_middle_acc) * 100.0
    dd_middle = (0.5 *x_middle_vel_cal * t_middle_dcc) * 100.0
    da_middle_to_high = (0.5 * t_high_acc * (x_high_vel_cal ** 2 - x_middle_vel_cal ** 2) / x_high_vel_cal) * 100.0
    dd_high = 0.5 * x_high_vel_cal * t_high_dec
    change_walk_distance = (da_middle + da_middle_to_high) # 距離をcmに変換するため100倍

    x_high_stop_distance = dd_high * 100     # 停止する司令を出す距離[cm]

    current_walk_x = 0
    current_walk_y = 0

    print_utils.debug_print(f"da_middle : {da_middle}")
    print_utils.debug_print(f"dd_middle : {dd_middle}")

    # if x_is_small:
    #     x_vel = 0.0
    # if y_is_small:
    #     y_vel = 0.0

    if pub.current_walk_cmd is None:
        current_walk_x = x_middle_vel
        current_walk_y = y_middle_vel
    else:
        current_walk_x = pub.current_walk_cmd.x_velocity
        current_walk_y = pub.current_walk_cmd.y_velocity

    # x方向の速度決定
    print_utils.debug_print(f"target_pos.x : {target_pos.x}")
    print_utils.debug_print(f"target_pos.y : {target_pos.y}")
    print_utils.debug_print(f"angle : {angle}")

    # if math.fabs(angle) > 0.3 and target_distance > 50.0:
    if math.fabs(angle) > 0.3:
        if angle > 0.0:
            theta_vel = theta_high_vel
        else:
            theta_vel = -theta_high_vel
    else:
        theta_vel = 0.0
    
    print_utils.debug_print(f"change_walk_distance : {change_walk_distance}")

    if target_pos.x >= 0.0:
        if math.fabs(current_walk_x) == x_high_vel: # 0.9 m/sで歩行していたら停止距離に入るまで0.9 m/sで歩行
            if math.fabs(target_pos.x) < x_high_stop_distance: # 0.9 m/sの停止距離に入ったら停止
                x_vel = 0.0
                print_utils.print_without_prefix("if math.fabs(target_pos.x) < x_high_stop_distance:")
            else:
                x_vel = x_high_vel
        elif math.fabs(target_pos.x) < change_walk_distance:
            if ((math.fabs(target_pos.x) < x_middle_stop_distance
                  and pub.get_current_command_elapsed_time() < t_middle_acc
                  and math.fabs(current_walk_x) == x_middle_vel)
                  or math.fabs(current_walk_x) == 10.0
                  or math.fabs(current_walk_x) == 0.0):
                if math.fabs(target_pos.x) < 5.0:
                    x_vel = 0.0
                    print_utils.print_without_prefix("if math.fabs(target_pos.x) < 5.0:")
                else:
                    x_vel = 20.0
            elif math.fabs(target_pos.x) < x_middle_stop_distance:
                x_vel = 0.0
                print_utils.print_without_prefix("if math.fabs(target_pos.x) < x_middle_stop_distance:")
            else:
                x_vel = x_middle_vel 
        else:
            if (math.fabs(current_walk_x) == x_middle_vel 
                and pub.get_current_command_elapsed_time() > t_middle_acc): # 0.4 m/s司令で等速区間に入ったら0.9 m/s
                x_vel = x_high_vel
            else:
                x_vel = x_middle_vel
    else:
        if (math.fabs(target_pos.x) >  20.0):
            x_vel = -x_middle_vel
        elif (math.fabs(target_pos.x) > (change_walk_distance)):
            x_vel = -x_high_vel
        else:
            print_utils.print_without_prefix("else:")

    # # y方向の速度決定
    # if math.fabs(target_pos.y) > y_high_stop_distance:
    #     if target_pos.y > 10.0:
    #         y_vel = y_high_vel
    #     elif target_pos.y < -10.0:
    #         y_vel = -y_high_vel
    #     else:
    #         y_vel = 0.0
    # else:
    #     y_vel = 0.0
        
    return WalkCommand(x_vel, y_vel, theta_vel)
    

def generate_turn_command(target_pos: Position) -> WalkCommand:
    if target_pos.coord != Position.COORD_LOCAL:
        raise ValueError(
            "generate_turn_command() :: target_pos must be in local coordinates"
        )
    angle = calculate_relative_angle(target_pos)
    cmd = generate_turn_command_by_angle(angle)
    return cmd


def generate_circle_command(center_pos: Position, target_pos: Position) -> WalkCommand:
    """
    体の向きを変えずにcenter_posを中心にtarget_posへ円周上に移動
    center_pos,target_pos:ローカル座標
    """
    # 回転方向を決める
    # +y方向の接線方向ベクトル
    tangent_center_vec_x = -center_pos.y
    tangent_center_vec_y = center_pos.x
    dot_product = (
        tangent_center_vec_x * target_pos.x + tangent_center_vec_y * target_pos.y
    )
    if dot_product < 0:
        tangent_center_vec_x *= -1
        tangent_center_vec_y *= -1
    tangent_dist = math.sqrt(tangent_center_vec_x**2 + tangent_center_vec_y**2)
    if tangent_dist == 0:
        return WalkCommand(0.0, 0.0, 0.0)
    unit_tx = tangent_center_vec_x / tangent_dist
    unit_ty = tangent_center_vec_y / tangent_dist
    target_speed = (
        ROBOT_MAX_SPEED_Y
        * calculate_distance(target_pos)
        * 0.01
    )
    x_vel = unit_tx * target_speed
    y_vel = unit_ty * target_speed
    return WalkCommand(x_vel, y_vel, 0.0)


def generate_center_circle_command(
    center_pos: Position, target_pos: Position
) -> WalkCommand:
    """
    中心を見ながらcenter_posを中心にtarget_posへ円周上に移動_半径30cm未満推奨
    center_pos,target_pos:ローカル座標
    """
    angle = get_angle_from_center(center_pos, target_pos)
    if math.fabs(angle) < 0.1:
        return WalkCommand(0.0, 0.0, theta_velocity=0.0)
    elif math.fabs(angle) > ROBOT_MAX_SPEED_PI * STRATEGY_SLEEP_TIME:
        theta_vel = 0.9 * math.copysign(ROBOT_MAX_SPEED_PI, angle)
        y_vel = -calculate_distance(center_pos) * theta_vel
        return WalkCommand(0.0, y_vel, theta_vel)
    else:
        time = STRATEGY_SLEEP_TIME - 0.05
        theta_vel = angle / time * 0.9
        y_vel = -calculate_distance(center_pos) * theta_vel
        return WalkCommand(0.0, y_vel, theta_velocity=theta_vel, continuous_time=time)


def refine_generate_center_circle_command(
    center_pos: Position, target_pos: Position
) -> WalkCommand:
    """
    中心を見ながらcenter_posを中心にtarget_posへ円周上に移動
    コマンド固定
    center_pos,target_pos:ローカル座標
    """
    angle = get_angle_from_center(center_pos, target_pos)
    print_utils.debug_print(f"angle!!!!!!!!!!!!!! : {angle}")
    print_utils.debug_print("++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    if math.fabs(angle) < 0.3:
        return WalkCommand(0.0, 0.0, 0.0)
    elif math.fabs(angle) > 0.52:
        if angle < 0:
            y_vel = 40.0
            theta_vel = -0.5
        else:
            y_vel = -40.0
            theta_vel = 0.5
    else:
        if angle < 0:
            y_vel = 20.0
            theta_vel = -0.35
        else:
            y_vel = -20.0
            theta_vel = 0.35

    return WalkCommand(0.0, y_vel, theta_vel)


def generate_turn_command_by_angle(angle: float) -> WalkCommand:
    """
    指定された角度になるための歩行コマンドを生成する。
    :param theta: 目標角度（ラジアン）
    :return: WalkCommand
    """
    # 不感帯（デッドゾーン）: 0.05rad (約3度)
    if math.fabs(angle) < 0.15:
        return WalkCommand(0.0, 0.0, theta_velocity=0.0)

    # 段階的な速度制御
    abs_angle = math.fabs(angle)

    # 微調整領域: 0.05〜0.15rad (約3〜8.5度)
    if abs_angle < 0.30:
        theta_vel = math.copysign(ROBOT_MAX_SPEED_PI * 0.2, angle)
    # 中間領域: 0.15〜0.5rad (約8.5〜28度)
    elif abs_angle < 0.6:
        theta_vel = math.copysign(ROBOT_MAX_SPEED_PI * 0.4, angle)
    # 大きな角度差: 0.5rad以上
    else:
        theta_vel = math.copysign(ROBOT_MAX_SPEED_PI * 0.8, angle)

    return WalkCommand(0.0, 0.0, theta_velocity=theta_vel)


def move_to_target_global_pose(
    s: gtpyhop.State,
    pub: typing.Optional[WalkCommandPublisher],
    target_global_pose: Position,
) -> typing.Optional[WalkCommand]:
    """
    グローバル座標系で指定された位置に移動するためのメソッド
    :param s: gtpyhopの状態
    :param target_global_pose: 目標位置（グローバル座標系）
    :return: 成功した場合はTrue、失敗した場合はFalse
    """
    if target_global_pose.coord != Position.COORD_GLOBAL:
        raise ValueError(
            print_utils.wrapp_notice(
                "move_to_target_global_pose() :: target_global_pose must be in global coordinates"
            )
        )
    
    # 自己位置 (self_position_global) が None の場合、座標変換が実行できないため処理を中断する
    if s.self_position_global is None:
        print_utils.print_in_strategy(
                print_utils.wrapp_notice(
                "move_to_target_global_pose() :: s.self_position_global is None. Cannot calculate local pose."
            ))
        # None を返すと、呼び出し元の actions.py が停止コマンドを出す設計になっている
        return None
    
    target_local_pose = convert_global_to_local(
        s.self_position_global, target_global_pose
    )
    
    # (推奨) 座標変換自体が失敗した場合も考慮
    if target_local_pose is None:
        print_utils.print_in_strategy(
            print_utils.wrapp_notice(
                "move_to_target_global_pose() :: convert_global_to_local returned None."
            ))
        return None
    
    if pub is not None:
        pub.send_target_position(target_global_pose)
    # 距離が近くて姿勢がほぼ一致している場合(0.2rad)は停止
    if (
        math.fabs(target_local_pose.theta - s.self_position_global.theta) < 0.1
        and calculate_distance(target_local_pose) < 30.0
    ):
        return WalkCommand(0.0, 0.0, 0.0)
    # 距離が近いなら(30cm)旋回で姿勢を調整
    elif calculate_distance(target_local_pose) < 30.0:
        target_theta = target_global_pose.theta - s.self_position_global.theta
        return generate_turn_command_by_angle(target_theta)
    else:
        # 距離が遠い場合はそこまで歩く
        obstacles = get_all_obstacles(s)
        planned_path, _ = pathplan.find_path_to_target(
            target_local_pose, obstacles, clearance=60.0
        )
        if validate_path(planned_path):
            return track_planned_path(planned_path, pub)
        else:
            return None


def track_planned_path(path: list[Position], pub) -> WalkCommand:
    """
    指定されたパスに沿ってロボットを移動させるためのコマンドを生成する。
    pathが空の場合は例外を投げる。基本はpathが空の場合はこの関数に渡さず、
    歩行コマンドは実行しないようにする。
    :param path: 目標の位置のリスト
    :param current_position: 現在のロボットの位置
    :return: WalkCommand
    """

    if not path:
        raise ValueError(
            "track_planned_path() :: Path cannot be empty"
        )  # TODO: これは試合の時には起こらないようにする必要がある

    # 最初の目標位置を取得
    target_pos = path[1]  # パスはスタート位置が先頭にあるので、次の位置を目標とする

    return refine_generate_walk_command(target_pos, pub)


def validate_path(path: list[Position]) -> bool:
    """
    パスが有効かどうかを確認する。
    :param path: 目標の位置のリスト
    :return: パスが有効な場合はTrue、無効な場合はFalse
    """
    if not path or len(path) < 2:
        return False  # パスが空または2つ以下の位置しかない場合は無効
    return True  # パスが有効


def change_role(new_role: ROLE):
    """
    ロボットのロールを切り替える。例外を投げるので、そこでfind_planから抜けてしまうのに注意する。
    """
    print_utils.wrapp_notice(f"Changing role to {new_role}.")
    raise RoleChangeException(new_role)  # ロール変更の例外を投げる

def need_waiting_at_kickoff(s: gtpyhop.State)-> bool:
    """
    キックオフ時に待機が必要かを判定する関数
    """
    if s.kickOffTeam == DEFAULT_STRATEGY_CONFIG["common_config"]["team_id"]:
        return False

    # キックオフチームでない場合、ボールが動くか、10秒立つまで待機する必要がある
    if s.ball_position_local is not None and s.self_position_global is not None:
        ball_pos_gl = convert_local_to_global(
            s.self_position_global, s.ball_position_local
        )
        # ボールの位置が原点から100mm以上離れたら動き出す
        if (math.sqrt(ball_pos_gl.x**2 + ball_pos_gl.y**2) > 100.0) or \
            (s.secondary_time < 0.2):
            return False
        else:
            # ボールがまだ動いていない場合は何もしない
            return True