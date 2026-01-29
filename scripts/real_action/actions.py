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
# - Haruki Ogawa
# - Yugo Nishio
# - Masafumi Horiguchi


import math
import time

import gtpyhop
import pathplan
import setting
import utils
from decision_logger import DecisionLogger
from constants import (
    BALL_POSITION,
    GAME_STATE,
    PENALTY,
    ROBOT_MODE,
    ROLE,
    SECONDARY_STATE,
)
from print_utils import print_in_real_action
from setting import ROBOT_MAX_SPEED_X_PLUS
from state import Position, RoleChangeException
from utils import (
    calculate_angle,
    calculate_distance,
    calculate_distance_global,
    convert_global_to_local,
    convert_local_to_global,
    get_all_obstacles,
    get_behind_position,
    get_behind_ball_position,
    get_goal_center_pos,
    get_goalpost_local,
    get_unit_vec,
    robot_on_line,
    get_start_position,
    convert_hardcoded_pos_to_our_field,
)

from strategy.ros_publishers import WalkCommand, WalkCommandPublisher

from . import action_functions
import print_utils


def wait_until_unpenalized(s: gtpyhop.State, pub: WalkCommandPublisher, logger: DecisionLogger):
    """
    Wait until the robot is unpenalized.
    """
    if s.world_states["game_state"] == GAME_STATE.PLAYING:
        print_in_real_action(
            "wait_until_unpenalized", "Executing wait_until_unpenalized()."
        )
        if s.world_states["penalty"] != PENALTY.NONE:
            # ロボットを停止させる
            pub.send_stop_walk()
            logger.action_executed("Robot is penalized. Sent stop command.")
            return s
        else:
            logger.action_postcondition_already_satisfied(details="Robot is unpenalized")
            print_in_real_action("wait_until_unpenalized", "Robot is unpenalized.")
            return s
    else:
        logger.action_postcondition_already_satisfied(details="Game state is not playing")
        print_in_real_action("wait_until_unpenalized", "Game state is not playing.")
        return s


def tackle_ball(s, pub: WalkCommandPublisher, logger: DecisionLogger):
    """
    ボールの位置に突っ込む。相手がいるいないは特に関係なし。
    ボールに向かっていく進行方向そのままに蹴る。
    蹴る方向等は歩き始めの位置くらいでしか調整できないため、クリア等のために使う
    """
    # preconditionを確認
    if (
        s.world_states["ball_position_relative"] != BALL_POSITION.UNKNOWN
        and s.ball_position_local is not None
    ):
        print_in_real_action("tackle_ball", " Executing tackle_ball().")
        obstacles = get_all_obstacles(s)
        planned_path, _ = pathplan.find_path_to_target(
            s.ball_position_local,
            obstacles,
            clearance=60.0,
            clearance_factor=pathplan.distance_clearance_factor(
                s.ball_position_local, obstacles, clearance=60.0
            ),
        )
        if not action_functions.validate_path(planned_path):
            # パスが見つからない場合、停止
            pub.send_walk_command(WalkCommand(0.0, 0.0, 0.0))
            logger.action_executed(details="No valid path found. Sent stop command.")
            return s
        cmd = action_functions.track_planned_path(planned_path, pub)
        pub.send_walk_command(cmd)
        # デバック用
        pub.send_target_position(
            convert_local_to_global(s.self_position_global, s.ball_position_local)
        )
        logger.action_executed(details="Tackling the ball")
        return s
    else:
        print_in_real_action("tackle_ball", "Ball is not in a position to approach.")
        logger.action_precondition_not_satisfied(details="Ball position unknown")
        return None


def approach_ball(s, pub: WalkCommandPublisher, logger: DecisionLogger):
    """
    Approach the ball at the given coordinates.
    """
    # preconditionを確認
    if (
        s.world_states["ball_position_relative"] == BALL_POSITION.DISTANT
        and s.ball_position_local is not None
    ):
        print_in_real_action("approach_ball", " Executing approach_ball().")
        obstacles = get_all_obstacles(s)
        planned_path, _ = pathplan.find_path_to_target(
            s.ball_position_local,
            obstacles,
            clearance=60.0,
            clearance_factor=pathplan.distance_clearance_factor(
                s.ball_position_local, obstacles, clearance=60.0
            ),
        )
        if not action_functions.validate_path(planned_path):
            # パスが見つからない場合、停止
            pub.send_walk_command(WalkCommand(0.0, 0.0, 0.0))
            logger.action_executed(details="No valid path found. Sent stop command.")
            return s
        cmd = action_functions.track_planned_path(planned_path, pub)
        pub.send_walk_command(cmd)
        # デバック用
        pub.send_target_position(
            convert_local_to_global(s.self_position_global, s.ball_position_local)
        )
        logger.action_executed(details="Approaching the ball")
        return s
    # 事後条件を既に満たしているので成功とする
    elif (
        s.world_states["ball_position_relative"]
        in [BALL_POSITION.NEAR, BALL_POSITION.IN_KICK_RANGE]
        and s.ball_position_local is not None
    ):
        print_in_real_action("approach_ball", "Ball is already near.")
        logger.action_postcondition_already_satisfied(details="Ball is already near")
        return s  # 近いのでアプローチは不要
    else:
        print_in_real_action("approach_ball", "Ball is not in a position to approach.")
        logger.action_precondition_not_satisfied(details="Ball is not in a position to approach")
        return None


def adjust_to_kick_position(s, pub: WalkCommandPublisher, logger: DecisionLogger):
    """
    Adjust the robot's position to be in kick range of the ball.
    """

    y_vel = 0.0
    theta_vel = 0.0
    theta_stop_range = 0.0

    if (
        robot_on_line(s.ball_position_local, get_goal_center_pos(s))
        and s.ball_position_local is not None
        and s.world_states["ball_position_relative"] == BALL_POSITION.NEAR
    ):
        print_in_real_action(
            "adjust_to_kick_position", "Executing adjust_to_kick_position()."
        )
        # cmd = action_functions.generate_turn_command(s.ball_position_local)
        # pub.send_walk_command(cmd)
        
        if s.ball_position_local.y > 0.0:
            y_vel = 20.0
        else:
            y_vel = -20.0

        pub.send_walk_command(WalkCommand(0.0, y_vel, theta_vel))
        logger.action_executed(details="Adjusting to kick position")
        return s
    elif s.world_states["ball_position_relative"] == BALL_POSITION.IN_KICK_RANGE:
        print_in_real_action("adjust_to_kick_position", "Already in kick range.")
        logger.action_postcondition_already_satisfied(details="Already in kick range")
        return s
    else:
        print_in_real_action(
            "adjust_to_kick_position",
            "Cannot adjust to kick position, ball is not near.",
        )
        logger.action_precondition_not_satisfied(details="Ball is not near")
        return None


def kick_ball(s, pub: WalkCommandPublisher, logger: DecisionLogger):
    """
    Kick the ball.
    """
    if (robot_on_line(s.ball_position_local, get_goal_center_pos(s))
        and s.world_states["ball_position_relative"] == BALL_POSITION.IN_KICK_RANGE
        ):
        print_in_real_action("kick_ball", "Executing kick_ball().")
        # cmd = WalkCommand(ROBOT_MAX_SPEED_X_PLUS * setting.DEFAULT_STRATEGY_CONFIG["common_config"]["kick_speed_x_scale"] , 0.0, 0.0)
        cmd = WalkCommand(60.0, 0.0, 0.0)
        pub.send_continuous_walk_command(cmd,continuous_time=1.2) # continuous_time秒分前進し続ける
        logger.action_executed(details="Kicking the ball")
        return s
    else:
        print_in_real_action("kick_ball", "Cannot kick the ball, not in kick range.")
        logger.action_precondition_not_satisfied(details="Cannot kick the ball, not in kick range")
        return s


def goal_kick_ball_position(s, pub: WalkCommandPublisher, logger: DecisionLogger):
    """
    move in a line between the goal and the ball
    """
    if (
        s.world_states["ball_position_relative"] == BALL_POSITION.DISTANT
        and s.ball_position_local is not None
    ):
        print_in_real_action(
            "goal_kick_ball_position", "Executing goal_kick_ball_position()."
        )
        target_position = get_behind_ball_position(
            s.ball_position_local,
            setting.DEFAULT_STRATEGY_CONFIG["common_config"][
                "get_behind_ball_position_space"
            ],
        )

        print_in_real_action(
            "goal_kick_ball_position", "Executing goal_kick_ball_position().pathplan"
        )
        obstacles = get_all_obstacles(s)
        obstacles.append(s.ball_position_local)
        planned_path, _ = pathplan.find_path_to_target(
            target_position,
            obstacles,
            clearance=60.0,
            clearance_factor=pathplan.distance_clearance_factor(
                s.ball_position_local, obstacles, clearance=60.0
            ),
        )
        if not action_functions.validate_path(planned_path):
            logger.action_executed(details="No valid path found. Do nothing.")
            return s
        cmd = action_functions.track_planned_path(planned_path, pub)
        if cmd is not None:
            pub.send_walk_command(cmd)
            logger.action_executed(details="Moving to behind ball position")
        # デバック用
        pub.send_target_position(
            convert_local_to_global(s.self_position_global, target_position)
        )
        return s
    # 事後条件：事前条件
    elif s.world_states["ball_position_relative"] in [
        BALL_POSITION.NEAR,
        BALL_POSITION.IN_KICK_RANGE]:
    # ] and robot_on_line(s.ball_position_local, get_goal_center_pos(s)):
        print_in_real_action(
            "goal_kick_ball_position", "ball is near."
        )
        logger.action_postcondition_already_satisfied(details="Ball is near")
        return s  # 近いのでアプローチは不要
    else:
        print_in_real_action("goal_kick_ball_position", "ball is NOT near.")
        logger.action_precondition_not_satisfied(details="Ball is not near")
        return s


def goal_kick_ball_around(s, pub: WalkCommandPublisher, logger: DecisionLogger):
    if (
        (s.world_states["ball_position_relative"] in [BALL_POSITION.NEAR, BALL_POSITION.IN_KICK_RANGE])
        and not robot_on_line(s.ball_position_local, get_goal_center_pos(s))
    ):
        print_utils.debug_print(f"around_position_x : {s.ball_position_local.x}")
        print_utils.debug_print(f"around_position_y : {s.ball_position_local.y}")
        # if s.world_states["ball_position_relative"] in [BALL_POSITION.NEAR, BALL_POSITION.IN_KICK_RANGE]:
        target_position = get_behind_position(
            s.ball_position_local,
            get_goal_center_pos(s),
            setting.DEFAULT_STRATEGY_CONFIG["common_config"][
                "get_behind_ball_position_space"
            ],
        )
        # if s.world_states["ball_position_relative"] == BALL_POSITION.IN_KICK_RANGE:
        print_in_real_action(
            "goal_kick_ball_around",
            "Executing goal_kick_ball_around().generate_center_circle_command",
        )
        cmd = action_functions.refine_generate_center_circle_command(
            s.ball_position_local, target_position
        )
        pub.send_walk_command(cmd)
        logger.action_executed(details="Moving around the ball")
        return s
    elif robot_on_line(s.ball_position_local, get_goal_center_pos(s)):
        print_in_real_action(
            "goal_kick_ball_around", "ball is on line"
        )
        logger.action_postcondition_already_satisfied(details="Ball is on line")
        return s
        # else:
        #     print_in_real_action(
        #         "goal_kick_ball_around",
        #         "Executing goal_kick_ball_around().generate_circle_command",
        #     )
        #     cmd = action_functions.generate_circle_command(
        #         s.ball_position_local, target_position
        #     )
        #     if cmd is not None:
        #         pub.send_walk_command(cmd)
        #     return s
    else:
        logger.action_precondition_not_satisfied(details="Ball is not on line")
        print_in_real_action("goal_kick_ball_around", "ball in NOT on line")
        return s


def goal_kick_ball_direction(s, pub: WalkCommandPublisher, logger: DecisionLogger):
    """
    Adjust the robot's direction to be in kick range of the ball into goal
    """
    if (
        s.world_states["ball_position_relative"] == BALL_POSITION.NEAR
        and robot_on_line(s.ball_position_local, get_goal_center_pos(s))
    ):
        print_in_real_action(
            "goal_kick_ball_direction", "Executing goal_kick_ball_direction()"
        )
        cmd = action_functions.generate_turn_command(s.ball_position_local)
        pub.send_walk_command(cmd)
        logger.action_executed(details="Adjusting ball kick direction towards goal")
        return s
    elif s.world_states[
        "ball_position_relative"
    ] == BALL_POSITION.IN_KICK_RANGE and robot_on_line(
        s.ball_position_local, get_goal_center_pos(s)
    ):
        print_in_real_action("goal_kick_ball_direction", "Already in kick range.")
        logger.action_postcondition_already_satisfied(details="Already in kick range")
        return s
    else:
        print_in_real_action(
            "goal_kick_ball_direction",
            "Cannot adjust to kick position, not on line between ball and goal.",
        )
        logger.action_precondition_not_satisfied(details="Not on line between ball and goal")
        return s


def gc_ready_command_move(s, pub: WalkCommandPublisher, logger: DecisionLogger):
    """
    レディコマンドを受けて、指定の位置に移動する。
    """
    if s.world_states["game_state"] == GAME_STATE.READY:
        print_in_real_action(
            "gc_ready_command_move", "Executing gc_ready_command_move()."
        )
        target_global_pose = convert_hardcoded_pos_to_our_field(s,get_start_position())
        if (s.self_position_global is not None) and utils.are_target_poses_near(
            s.self_position_global, target_global_pose, 40.0, 40.0, math.radians(15.0)
        ):
            print_in_real_action(
                "gc_ready_command_move", "Already in ready position."
            )
            pub.send_stop_command()
            logger.action_postcondition_already_satisfied(details="Already in ready position")
            return s
        cmd = action_functions.move_to_target_global_pose(s, pub, target_global_pose)
        if cmd is not None:
            pub.send_walk_command(cmd)
            logger.action_executed(details="Moving to ready position")
        else:
            logger.action_executed(details="No valid path found. Stop.")
            pub.send_stop_command()
        return s
    elif (
        s.world_states["game_state"] == GAME_STATE.SET
        or s.world_states["game_state"] == GAME_STATE.PLAYING
    ):
        print_in_real_action("gc_ready_command_move", "Already in ready state.")
        logger.action_postcondition_already_satisfied(details="Already in ready state")
        return s
    else:
        logger.action_precondition_not_satisfied(details="Game state is not ready")
        print_in_real_action("gc_ready_command_move", "Game state is not ready.")
        return s


def gc_set_command_wait(s, pub: WalkCommandPublisher, logger: DecisionLogger):
    """
    Set command to wait for the game to start.
    """
    if s.world_states["game_state"] == GAME_STATE.SET:
        print_in_real_action("gc_set_command_wait", "Executing gc_set_command_wait().")
        pub.send_walk_command(WalkCommand(0.0, 0.0, 0.0))  # 停止する
        logger.action_executed(details="Sent stop command in SET state")
        return s
    elif s.world_states["game_state"] == GAME_STATE.PLAYING:
        logger.action_postcondition_already_satisfied(details="Already in playing state")
        print_in_real_action("gc_set_command_wait", "Already in playing state.")
        return s
    else:
        logger.action_precondition_not_satisfied(details="Game state is not set")
        print_in_real_action("gc_set_command_wait", "Game state is not set.")
        return s


def do_nothing(s, pub: WalkCommandPublisher, logger: DecisionLogger):
    """
    何もしないアクション。プランニングを終了させるために使用
    """
    print_in_real_action("do_nothing", "Executing do_nothing().")
    # 動いている場合ロボットを停止させる
    if not pub.is_stop_now():
        pub.send_walk_command(WalkCommand(0.0, 0.0, 0.0))
        logger.action_executed(details="sent stop command")
        return s
    logger.action_postcondition_already_satisfied()
    return s


# Attackerがsecondaryかつ敵infoの場合のアクション
def approach_secondary_attacker_enemyball_position(s, pub: WalkCommandPublisher, logger: DecisionLogger):
    """
    ATTACKERの時にSECONDARYを受けて、指定の位置に移動する
    """
    target_global_pose = None

    # 自身とボールの位置が不明な場合は、ボール探索タスクに切り替える
    if s.self_position_global is None or s.ball_position_local is None:
        print_in_real_action(
            "approach_defense_position",
            "Self or ball position is unknown. Cannot determine target.",
        )
        logger.action_precondition_not_satisfied(details="Self or ball position is unknown")
        return s

    # ボールのグローバル座標
    ball_position_global = convert_local_to_global(
        s.self_position_global, s.ball_position_local
    )

    # ボールのx座標が-450より低い場合、目標x座標を-500に固定
    target_x = (
        -500.0 if ball_position_global.x < -350.0 else ball_position_global.x - 150
    )
    # y座標がプラスかマイナスかでオフセットを調整
    y_offset = 250.0 if ball_position_global.y >= 0 else -250.0

    if s.world_states["secondary_state"] in [
        SECONDARY_STATE.DIRECT_FREEKICK,
        SECONDARY_STATE.INDIRECT_FREEKICK,
        SECONDARY_STATE.THROW_IN,
    ]:
        # ボールから一定距離離れた位置（ゴール方向とは逆側）
        target_global_pose = convert_hardcoded_pos_to_our_field(s,Position(Position.COORD_GLOBAL, target_x, y_offset, 0.0))
    elif s.world_states["secondary_state"] == SECONDARY_STATE.CORNER_KICK:
        # ペナルティエリアの角
        target_global_pose = convert_hardcoded_pos_to_our_field(s,Position(Position.COORD_GLOBAL, -600.0, y_offset, 0.0))
    elif s.world_states["secondary_state"] == SECONDARY_STATE.GOAL_KICK:
        # ゴールキック
        target_global_pose = convert_hardcoded_pos_to_our_field(s,Position(Position.COORD_GLOBAL, -300.0, y_offset, 0.0))
    elif s.world_states["secondary_state"] == SECONDARY_STATE.PENALTYKICK:
        # ペナルティキックの待機場所、defenderがゴールを守っている
        target_global_pose = convert_hardcoded_pos_to_our_field(s,Position(Position.COORD_GLOBAL, -350.0, -200.0, 0.0))
    else:
        target_global_pose = convert_hardcoded_pos_to_our_field(s,Position(Position.COORD_GLOBAL, -450.0, 0.0, 0.0))

    # 目標位置と現在のロボットの位置を比較し、距離を計算する
    distance_to_target = calculate_distance_global(
        s.self_position_global, target_global_pose
    )

    # 目標に十分近づいたら停止する
    if distance_to_target < 20.0:
        print_in_real_action(
            "approach_secondary_attacker_enemyball_position",
            "Already near target. Stopping.",
        )
        # ローカル座標のボールの角度を計算
        ball_angle = calculate_angle(s.ball_position_local)

        # 首の動きと体の旋回を調整
        # 首をボールの方向へ
        # pub.move_neck_yaw(yaw_angle=-ball_angle)
        # 体の向きを調整するための旋回コマンドを生成
        cmd = action_functions.generate_turn_command(s.ball_position_local)
        pub.send_walk_command(cmd)
        logger.action_postcondition_already_satisfied(details="Turn to near target")

        return s
    else:
        # グローバル座標の目標地点をロボットのローカル座標に変換
        target_local_pose = convert_global_to_local(
            s.self_position_global, target_global_pose
        )

        # 障害物回避のためのパスプランニング
        obstacles = get_all_obstacles(s)
        planned_path, _ = pathplan.find_path_to_target(
            target_local_pose, obstacles, clearance=60.0
        )

        if not action_functions.validate_path(planned_path):
            # パスが見つからない場合、停止
            pub.send_walk_command(WalkCommand(0.0, 0.0, 0.0))
            logger.action_executed(details="No valid path found. Sent stop command.")
            return s

        # 計画されたパスに沿って移動するコマンドを生成
        cmd = action_functions.track_planned_path(planned_path, pub)
        pub.send_walk_command(cmd)
        logger.action_executed(details="Moving to secondary attacker position")
        return s


# defender action
def approach_defense_position(s, pub: WalkCommandPublisher, logger: DecisionLogger):
    """
    ボールとゴールの間に移動する、またはボールが見えない場合は指定の防御位置に移動
    """
    # 自身とボールの位置が不明な場合は、ボール探索タスクに切り替える
    if s.self_position_global is None or s.ball_position_local is None:
        print_in_real_action(
            "approach_defense_position",
            "Self or ball position is unknown. Initiating ball search.",
        )
        # ボール探索タスクに切り替える
        s.world_states["ball_position_relative"] = BALL_POSITION.UNKNOWN
        logger.action_precondition_not_satisfied(details="Self or ball position is unknown")
        return s

    # ボールが見えている場合の処理
    if s.world_states["ball_position_relative"] == BALL_POSITION.DISTANT:
        print_in_real_action(
            "approach_defense_position", "Executing approach_defense_position()."
        )

        # ボールのグローバル座標
        ball_position_global = convert_local_to_global(
            s.self_position_global, s.ball_position_local
        )

        # ボールのy座標をチェックし、±1300の範囲内に制限する
        if ball_position_global.x > -100.0:
            defense_x = -500.0
        else:
            goal_x = -700.0
            defense_x = goal_x + (ball_position_global.x - goal_x) / 4

        # ボールのy座標をチェックし、±1300の範囲内に制限する
        if ball_position_global.y > 130:
            defense_y = 130.0
        elif ball_position_global.y < -130:
            defense_y = -130.0
        else:
            defense_y = ball_position_global.y

        defense_position_global = convert_hardcoded_pos_to_our_field(s,Position(
            Position.COORD_GLOBAL, defense_x, defense_y, 0.0
        ))

        # 目標位置とロボットの現在の位置のグローバル座標間の距離を計算
        distance_to_target = calculate_distance_global(
            s.self_position_global, defense_position_global
        )

        # ロボットが目標位置に十分に近づいたら、ボールの方向を向く
        if distance_to_target < 30.0:  # 距離が20mm以内なら到着したとみなす
            print_in_real_action(
                "approach_defense_position",
                "Reached defense position. Turning to face the ball.",
            )
            # ローカル座標のボールの角度を計算
            ball_angle = calculate_angle(s.ball_position_local)

            # 首の動きと体の旋回を調整
            # 首をボールの方向へ
            # pub.move_neck_yaw(yaw_angle=-ball_angle)
            # 体の向きを調整するための旋回コマンドを生成
            cmd = action_functions.generate_turn_command(s.ball_position_local)
            pub.send_walk_command(cmd)

            logger.action_postcondition_already_satisfied(details="Turn to face the ball")
            return s

        else:
            print_in_real_action(
                "approach_defense_position",
                f"Target defense position: {defense_position_global}",
            )
            # move_to_target_global_pose関数を使用して目標座標に移動
            cmd = action_functions.move_to_target_global_pose(
                s, pub, defense_position_global
            )
            if cmd is not None:
                pub.send_walk_command(cmd)

            # デバッグ用に目標位置を可視化
            print_in_real_action(
                "approach_defense_position",
                f"Target defense position: {defense_position_global}",
            )
            logger.action_executed(details=f"Moving to defense position: {defense_position_global}")
            return s

    # ボールが近い場合は停止 ここでattackerに切り替わってほしい
    elif s.world_states["ball_position_relative"] in [
        BALL_POSITION.NEAR,
        BALL_POSITION.IN_KICK_RANGE,
    ]:
        print_in_real_action(
            "approach_defense_position", "Ball is near. Stopping defense approach."
        )
        pub.send_walk_command(WalkCommand(0.0, 0.0, 0.0))
        logger.action_postcondition_already_satisfied(details="Ball is near, stop defense approach.")
        return s

    # ボールが見えない場合
    else:
        print_in_real_action(
            "approach_defense_position",
            "Ball is not visible. Moving to a default defense position.",
        )

        # ボールが見えない場合のデフォルトの防御位置（ゴール前の固定位置など）
        default_defense_position = Position(
            Position.COORD_GLOBAL,
            -600.0,
            0.0,
            0.0,
        )

        cmd = action_functions.move_to_target_global_pose(
            s, pub, default_defense_position
        )
        if cmd is not None:
            pub.send_walk_command(cmd)

        # デバッグ用に目標位置を可視化
        print_in_real_action(
            "approach_defense_position",
            f"Target defense position: {defense_position_global}",
        )
        logger.action_executed(details="Moving to default defense position")
        return s


def approach_secondary_defense_position(s, pub: WalkCommandPublisher, logger: DecisionLogger):
    """
    ROLEがDEFENDERの時にSECONDARYを受けて、指定の位置に移動する。
    """
    print_in_real_action(
        "approach_defense_position",
        f"現在のsecondary_state: {s.world_states['secondary_state']}",
    )

    if s.world_states["secondary_state"] in [
        SECONDARY_STATE.DIRECT_FREEKICK,
        SECONDARY_STATE.INDIRECT_FREEKICK,
        SECONDARY_STATE.PENALTYKICK,
        SECONDARY_STATE.CORNER_KICK,
        SECONDARY_STATE.GOAL_KICK,
        SECONDARY_STATE.THROW_IN,
    ]:
        print_in_real_action(
            "approach_secondary_defense_position",
            "Executing approach_secondary_defense_position().",
        )
        secondary_defender_target_global_pose = convert_hardcoded_pos_to_our_field(s,Position(
            Position.COORD_GLOBAL,
            -670.0,
            0.0,
            0.0,
        ))
        cmd = action_functions.move_to_target_global_pose(
            s, pub, secondary_defender_target_global_pose
        )
        if cmd is not None:
            pub.send_walk_command(cmd)
            logger.action_executed(details="Moving to secondary defense position")
        else:
            logger.action_executed(details="No valid path found.")
        return s
    else:
        print_in_real_action(
            "approach_secondary_defense_position", "Game state is not ready."
        )
        logger.action_precondition_not_satisfied(details="Game state is not ready")
        return s

# attackerが2体いる場合の別ポジション移動
def approach_secondary_attacker_follower(s, pub: WalkCommandPublisher, logger: DecisionLogger):
    """
    Attackerが2体目いる場合にSECONDARYを受けて、指定の位置に移動する。
    """
    if s.world_states["secondary_state"] in [
        SECONDARY_STATE.DIRECT_FREEKICK,
        SECONDARY_STATE.INDIRECT_FREEKICK,
        SECONDARY_STATE.PENALTYKICK,
        SECONDARY_STATE.CORNER_KICK,
        SECONDARY_STATE.GOAL_KICK,
        SECONDARY_STATE.THROW_IN,
    ]:
        print_in_real_action(
            "approach_secondary_attacker_follower",
            "Executing approach_secondary_attacker_follower().",
        )
        # 味方local、ボールlocal、自分のglobalがある場合
        if s.allies_position_local and s.ball_position_local is not None:
            # s.allies_position_localはPositionオブジェクトのリストなので、最初の要素を取得する
            first_ally_local_position = s.allies_position_local[0]

            # 自分とボールの差分、味方とボールの差分を比較して
            # ボールのグローバル座標
            ball_gl = convert_local_to_global(s.self_position_global, s.ball_position_local)
            allies_gl = convert_local_to_global(
            s.self_position_global, first_ally_local_position
            )

            diff_ball_alies = calculate_distance_global(ball_gl, allies_gl)
            diff_ball_self_position = calculate_distance_global(
            ball_gl, s.self_position_global
            )

            if diff_ball_self_position > diff_ball_alies:
                # 味方と一定の距離を保って、追従する
                # ペナルティエリアよりボールが後ろにあるとそれ以上下がらない
                if ball_gl.x > -450.0:
                    allies_follower_pos_x = ball_gl.x - 150.0
                else:
                    allies_follower_pos_x = -450.0

                # ボールのy座標から離れる
                if ball_gl.y > 0.0:
                    allies_follower_pos_y = ball_gl.y - 150.0
                else:
                    allies_follower_pos_y = ball_gl.y + 150.0

                follower_position_global = Position(
                    Position.COORD_GLOBAL, allies_follower_pos_x, allies_follower_pos_y, 0.0
                )

                # 目標位置とロボットの現在の位置のグローバル座標間の距離を計算
                distance_to_target = calculate_distance_global(
                    s.self_position_global, follower_position_global
                )

                # ロボットが目標位置に十分に近づいたら、ボールの方向を向く
                if distance_to_target < 150.0:  # 距離が150mm以内なら到着したとみなす
                    cmd = action_functions.generate_turn_command(s.ball_position_local)
                    pub.send_walk_command(cmd)
                    logger.action_postcondition_already_satisfied(details="Turn to face the ball")
                    return s
                else:
                    cmd = action_functions.move_to_target_global_pose(
                        s, pub, follower_position_global
                    )
                    if cmd is not None:
                        pub.send_walk_command(cmd)
                    logger.action_executed(details="Moving to follower position")

                    return s
    # ボールが見えない場合
    else:
        print_in_real_action(
            "approach_attacker_follower",
            "Ball is not visible. Moving to a default follower position.",
        )

        # ボールが見えない場合の自陣でボールが探しやすい場所に移動
        default_follower_position = Position(
            Position.COORD_GLOBAL,
            0.0,
            0.0,
            0.0,
        )
        cmd = action_functions.move_to_target_global_pose(
            s, pub, default_follower_position
        )
        if cmd is not None:
            pub.send_walk_command(cmd)
        
        logger.action_executed(details="Moving to default follower position")

        return s

def walk_to_default_position(s, pub: WalkCommandPublisher, logger: DecisionLogger):
    """
    ロボットをデフォルトの位置に移動させるアクション
    """
    print_in_real_action("walk_to_default_position", "Executing walk_to_default_position().")
    target_global_pose = utils.get_start_position()
    cmd = action_functions.move_to_target_global_pose(s, pub, target_global_pose)
    if cmd is not None:
        pub.send_walk_command(cmd)
    else:
        pub.send_stop_command()
    logger.action_executed(details="Move to default position")
    return s

def walk_straight_to_enter_field(s, pub: WalkCommandPublisher, logger: DecisionLogger):
    """
    ロボットをフィールドにまっすぐ入場させるアクション
    """
    print_in_real_action("walk_straight_to_enter_field", "Executing walk_straight_to_enter_field().")
    cmd = WalkCommand(x_velocity=40.0, y_velocity=0.0, theta_velocity=0.0)
    pub.send_walk_command(cmd)
    logger.action_executed(details="Walk straight to enter field")
    return s

REAL_ACTIONS_DICT = {
    "approach_ball": approach_ball,
    "adjust_to_kick_position": adjust_to_kick_position,
    "kick_ball": kick_ball,
    "goal_kick_ball_position": goal_kick_ball_position,
    "goal_kick_ball_around": goal_kick_ball_around,
    "goal_kick_ball_direction": goal_kick_ball_direction,
    "gc_ready_command_move": gc_ready_command_move,
    "gc_set_command_wait": gc_set_command_wait,
    "do_nothing": do_nothing,
    "wait_until_unpenalized": wait_until_unpenalized,
    "approach_secondary_attacker_enemyball_position": approach_secondary_attacker_enemyball_position,
    "tackle_ball": tackle_ball,
    "approach_defense_position": approach_defense_position,
    "approach_secondary_defense_position": approach_secondary_defense_position,
    "approach_secondary_attacker_follower": approach_secondary_attacker_follower,
    "walk_to_default_position":  walk_to_default_position,
    "walk_straight_to_enter_field": walk_straight_to_enter_field,
}

gtpyhop.declare_actions(
    approach_ball,
    adjust_to_kick_position,
    kick_ball,
    goal_kick_ball_position,
    goal_kick_ball_around,
    goal_kick_ball_direction,
    gc_ready_command_move,
    gc_set_command_wait,
    do_nothing,
    wait_until_unpenalized,
    approach_secondary_attacker_enemyball_position,
    tackle_ball,
    approach_defense_position,
    approach_secondary_defense_position,
    approach_secondary_attacker_follower,
    walk_straight_to_enter_field,
    walk_to_default_position
)
