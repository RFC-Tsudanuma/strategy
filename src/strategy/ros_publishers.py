import json
import math
import os
import threading
import time
import asyncio

import gtpyhop
import sys
import typing
from ament_index_python.packages import get_package_share_directory
from booster_msgs.msg import RpcReqMsg
from futbol_msgs.msg import ObjectPosition, StrategyState, NeckMovementMode, FloatWalkCommand, DecisionLog
from localization_msgs.srv import SetInitialPose
from rclpy.node import Node
from state import OUR_FIELD, Position
from strategy_time import get_strategy_time
import subprocess

def in_the_t1_robot():
    """
    T1ロボット上で動作しているかを判定する。
    これは単にJetsonで動いているかを判定しているだけ。他の環境でもjetsonを使うなら改良する必要あり。
    """
    result = subprocess.run(['uname', '-a'], capture_output=True, text=True)
    uname_str = result.stdout.strip()
    if "rt-tegra" in uname_str:
        return True
    return False

def fast_print(string: str):
    """
    標準のprint関数よりも高速に動作するprint関数
    """
    sys.stdout.write(string + '\n')

class DDSPublisherProxy:
    """
    T1ロボット上で動作している場合、DDS Publisherを利用するためのプロキシクラス
    """

    def __init__(self):
        self.in_t1 = in_the_t1_robot()
        if self.in_t1:
            from booster_robotics_sdk_python import B1LocoClient, ChannelFactory, B1LocoApiId
            ChannelFactory.Instance().Init(0,"127.0.0.1")
            self.client = B1LocoClient()
            self.client.Init()
            time.sleep(1.0) # 少し待つ
            self.walk_api_id = B1LocoApiId.kMove

    def publish(self, vx: float, vy: float = 0.0, vyaw: float = 0.0):
        if self.in_t1:
            self.client.Move(vx=vx, vy=vy, vyaw=vyaw)
        else:
            # T1ロボット上でない場合は何もしない
            return


class WalkCommand:
    x_velocity: float
    y_velocity: float
    theta_velocity: float
    continuous_time: float

    def __init__(
        self, x_velocity=0.0, y_velocity=0.0, theta_velocity=0.0, continuous_time=None
    ):
        self.x_velocity = x_velocity
        self.y_velocity = y_velocity
        self.theta_velocity = theta_velocity
        self.continuous_time = continuous_time

    def __repr__(self) -> str:
        return f"WalkCommand(x_velocity={self.x_velocity}, y_velocity={self.y_velocity}, theta_velocity={self.theta_velocity})"

def command_is_same(cmd1: typing.Optional[WalkCommand], cmd2: typing.Optional[WalkCommand]) -> bool:
    """
    2つのWalkCommandが同じかどうかを判定する
    """
    if cmd1 is None and cmd2 is None:
        return True
    if cmd1 is None or cmd2 is None:
        return False
    if (
        math.isclose(cmd1.x_velocity, cmd2.x_velocity,abs_tol=1e-4)
        and math.isclose(cmd1.y_velocity, cmd2.y_velocity,abs_tol=1e-4)
        and math.isclose(cmd1.theta_velocity, cmd2.theta_velocity,abs_tol=1e-4)
    ):
        return True
    return False

class WalkCommandPublisher(Node):
    def __init__(self, sim_robot_id=None):
        super().__init__("strategy_walk_command_publisher")
        self.command_continuous_time = None
        self.command_continuous_time_lock = threading.Lock()
        self.timer_handler = None
        self.current_walk_cmd = None
        self.current_neck_mode = NeckMovementMode().MODE_IDLE
        self.event_loop = asyncio.new_event_loop()
        self.event_loop_thread = threading.Thread(target=self.event_loop.run_forever)
        self.event_loop_thread.daemon = True
        self.event_loop_thread.start()
        self.last_walk_command_send_time = get_strategy_time()
        if sim_robot_id is not None:
            self.publisher_ = self.create_publisher(
                RpcReqMsg, "/LocoApiTopicReq_" + str(sim_robot_id), 1
            )
            self.neck_publisher_ = self.create_publisher(
                NeckMovementMode, "/NeckMovement_" + str(sim_robot_id), 1
            )
            self.debug_publisher_ = self.create_publisher(
                StrategyState, "/StrategyState_" + str(sim_robot_id), 1
            )
            self.targetpos_publisher_ = self.create_publisher(
                ObjectPosition, "/TargetPosition_" + str(sim_robot_id), 1
            )
            self.decision_log_publisher = self.create_publisher(
                DecisionLog, "/DecisionLog_" + str(sim_robot_id), 1
            )
            self.dds_client: typing.Optional[DDSPublisherProxy] = None
            fast_print(
                f"[WalkPub] WalkCommandPublisher initialized for sim_robot_id {sim_robot_id}"
            )
        else:
            self.publisher_ = self.create_publisher(RpcReqMsg, "/LocoApiTopicReq", 1)
            self.dds_client: typing.Optional[DDSPublisherProxy] = DDSPublisherProxy()
            self.walk_command_logger = self.create_publisher(FloatWalkCommand, "/FloatWalkCommandLog", 1)
            self.neck_publisher_ = self.create_publisher(
                NeckMovementMode, "/NeckMovement", 1
            )
            self.debug_publisher_ = self.create_publisher(
                StrategyState, "/StrategyState", 1
            )
            self.targetpos_publisher_ = self.create_publisher(
                ObjectPosition, "/TargetPosition", 1
            )
            self.decision_log_publisher = self.create_publisher(
                DecisionLog, "/DecisionLog", 1
            )
        self.selfpos_set_pose_client = self.create_client(
                SetInitialPose,
                'set_initial_pose'
                )
        package_share_dir = get_package_share_directory("strategy")
        self.yaml_file_path = os.path.join(
            package_share_dir, "config", "strategy_config.yaml"
        )

    def __del__(self):
        self.event_loop.call_soon_threadsafe(self.event_loop.stop)
        self.event_loop_thread.join()

    def set_entry_pose(self, entry_side: str):
        request = SetInitialPose.Request()
        if entry_side == "right":
            request.x = -3.50  # これはm単位
            request.y = 4.50
            request.theta = math.radians(-90.0)
        else:
            request.x = -3.50  # これはm単位
            request.y = -4.50
            request.theta = math.radians(90.0)
        request.reset_particles = True
        resp = self.selfpos_set_pose_client.call_async(request)
        import time
        start = time.time()
        while True:
            if resp.done():
                break
            else:
                time.sleep(0.1)
                if time.time() - start > 2.0:
                    break
        if resp.done:
            print("Set Pose Service request suceeded!!")
        else:
            print("Set Pose Service request failed....")

    def send_stop_command(self):
        """
        歩行停止コマンドを送信する
        """
        cmd = WalkCommand(0.0, 0.0, 0.0)
        self.send_walk_command(cmd)


    def send_walk_command(self, cmd: WalkCommand):
        """
        歩行コマンドを送信する
        cmd: WalkCommandオブジェクト
        WARNING!!! このメソッドに渡す速度はセンチ単位であることに注意！
        """
        # 前のコマンドが継続期間中の場合は送信しない
        with self.command_continuous_time_lock:
            if self.command_continuous_time is not None:
                fast_print("[WalkPub] Previous continuous command is still valid. Not sending new command.")
                return
        # 継続時間指定が無効な場合のみコマンドを送信
        fast_print(f"[WalkPub] Sending walk command: {cmd}")
        msg = RpcReqMsg()
        msg.uuid = "12345678"
        msg.header = json.dumps(
            {"api_id": 2001}, separators=(",", ":"), ensure_ascii=True
        )
        msg.body = json.dumps(
            {
                "vx": cmd.x_velocity / 100.0,
                "vy": cmd.y_velocity / 100.0,
                "vyaw": cmd.theta_velocity,
            },
            separators=(",", ":"),
            ensure_ascii=True,
        )
        is_same_command = command_is_same(self.current_walk_cmd, cmd)
        self.current_walk_cmd = cmd # 今実行しているコマンドを記録
        if self.dds_client is not None: # T1ロボット上で動作している場合、DDSでパブリッシュする
            self.dds_client.publish(cmd.x_velocity / 100.0, cmd.y_velocity / 100.0, cmd.theta_velocity)
            logcmd = FloatWalkCommand()
            logcmd.x_vel = cmd.x_velocity / 100.0
            logcmd.y_vel = cmd.y_velocity / 100.0
            logcmd.theta_vel = cmd.theta_velocity
            self.walk_command_logger.publish(logcmd)
        else:
            self.publisher_.publish(msg)
        if not is_same_command:
            # 同じコマンドだった場合は継続時間はリセットしない。また、継続時間中も当然リセットなし。
            self.last_walk_command_send_time = get_strategy_time()
        # 継続時間指定がある場合、継続時間を設定し、タイマーをセットする
        if cmd.continuous_time is not None:
            with self.command_continuous_time_lock:
                # 継続時間を設定
                self.command_continuous_time = cmd.continuous_time + time.clock_gettime(
                    time.CLOCK_MONOTONIC
                )
            def schedule_timeout():
                self.event_loop.call_later(
                    cmd.continuous_time + 0.02, self.__check_continuous_command_timeout
                )
            self.event_loop.call_soon_threadsafe(schedule_timeout)
        # 継続時間指定が無い場合、終了時間をNoneに設定
        with self.command_continuous_time_lock:
            if cmd.continuous_time is None:
                self.command_continuous_time = None
                # タイマーもキャンセルしておく
                if self.timer_handler is not None:
                    self.timer_handler.cancel()

# 首動作関連のメソッド -------------------------------------------------------
    def __send_neck_command(self, move_mode: int, target_position: typing.Optional[Position] = None) -> None:
        """
        首の動作コマンドを送信する
        move_mode: 0=停止, 1=ボール追従, 2=指定位置を見る, 3=ボール探索, 4=ランドマーク探索 (msgを参照)
        target_position: 位置指定モード時に使用するターゲット位置
        """
        if move_mode < NeckMovementMode.MODE_IDLE or move_mode > NeckMovementMode.MODE_SEARCH_LANDMARK:
            raise ValueError("Invalid neck movement mode.")
        msg = NeckMovementMode()
        msg.movement_mode = move_mode
        if target_position is not None:
            pos = ObjectPosition()
            pos.x = target_position.x
            pos.y = target_position.y
            msg.target_position_local = pos
        self.neck_publisher_.publish(msg)

    def set_neck_idle_mode(self) -> None:
        """
        首の動作を停止モードに設定する
        """
        self.__send_neck_command(NeckMovementMode.MODE_IDLE)
        self.current_neck_mode = NeckMovementMode.MODE_IDLE

    def set_neck_ball_tracking_mode(self, ball_pos_local: ObjectPosition) -> None:
        """
        首の動作をボール追従モードに設定する
        """
        self.__send_neck_command(NeckMovementMode.MODE_BALL_TRACKING, target_position=ball_pos_local)
        self.current_neck_mode = NeckMovementMode.MODE_BALL_TRACKING

    def set_neck_look_position_mode(self, target_pos_local: Position) -> None:
        """
        首の動作を指定位置注視モードに設定する
        """
        self.__send_neck_command(NeckMovementMode.MODE_LOOK_POSITION, target_position=target_pos_local)
        self.current_neck_mode = NeckMovementMode.MODE_LOOK_POSITION

    def set_neck_search_ball_mode(self) -> None:
        """
        首の動作をボール探索モードに設定する
        """
        self.__send_neck_command(NeckMovementMode.MODE_SEARCH_BALL)
        self.current_neck_mode = NeckMovementMode.MODE_SEARCH_BALL

    def set_neck_search_landmark_mode(self) -> None:
        """
        首の動作をランドマーク探索モードに設定する
        """
        self.__send_neck_command(NeckMovementMode.MODE_SEARCH_LANDMARK)
        self.current_neck_mode = NeckMovementMode.MODE_SEARCH_LANDMARK

    def set_neck_search_from_target_position_mode(self, target_pos_local: Position) -> None:
        """
        首の動作を指定位置基準探索モードに設定する
        """
        self.__send_neck_command(NeckMovementMode.MODE_SEARCH_FROM_TARGET_POSITION, target_position=target_pos_local)
        self.current_neck_mode = NeckMovementMode.MODE_SEARCH_FROM_TARGET_POSITION

    def get_current_neck_mode(self) -> int:
        """
        現在の首の動作モードを取得する
        """
        return self.current_neck_mode

# ---------------------------------------------------------------------------

    def send_continuous_walk_command(self, cmd: WalkCommand, continuous_time: typing.Optional[float] = None):
        """
        継続時間を指定して歩行コマンドを送信する。
        前回の継続時間指定コマンドが有効な場合は、それを上書きする。
        上書きをしたくない場合、呼び出し側の責任で、is_continuous_command_valid()を確認すること。
        """
        if cmd.continuous_time is None and continuous_time is None:
            raise ValueError("Continuous time must be set for continuous walk command.")
        if continuous_time is not None:
            cmd.continuous_time = continuous_time
        self.send_walk_command(cmd)

    def send_stop_walk(self):
        cmd = WalkCommand(
            0.0,
            0.0,
            0.0,
        )
        self.send_walk_command(cmd)

    def is_walking_with_lin_vel_now(self) -> bool:
        """
        現在x,y方向への速度を伴って歩行しているかを確認する
        """
        if self.current_walk_cmd is None:
            return False
        if math.fabs(self.current_walk_cmd.x_velocity) > 0.1 or math.fabs(self.current_walk_cmd.y_velocity) > 0.1:
            return True
        else:
            return False
    
    def is_turning_with_only_theta_vel_now(self) -> bool:
        """
        現在theta方向への速度のみで歩行しているかを確認する
        """
        if self.current_walk_cmd is None:
            return False
        if math.fabs(self.current_walk_cmd.theta_velocity) > 0.1 and \
            math.fabs(self.current_walk_cmd.x_velocity) <= 0.01 and \
                math.fabs(self.current_walk_cmd.y_velocity) <= 0.01:
            return True
        else:
            return False

    def is_stop_now(self) -> bool:
        if self.current_walk_cmd is None:
            return True
        if math.fabs(self.current_walk_cmd.x_velocity) > 0.1 or \
            math.fabs(self.current_walk_cmd.y_velocity) > 0.1 or \
                math.fabs(self.current_walk_cmd.theta_velocity) > 0.1:
            return False
        else:
            return True

    def is_continuous_command_valid(self):
        """
        今現在、継続時間指定の歩行コマンドが有効かどうかを確認する
        """
        with self.command_continuous_time_lock:
            if self.command_continuous_time is None:
                return False
            current_time = time.clock_gettime(time.CLOCK_MONOTONIC)
            return self.command_continuous_time >= current_time

    def get_current_command_elapsed_time(self) -> float:
        """
        最後に歩行コマンドを送ってからの経過時間を取得する。
        """
        return get_strategy_time() - self.last_walk_command_send_time

    def __check_continuous_command_timeout(self) -> None:
        # 継続時間指定が有効であり、かつ現在の時間が継続時間を超えている場合、コマンドが無効だとみなすようにする
        # これはストップコマンド等を送信するわけではないので、ユーザ側で停止コマンドなどを送る必要がある
        current_time = time.clock_gettime(time.CLOCK_MONOTONIC)
        # pub_command = False
        with self.command_continuous_time_lock:  # ロック取得
            if (self.command_continuous_time is not None) and (
                current_time > self.command_continuous_time
            ):
                # 継続時間が過ぎている場合、コマンドを無効化
                self.command_continuous_time = None
                # pub_command = True
        # if pub_command:
        #     cmd = WalkCommand()
        #     self.send_walk_command(cmd)

    def send_standup_command(self):
        """
        立ち上がりコマンドを送信する
        """
        msg = RpcReqMsg()
        msg.uuid = "12345678"
        msg.header = json.dumps(
            {
                "api_id": 2008,
            }
        )
        msg.body = json.dumps({})  # TODO: これあってるか確認する
        self.publisher_.publish(msg)

    def send_debug_state(self, state: gtpyhop.State, strategy_hz: float = 0.0, detected_ballpos: Position = None):
        """
        戦略が内部で認識している状態をデバッグ用にパブリッシュする
        """
        debug_state = StrategyState()
        debug_state.strategy_hz = strategy_hz
        # 定数で表される状態
        debug_state.role = state.role.value
        debug_state.robot_mode = state.robot_mode.value
        debug_state.ball_position_relative = state.world_states[
            "ball_position_relative"
        ].value
        debug_state.current_robot_field = state.world_states[
            "current_robot_field"
        ].value
        debug_state.penalty = state.world_states["penalty"].value
        debug_state.game_state = state.world_states["game_state"].value
        debug_state.secondary_state = state.world_states["secondary_state"].value
        if state.world_states["our_field"] is not None:
            if state.world_states["our_field"] == OUR_FIELD.LEFT:
                debug_state.our_field = "Left"
            elif state.world_states["our_field"] == OUR_FIELD.RIGHT:
                debug_state.our_field = "Right"
        debug_state.team_color = state.world_states["team_color"].value
        # 位置情報
        if state.self_position_global is not None:
            debug_state.self_position_global.x = state.self_position_global.x
            debug_state.self_position_global.y = state.self_position_global.y
            debug_state.self_position_global.theta = state.self_position_global.theta
            debug_state.self_position_global.coord = state.self_position_global.coord
        if state.ball_position_local is not None:
            debug_state.ball_position_local.x = state.ball_position_local.x
            debug_state.ball_position_local.y = state.ball_position_local.y
            debug_state.ball_position_local.theta = state.ball_position_local.theta
            debug_state.ball_position_local.coord = state.ball_position_local.coord
        if state.shared_ball_global is not None:
            debug_state.shared_ball_global.x = state.shared_ball_global.x
            debug_state.shared_ball_global.y = state.shared_ball_global.y
            debug_state.shared_ball_global.theta = state.shared_ball_global.theta
            debug_state.shared_ball_global.coord = state.shared_ball_global.coord
        if detected_ballpos is not None:
            debug_state.ball_detected_by_camera.x = detected_ballpos.x
            debug_state.ball_detected_by_camera.y = detected_ballpos.y
            debug_state.ball_detected_by_camera.theta = detected_ballpos.theta
            debug_state.ball_detected_by_camera.coord = detected_ballpos.coord
        for pos in state.opponent_position_local:
            obj_pos = ObjectPosition()
            obj_pos.x = pos.x
            obj_pos.y = pos.y
            obj_pos.theta = pos.theta
            obj_pos.coord = pos.coord
            debug_state.opponent_position_local.append(obj_pos)
        for pos in state.normal_obstacles_position_local:
            obj_pos = ObjectPosition()
            obj_pos.x = pos.x
            obj_pos.y = pos.y
            obj_pos.theta = pos.theta
            obj_pos.coord = pos.coord
            debug_state.normal_obstacles_position_local.append(obj_pos)
        for pos in state.allies_position_local:
            obj_pos = ObjectPosition()
            obj_pos.x = pos.x
            obj_pos.y = pos.y
            obj_pos.theta = pos.theta
            obj_pos.coord = pos.coord
            debug_state.allies_position_local.append(obj_pos)
        for pos in state.opponent_goalpost_local:
            obj_pos = ObjectPosition()
            obj_pos.x = pos.x
            obj_pos.y = pos.y
            obj_pos.theta = pos.theta
            obj_pos.coord = pos.coord
            debug_state.opponent_goalpost_local.append(obj_pos)
        for pos in state.opponent_goalpost_global:
            obj_pos = ObjectPosition()
            obj_pos.x = pos.x
            obj_pos.y = pos.y
            obj_pos.theta = pos.theta
            obj_pos.coord = pos.coord
            debug_state.opponent_goalpost_global.append(obj_pos)

        # その他の情報
        if state.neck_yaw_rad is not None:
            debug_state.neck_yaw_rad = state.neck_yaw_rad
        else:
            debug_state.neck_yaw_rad = 0.0
        self.debug_publisher_.publish(debug_state)

    def send_target_position(self, target_pos: Position):
        """
        ターゲット位置をパブリッシュする
        """
        if target_pos is None or target_pos.coord != Position.COORD_GLOBAL:
            return
        msg = ObjectPosition()
        msg.x = target_pos.x
        msg.y = target_pos.y
        msg.theta = target_pos.theta
        msg.coord = target_pos.coord
        self.targetpos_publisher_.publish(msg)

    def send_decision_log(self, decision_log: DecisionLog):
        self.decision_log_publisher.publish(decision_log)
