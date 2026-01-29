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
# - Yugo Nishio

import math
import threading
import time

import game_communications.brain_communication as brain_comm
import gtpyhop
import rclpy
from rclpy.executors import MultiThreadedExecutor
import setting
import state
import utils
import numpy as np

from ball_prediction import NormalSamplingPredictWithParticleGausianFilter as NSPF
from filters import LPF
from decision_logger import DecisionLogger

from constants import (
    ACTION_RESULT,
    BALL_POSITION,
    GAME_STATE,
    GC_CONSTANTS,
    KICKOFF_STATE,
    OBJECT_LABEL,
    OUR_FIELD,
    ROBOT_FIELD,
    ROBOT_MODE,
    PENALTY
)
from print_utils import print_in_strategy, wrapp_notice
from setting import (
    CONVERT_SCALE,
    DEFAULT_STRATEGY_CONFIG,
    get_opponent_team_color,
    STRATEGY_SHARED_CONFIG
)
from state import Position, RoleChangeException
from utils import (
    calculate_distance,
    get_other_player_ballpos_from_shared_info,
    convert_local_to_global,
    get_real_action_name,
)
from strategy_time import get_strategy_time

from strategy.ros_publishers import WalkCommandPublisher
from strategy.ros_subscribers import StrategySubscriber


# 戦略を実際に実行するクラス。ここにはプランニングに直接関与するコードは実装しない。
# このクラスは、プランニングを実行し、そこから得たアクションを実行する。
# このクラスはpublisherとsubscriberも管理する。全体を統合して管理する役割を持つ。
class Strategy:
    def __init__(self, sim_robot_id=None):
        self.finish = False
        self.prev_strategy_time = get_strategy_time()
        # gtpyhop設定  -------------------------------
        self.domain_str = setting.DEFAULT_STRATEGY_DOMAIN
        gtpyhop.current_domain = gtpyhop.Domain(self.domain_str)
        gtpyhop.verbose = 0
        # import role
        import methods.attacker
        import methods.defender
        import methods.neutral
        import methods.player_entry
        import methods.search
        import methods.select_mode
        import real_action.actions
        import role

        self.current_domain = gtpyhop.current_domain
        self.current_state = state.default_init_state

        # robot_id設定 -------------------------------
        self.robot_id = setting.DEFAULT_STRATEGY_CONFIG["common_config"]["robot_id"]
        self.sim_robot_id = sim_robot_id  # シミュレーション用のロボットID。Noneの場合はsim_robot_idを使用しない。
        is_in_simulation = (sim_robot_id is not None)
        if sim_robot_id is not None and sim_robot_id != self.robot_id:
            raise ValueError(
                wrapp_notice("sim_robot_id must be the same as robot_id or None")
            )

        # ROS2 init -------------------------------
        rclpy.init(args=None)
        self.subscriber = StrategySubscriber(sim_robot_id=self.sim_robot_id)
        self.publisher = WalkCommandPublisher(sim_robot_id=self.sim_robot_id)
        self.logger = DecisionLogger()
        self.ros_executor = None  # Executorへの参照を保持
        self.sub_thread = threading.Thread(target=self.subscribe_function)
        self.sub_thread.daemon = True
        self.sub_thread.start()

        # self.default_todo_list = [("select_mode", self.publisher)]
        self.default_todo_list = [("select_mode", self.publisher, self.logger)]

        # commonly used setup
        self.brain_comm = brain_comm.BrainCommunication(is_in_simulation)
        self.brain_comm.init_udp_broadcast()  # ゲームコントローラーとの通信を初期化
        self.prev_infoshare_time = 0.0

        # load configuration
        self.load_config()  # 設定の読み込み
        print_in_strategy(f"Strategy initialized for robot {self.robot_id}")

        # ball_prediction setting
        self.nspf = NSPF(
            particle_num=STRATEGY_SHARED_CONFIG["filter_config"]["pf_config"]["particle_num"],
            num_seq=STRATEGY_SHARED_CONFIG["filter_config"]["pf_config"]["num_seq"],
            )

        self.lpf = LPF(STRATEGY_SHARED_CONFIG["filter_config"]["lpf_alpha"])

    def load_config(self):
        conf = setting.DEFAULT_STRATEGY_CONFIG
        team_color = conf["common_config"]["team_color"]
        if team_color == "blue":
            self.current_state.world_states["team_color"] = state.TEAM_COLOR.BLUE
        elif team_color == "red":
            self.current_state.world_states["team_color"] = state.TEAM_COLOR.RED
        else:
            raise ValueError(
                wrapp_notice(f"load_config():: Invalid team color: {team_color}")
            )

    def subscribe_function(self):
        print_in_strategy("subscribe_function thread started")
        try:
            self.ros_executor = MultiThreadedExecutor()
            self.ros_executor.add_node(self.subscriber)
            self.ros_executor.add_node(self.publisher)
            # Use spin_once with timeout instead of blocking spin
            while rclpy.ok() and not self.finish:
                self.ros_executor.spin_once(timeout_sec=0.1)
            print_in_strategy("[DEBUG] subscribe_function loop exited normally")
        except KeyboardInterrupt:
            print_in_strategy("Subscriber thread has been interrupted by keyboard.")
            self.finish = True
        except rclpy.executors.ExternalShutdownException:
            print_in_strategy("Subscriber thread has been shutdown externally.")
            self.finish = True
        except Exception as e:
            print_in_strategy(f"Subscriber thread error: {e}")
            self.finish = True
        finally:
            self.finish = True
            print_in_strategy("[DEBUG] subscribe_function finally block")
            if self.ros_executor:
                self.ros_executor.shutdown()

    def get_decision_log(self):
        return self.logger.get_decision_log()

    def reset_decision_log(self):
        self.logger.clear_decision_log()

    def send_decision_log(self):
        decision_log = self.get_decision_log()
        self.publisher.send_decision_log(decision_log)
        self.reset_decision_log()

    def find_plan(self, state) -> list | None:
        if self.finish:
            raise RuntimeError("Strategy has been finished.")
        # plan変数をtryブロックの外で初期化
        plan = None

        try:
            plan = gtpyhop.find_plan(state, self.default_todo_list)
            if plan:
                print_in_strategy(
                    f"[ Strategy ] Plan found for robot {self.robot_id}: "
                )
                for i, action in enumerate(plan):
                    print(f"---> {i}: {get_real_action_name(action)}")
            else:
                print(f"[ Strategy ] No plan found for robot {self.robot_id}")
                return None

        except RoleChangeException as e:
            print_in_strategy(f"Role change requested: {e.new_role}")
            self.current_state.role = e.new_role
            # ロール変更後、新しいプランを再生成し、planに再代入する
            plan = gtpyhop.find_plan(self.current_state, self.default_todo_list)
            if not plan:  # 再度プランが見つからない場合も考慮
                return None
            print_in_strategy(
                f"[ Strategy ] Plan found after role change for robot {self.robot_id}: "
            )
            for i, action in enumerate(plan):
                print(f"---> {i}: {get_real_action_name(action)}")
        except Exception as e:
            self.finish = True
            print_in_strategy(f" An error occurred: {e}")
            import traceback
            traceback.print_exc()
            raise RuntimeError("Strategy will be finished due to an error.")

        return plan

    def execute_action(self, action) -> ACTION_RESULT:
        # アクションを実行するロジックを実装
        # 成功した場合はTrue、失敗した場合はFalseを返す
        # 今はvirtual_actionが消えたので、恐らく利用しない
        from real_action.actions import REAL_ACTIONS_DICT

        real_action = REAL_ACTIONS_DICT[get_real_action_name(action)]
        print_in_strategy(f"Executing action: {real_action.__name__}")
        try:
            args = action[1:]
            if len(args) > 0:
                result = real_action(
                    self.current_state, *args
                )  # 引数ありの実アクションを実行
            else:
                result = real_action(
                    self.current_state,
                )  # 引数なしの実アクションを実行
        except RoleChangeException as e:
            print_in_strategy(f"Role change requested: {e.new_role}")
            self.current_state.role = e.new_role
            result = ACTION_RESULT.SUCCESS
        return result

    def finish_strategy(self):
        self.finish = True
        # 戦略の終了処理を実装
        print_in_strategy(f"Strategy finished for robot {self.robot_id}")

        # まずゲームコントローラーとの通信を終了
        self.brain_comm.clearup_game_controller_broadcast()

        # rclpyのシャットダウンを先に実行（これによりexecutorのspin()が停止する）
        print_in_strategy("Shutting down rclpy...")
        try:
            rclpy.shutdown()
        except Exception as e:
            print_in_strategy(f"rclpy.shutdown() failed: {e}")

        # スレッドの終了を待つ（タイムアウトを設定）
        if self.sub_thread and self.sub_thread.is_alive():
            print_in_strategy("Waiting for subscriber thread to finish...")
            self.sub_thread.join(timeout=3.0)
            if self.sub_thread.is_alive():
                print_in_strategy("Warning: Subscriber thread did not finish in time")

        # ノードを破棄（既に破棄されている可能性があるのでtry-catchで囲む）
        if self.subscriber:
            self.subscriber.destroy_node()

        if self.publisher:
            self.publisher.destroy_node()

    def reset_real_number_states(self):
        self.current_state.allies_position_local = []
        self.current_state.opponent_goalpost_local = []
        self.current_state.ally_goalpost_local = []
        self.current_state.opponent_goalpost_global = []  # グローバル座標もリセット
        self.current_state.opponent_position_local = []
        self.current_state.normal_obstacles_position_local = []
        self.current_state.ball_position_local = None
        # self.current_state.self_position_global = None
        # 👆そもそも自己位置はどんなに遅くても0.1秒くらい待てば次のが来るので、
        # もしも来なかったとしてもNoneにするよりも前の記憶をそのまま使う方が良い結果になるだろう。

    def get_world_state(self, last_action_name=""):
        # 状態のリセット
        self.reset_real_number_states()
        # 最後に実行したアクション名の保存
        self.current_state.last_action_name = last_action_name
        #  新しい受け取ったステートの取り出し
        sub_states = self.subscriber.get_latest_state()

        # オブジェクトの履歴の更新
        self.current_state.object_position_history = (
            self.subscriber.get_position_history()["detections"]
        )

        # 自己位置更新（confidence等確認する処理の追加の必要あり）
        selfpos = sub_states["selfpos"]
        if selfpos is not None:
            self.current_state.self_position_global = Position(
                Position.COORD_GLOBAL,
                selfpos.pose.x * CONVERT_SCALE,
                selfpos.pose.y * CONVERT_SCALE,
                selfpos.pose.theta,
            )

        detected_ball_for_debug = None
        # 各種オブジェクト情報の更新
        if sub_states["detections"] is not None:
            for detection in sub_states["detections"].detected_objects:
                detected_position = utils.convert_detectedobject_to_position(detection)
                if detection.label == OBJECT_LABEL.BALL:
                    self.current_state.ball_position_local = detected_position
                    detected_ball_for_debug = detected_position
                    if self.current_state.self_position_global is not None:
                        tmp_global_ball = convert_local_to_global(self.current_state.self_position_global, detected_position)
                        # フィールド外にあるボールは無視する
                        if utils.is_ball_in_outside_the_field(tmp_global_ball):
                            self.current_state.ball_position_local = None
                            continue
                        self.current_state.last_visible_ball = (
                                tmp_global_ball,
                                get_strategy_time())
                elif (detection.label == OBJECT_LABEL.OPPONENT) or (detection.label == OBJECT_LABEL.PERSON):
                    # チームカラーと同じなら味方
                    if detection.color == DEFAULT_STRATEGY_CONFIG["common_config"]["team_color"]:
                        self.current_state.allies_position_local.append(detected_position)
                    # Noneは単なる障害物
                    elif detection.color == "":
                        self.current_state.normal_obstacles_position_local.append(detected_position)
                    # 敵チームの色なら敵とする
                    elif detection.color == get_opponent_team_color():
                        self.current_state.opponent_position_local.append(detected_position)
                elif detection.label == OBJECT_LABEL.GOALPOST:
                    if (
                        self.current_state.self_position_global is not None
                    ):  # 自己位置が分かっている時
                        goalpos_gl = convert_local_to_global(
                            self.current_state.self_position_global, detected_position
                        )
                        our_field = self.current_state.world_states["our_field"]
                        is_our_goal = utils.is_object_in_our_field(
                            our_field, goalpos_gl
                        )
                        if is_our_goal:
                            self.current_state.ally_goalpost_local.append(
                                detected_position
                            )
                        else:  # 自分たちのゴールではない時
                            self.current_state.opponent_goalpost_local.append(
                                detected_position
                            )
                            self.current_state.opponent_goalpost_global.append(
                                goalpos_gl
                            )

        # 共有された情報の格納
        self.current_state.shared_msg = self.brain_comm.get_latest_shared_msg()
        tmp_ball_gl = utils.get_most_reliable_shared_ballpos(self.current_state.shared_msg)
        self.current_state.shared_ball_global = tmp_ball_gl

        # 今の観測と共有された情報の中で最も信頼できる情報を利用する
        # if tmp_ball_gl is not None and self.current_state.self_position_global is not None:
        #     self.current_state.ball_position_local = convert_global_to_local(
        #         self.current_state.self_position_global, tmp_ball_gl
        #     )

        # ボールが現在見えていない時
        # TODO :numpyじゃなく、Positon型を引数に取れるものを実装する
        if self.current_state.ball_position_local is None:
            predict , particle = self.nspf.predict_for_debug([np.nan,np.nan])
            if (
                np.std(particle) < DEFAULT_STRATEGY_CONFIG["common_config"]["predict_valid_std_range"] and 
                self.current_state.self_position_global is not None and
                predict is not None # Predictor側で見失いすぎるとNoneを返すようにした
            ):
                self.current_state.ball_position_local = utils.convert_global_to_local(
                    self.current_state.self_position_global,
                    state.Position(
                        coord=state.Position.COORD_GLOBAL,
                        x=predict[0],
                        y=predict[1]
                    )
                )
        else:
            if self.current_state.self_position_global is not None:
                current_ball = utils.convert_local_to_global(self.current_state.self_position_global,self.lpf.local_filter(self.current_state.ball_position_local))
                predict, _  = self.nspf.predict_for_debug(
                    np.array([current_ball.x,current_ball.y])
                )
                # predict = utils.convert_global_to_local(
                self.current_state.ball_position_local = utils.convert_global_to_local(
                    self.current_state.self_position_global,
                    state.Position(
                        coord=state.Position.COORD_GLOBAL,
                        x=predict[0],
                        y=predict[1]
                    )
                )
                print_in_strategy(f"PREDICT found ball:{current_ball},{predict}")
        # print_in_strategy(f"<DEBUG>Ball prediction time: {(time.perf_counter() - predict_start)*1000.0: .5f}ms")

        #  相手のゴールポストが検出されていない場合
        if (
            self.current_state.opponent_goalpost_local is None
            or len(self.current_state.opponent_goalpost_local) == 0
        ):
            last_visible_goalposts = utils.get_last_visible_goalpost(
                self.current_state.object_position_history
            )
            if (
                len(last_visible_goalposts) > 0
                and self.current_state.self_position_global is not None
            ):
                for post in last_visible_goalposts:
                    if not utils.is_object_in_our_field(
                        self.current_state.world_states["our_field"], post
                    ):
                        # print("DEBUG",self.current_state.self_position_global,post)
                        # グローバル座標から現在のロボット位置基準のローカル座標に変換
                        post_local = utils.convert_global_to_local(
                            self.current_state.self_position_global, post
                        )
                        self.current_state.opponent_goalpost_local.append(post_local)
                        self.current_state.opponent_goalpost_global.append(post)

        # 実数値でないステートの更新
        # ボールの位置の更新
        if (
            self.current_state.ball_position_local is None
        ):  # ボールが検出されていない場合
            self.current_state.world_states["ball_position_relative"] = (
                BALL_POSITION.UNKNOWN
            )
        else:
            # ボールの位置条件の更新 -----------------------------------
            if (
                calculate_distance(self.current_state.ball_position_local)
                < DEFAULT_STRATEGY_CONFIG["common_config"]["ball_near_range"]
            ):
                # ボールが近い、かつ角度が範囲内の場合はキック可能とする
                if (
                    self.current_state.ball_position_local.x > DEFAULT_STRATEGY_CONFIG["common_config"]["x_near_kick_range"]
                    and self.current_state.ball_position_local.x < DEFAULT_STRATEGY_CONFIG["common_config"]["x_far_kick_range"]
                    and self.current_state.ball_position_local.y < DEFAULT_STRATEGY_CONFIG["common_config"]["y_left_kick_range"]
                    and self.current_state.ball_position_local.y > DEFAULT_STRATEGY_CONFIG["common_config"]["y_right_kick_range"]
                ):  # ボールがキックレンジの範囲内の場合
                    self.current_state.world_states["ball_position_relative"] = (
                        BALL_POSITION.IN_KICK_RANGE
                    )
                else:
                    # ボールが近いがキックできない場合は、近いとする
                    self.current_state.world_states["ball_position_relative"] = (
                        BALL_POSITION.NEAR
                    )
            elif self.current_state.ball_position_local is not None:
                self.current_state.world_states["ball_position_relative"] = (
                    BALL_POSITION.DISTANT
                )
            else:
                self.current_state.world_states["ball_position_relative"] = (
                    BALL_POSITION.UNKNOWN
                )

        # ロボットがいるフィールドの更新 ------------------------------------------------
        if self.current_state.self_position_global is None:
            self.current_state.world_states["current_robot_field"] = ROBOT_FIELD.UNKNOWN
        else:
        #     # サイド検出
        #     if not self.side_detected and selfpos is not None:
        #         # まだサイドが検出されていない場合
        #         if (
        #             selfpos.convergence and selfpos.confidence > 0.7
        #         ):  # 収束し、かつ70%以上の信頼度がある時
        #             if OUR_FIELD.RIGHT(selfpos.pose.x):
        #                 self.current_state.world_states["our_field"] = OUR_FIELD.RIGHT
        #                 print_in_strategy(
        #                     "Our field detected!! Our field is < Right >."
        #                 )
        #             else:
        #                 self.current_state.world_states["our_field"] = OUR_FIELD.LEFT
        #                 print_in_strategy("Our field detected!! Our field is < Left >.")
        #             self.side_detected = True
            # 右側が相手のフィールドとする。
            if self.current_state.world_states["our_field"](
                self.current_state.self_position_global.x
            ):
                self.current_state.world_states["current_robot_field"] = (
                    ROBOT_FIELD.ALLY_FIELD
                )
            else:
                self.current_state.world_states["current_robot_field"] = (
                    ROBOT_FIELD.OPPONENT_FIELD
                )

        # ゲーコン関連の更新 -----------------------------------------------------
        gc_state = sub_states["game_controller"]
        if gc_state is not None:
            self.current_state.world_states["game_state"] = GC_CONSTANTS.STATE[
                gc_state.state
            ]
            penalty = GC_CONSTANTS.PENALTY[
                utils.retrieve_team_data_from_gc_teams(gc_state.teams)
                .players[DEFAULT_STRATEGY_CONFIG["common_config"]["robot_id"] - 1]
                .penalty
            ]
            # ペナルティ状態からNONEになった時にlast_penaltyを更新する。
            if (penalty == PENALTY.NONE) and (self.current_state.world_states["penalty"] != PENALTY.NONE):
                self.current_state.last_panelty = self.current_state.world_states["penalty"]
                self.current_state.last_unpenaized_time = get_strategy_time()
            self.current_state.world_states["penalty"] = penalty
            self.current_state.world_states["secondary_state"] = (
                GC_CONSTANTS.SECONDARY_STATE[gc_state.secondary_state]
            )
            print_in_strategy(
                wrapp_notice(str(self.current_state.world_states["penalty"]))
            )
            self.current_state.secsTillUnpenalised = (
                utils.retrieve_team_data_from_gc_teams(gc_state.teams)
                .players[DEFAULT_STRATEGY_CONFIG["common_config"]["robot_id"] - 1]
                .secs_till_unpenalised
            )
            self.current_state.kickOffTeam = gc_state.kick_off_team
            self.current_state.secondary_time = gc_state.secondary_time
            self.current_state.secs_remaining = gc_state.secs_remaining
            self.current_state.first_half = (gc_state.first_half == 1)
            self.current_state.secondary_state_info[0] = gc_state.secondary_state_info[0]
            self.current_state.secondary_state_info[1] = gc_state.secondary_state_info[1]
            self.current_state.secondary_state_info[2] = gc_state.secondary_state_info[2]
            self.current_state.secondary_state_info[3] = gc_state.secondary_state_info[3]

            if self.current_state.world_states["game_state"] == GAME_STATE.PLAYING:
                # キックオフチームの場合か、待機時間が終わるとキックオフ
                if (
                    self.current_state.kickOffTeam
                    == DEFAULT_STRATEGY_CONFIG["common_config"]["team_id"]
                ):
                    self.current_state.world_states["kickoff_state"] = (
                        KICKOFF_STATE.START
                    )
                elif self.current_state.secs_remaining <= 590:
                    self.current_state.world_states["kickoff_state"] = (
                        KICKOFF_STATE.START
                    )
                elif (
                    self.current_state.ball_position_local is not None
                    and self.current_state.self_position_global is not None
                ):
                    ball_pos_gl = utils.convert_local_to_global(
                        self.current_state.self_position_global,
                        self.current_state.ball_position_local,
                    )
                    if math.sqrt(ball_pos_gl.x**2 + ball_pos_gl.y**2) > 150.0:
                        self.current_state.world_states["kickoff_state"] = (
                            KICKOFF_STATE.START
                        )
            elif (
                self.current_state.world_states["kickoff_state"] != KICKOFF_STATE.START
            ):
                # ゲームがPLAYING状態でない場合、かつキックオフしていない時はキックオフ状態をWAITに設定
                self.current_state.world_states["kickoff_state"] = KICKOFF_STATE.WAIT
        # リモコンからの信号を確認 -----------------------------------------------------
        if sub_states["remote_controller"] is not None:
            remocon_state = sub_states["remote_controller"]
            # リモコン操作によるモード切替
            if remocon_state.x and remocon_state.rb:
                # モードをsoccer_gameに切り替え
                self.current_state.robot_mode = ROBOT_MODE.SOCCER_GAME
                print_in_strategy(
                    f"Robot {self.robot_id} switched to SOCCER_GAME mode due to button X pressed."
                )
            if remocon_state.b and remocon_state.rb:
                # NO_GCモードに切り替え
                self.current_state.robot_mode = ROBOT_MODE.NO_GC
                print_in_strategy(
                    f"Robot {self.robot_id} switched to NO_GC mode due to button B pressed."
                )
            if remocon_state.a and remocon_state.rb:
                # STOPモードに切り替え
                self.current_state.robot_mode = ROBOT_MODE.STOP
                print_in_strategy(
                    f"Robot {self.robot_id} switched to STOP mode due to button A pressed."
                )
            if remocon_state.y and remocon_state.rb and remocon_state.lb:
                selfpos_side = "left"
                if STRATEGY_SHARED_CONFIG["game_config"]["first_half_our_side"] == "right":
                    if self.current_state.first_half:
                        selfpos_side = "right"
                    else:
                        selfpos_side = "left"
                else:
                    if self.current_state.first_half:
                        selfpos_side = "left"
                    else:
                        selfpos_side = "right"
                self.publisher.set_entry_pose(selfpos_side)
                print_in_strategy(
                    f"Robot {self.robot_id} resetting our_field to {selfpos_side} side due to button Y+LB+RB pressed."
                )

        now = get_strategy_time()
        hz = 1.0 / (now - self.prev_strategy_time)
        self.current_state.last_strategy_process_time = (now - self.prev_strategy_time)
        print_in_strategy(f"[Hz info] {hz}hz")
        self.prev_strategy_time = now
        # デバッグ用に現在の状態をパブリッシュ
        self.publisher.send_debug_state(self.current_state,strategy_hz=hz,detected_ballpos=detected_ball_for_debug)

        # 情報共有の実行 -------------------------------------------------
        if (
            get_strategy_time() - self.prev_infoshare_time
            > brain_comm.BROADCAST_INFOSHARE_INTERVAL_S
        ):
            # 送信するボールの情報は記憶ではなく現在の観測
            if (
                self.current_state.self_position_global is not None
                and self.current_state.ball_position_local is not None
            ):
                ball_pos_gl = utils.convert_local_to_global(
                    self.current_state.self_position_global,
                    self.current_state.ball_position_local,
                )
            else:
                ball_pos_gl = None
            self.brain_comm.send_infoshare(
                ballpos=ball_pos_gl,
                selfpos=self.current_state.self_position_global,
                selfpos_local=self.current_state.ball_position_local,
                role=self.current_state.role,
                strategy_hz=hz
            )
            self.prev_infoshare_time = get_strategy_time()

        if sub_states["robot_low_state"] is not None:
            # ロボットの低レベル状態を更新
            self.current_state.robot_low_state = sub_states["robot_low_state"]

        # 首の動作モードの設定 -------------------------------------------------
        if self.current_state.ball_position_local is not None:
            self.publisher.set_neck_ball_tracking_mode(self.current_state.ball_position_local)
        elif len(get_other_player_ballpos_from_shared_info(self.current_state.shared_msg)) > 0:
            ballpos = utils.get_other_player_average_ballpos_from_shared_info(self.current_state.shared_msg)
            if self.current_state.self_position_global is not None:
                local_ballpos = utils.convert_global_to_local(self.current_state.self_position_global, ballpos)
                self.publisher.set_neck_ball_tracking_mode(local_ballpos)
            else:
                self.publisher.set_neck_search_ball_mode()
        else:
            self.publisher.set_neck_search_ball_mode()

        return self.current_state.copy()

    def check_standup(self):
        """
        ロボットの姿勢を確認し、起き上がり動作が必要なら行う。
        これは内部でsleepを伴うので、状態の更新前に行う。
        """
        # ストップモードの時は行わない
        if self.current_state.robot_mode != ROBOT_MODE.STOP:
            if self.current_state.robot_low_state is not None:
                if (
                    math.fabs(self.current_state.robot_low_state.roll) > 1.26
                    or math.fabs(self.current_state.robot_low_state.pitch) > 1.26
                ):
                    self.publisher.send_standup_command()
                    time.sleep(5.6)
