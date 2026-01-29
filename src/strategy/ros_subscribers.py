import math
import threading
import time

import constants
import rclpy
import setting
import utils
from booster_interface.msg import LowState, RemoteControllerState
from game_controller_interface.msg import GameControlData
from localization_msgs.msg import LocalizationResult
from rclpy.node import Node
from state import ObjectGlobalPositions, Position, RobotLowState
from vision_interface.msg import DetectedObject, Detections


class StrategySubscriber(Node):
    def __init__(self, sim_robot_id=None):
        super().__init__("strategy_subscriber")
        if sim_robot_id is not None:
            self.subscription_selfpos = self.create_subscription(
                LocalizationResult,
                "/self_position_" + str(sim_robot_id),
                self.selfpos_callback,
                1,
            )
            self.subscription_detection = self.create_subscription(
                Detections,
                "/booster_vision/detection_" + str(sim_robot_id),
                self.detection_callback,
                1,
            )
            self.subscription_low_state = self.create_subscription(
                LowState,
                "/low_state_" + str(sim_robot_id),
                self.low_state_callback,
                1,
            )
        else:
            self.subscription_selfpos = self.create_subscription(
                LocalizationResult, "/localization_result", self.selfpos_callback, 1
            )
            self.subscription_detection = self.create_subscription(
                Detections, "/booster_vision/detection", self.detection_callback, 1
            )
            self.subscription_low_state = self.create_subscription(
                LowState,
                "/low_state",
                self.low_state_callback,
                1,
            )
        self.subscription_game_control = self.create_subscription(
            GameControlData,
            "/robocup/game_controller",
            self.game_controller_callback,
            1,
        )
        self.subscription_controller = self.create_subscription(
            RemoteControllerState,
            "/remote_controller_state",
            self.remote_controller_callback,
            1,
        )

        self.subscription_selfpos  # prevent unused variable warning
        self.subscription_detection  # prevent unused variable warning
        self.state_mutex = threading.Lock()
        self.latest_selfpos = None
        self.latest_detections = None
        self.latest_game_controller_data = None
        self.latest_remote_controller_state = None
        self.latest_robot_low_state = None
        self.low_state_index = 0
        self.selfpos_history = []
        self.detections_history = []
        self.detections_index = 0
        self.start_time = time.clock_gettime(time.CLOCK_MONOTONIC)
        print(f"[StrategySubscriber] Initialized for robot_id={sim_robot_id}")

    def __del__(self):
        print("[StrategySubscriber] Finalized...")

    def selfpos_callback(self, msg):
        with self.state_mutex:
            self.latest_selfpos = msg
            self.selfpos_history.append(msg)
            if len(self.selfpos_history) > setting.POSITION_LOCAL_HISTORY_LENGTH:
                self.selfpos_history.pop(0)

    def _convert_all_global(
        self, selfpos: Position, detections: Detections
    ) -> ObjectGlobalPositions:
        balls = utils.take_target_out_from_detected_objects(
            constants.OBJECT_LABEL.BALL, detections
        )
        goalposts = utils.take_target_out_from_detected_objects(
            constants.OBJECT_LABEL.GOALPOST, detections
        )
        result = ObjectGlobalPositions()
        for ball in balls:
            ball_pos_global = utils.convert_local_to_global(
                selfpos, utils.convert_detectedobject_to_position(ball)
            )
            result.ball_pos_global.append(ball_pos_global)
        for goalpost in goalposts:
            goalpost_pos_global = utils.convert_local_to_global(
                selfpos, utils.convert_detectedobject_to_position(goalpost)
            )
            result.goalpost_pos_global.append(goalpost_pos_global)
        return result

    def detection_callback(self, msg):
        self.detections_index += 1
        now = time.clock_gettime(time.CLOCK_MONOTONIC)
        hz = self.detections_index / (now - self.start_time)
        history_length = math.ceil(hz * setting.POSITION_LOCAL_HISTORY_LENGTH_S)
        if len(self.detections_history) > history_length:
            self.detections_history.pop(0)

        with self.state_mutex:
            self.latest_detections = msg
            if self.latest_selfpos is None:
                self.detections_history.append(ObjectGlobalPositions())
                return
            selfpos_position = Position(
                Position.COORD_GLOBAL,
                self.latest_selfpos.pose.x * setting.CONVERT_SCALE,
                self.latest_selfpos.pose.y * setting.CONVERT_SCALE,
                self.latest_selfpos.pose.theta,
            )
            detections_global = self._convert_all_global(
                selfpos_position, self.latest_detections
            )
            self.detections_history.append(detections_global)

    def game_controller_callback(self, msg):
        with self.state_mutex:
            self.latest_game_controller_data = msg

    def remote_controller_callback(self, msg):
        with self.state_mutex:
            self.latest_remote_controller_state = msg

    def get_latest_state(self):
        with self.state_mutex:
            # Return the latest state information
            ret = {
                "selfpos": self.latest_selfpos,
                "detections": self.latest_detections,
                "game_controller": self.latest_game_controller_data,
                "remote_controller": self.latest_remote_controller_state,
                "robot_low_state": self.latest_robot_low_state,
            }
            self.latest_selfpos = None
            self.latest_detections = None
            self.latest_game_controller_data = None
            self.latest_remote_controller_state = None
            return ret

    def get_position_history(self):
        with self.state_mutex:
            return {
                "selfpos": self.selfpos_history,
                "detections": self.detections_history,
            }

    def low_state_callback(self, msg):
        self.low_state_index += 1
        if self.low_state_index % 25 == 0:
            with self.state_mutex:
                self.latest_robot_low_state = RobotLowState(
                    neck_angle=msg.motor_state_serial[0].q,  # 首のyaw角度
                    neck_pitch_angle=msg.motor_state_serial[1].q,  # 首のyaw角度
                    roll=msg.imu_state.rpy[0],
                    pitch=msg.imu_state.rpy[1],
                    yaw=msg.imu_state.rpy[2],
                )
