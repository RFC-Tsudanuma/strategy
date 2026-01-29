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


"""
GameControlData GUIビューアー
/robocup/game_controllerトピックのGameControlDataメッセージを可視化
"""

import sys

import rclpy
from game_controller_interface.msg import GameControlData
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QApplication,
    QButtonGroup,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QRadioButton,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node

from constants import GC_CONSTANTS


class GameControlDataGUI(Node):
    def __init__(self, main_window):
        super().__init__("game_control_data_gui")
        self.main_window = main_window

        # メッセージ保持用
        self.last_msg = None

        # 選択状態
        self.selected_team = 0
        self.selected_robot = 0

        # GUI構築
        self._create_widgets()

        # サブスクライバー作成
        self.subscription = self.create_subscription(
            GameControlData, "/robocup/game_controller", self.game_control_callback, 10
        )

        # ROS2のスピン処理用タイマー
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_once)
        self.timer.start(10)  # 10ms毎にスピン

        self.get_logger().info("GameControlData GUI started")

    def spin_once(self):
        """ROS2のスピン処理"""
        rclpy.spin_once(self, timeout_sec=0.0)

    def _create_widgets(self):
        """GUIウィジェットを作成"""
        # メインウィジェット
        central_widget = QWidget()
        self.main_window.setCentralWidget(central_widget)

        # メインレイアウト
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)

        # タイトル
        title = QLabel("GameControlData Viewer")
        title_font = QFont("Arial", 16)
        title_font.setBold(True)
        title.setFont(title_font)
        main_layout.addWidget(title)

        # ゲーム状態セクション
        game_group = QGroupBox("Game State")
        game_layout = QGridLayout()
        game_group.setLayout(game_layout)

        self._add_label_value(game_layout, 0, "State:", "state_label")
        self._add_label_value(game_layout, 1, "Secondary State:", "secondary_state_label")
        self._add_label_value(game_layout, 2, "First Half:", "first_half_label")
        self._add_label_value(game_layout, 3, "Kick Off Team:", "kick_off_team_label")
        self._add_label_value(game_layout, 4, "Secs Remaining:", "secs_remaining_label")
        self._add_label_value(game_layout, 5, "Secondary Time:", "secondary_time_label")

        main_layout.addWidget(game_group)

        # Secondary State Infoセクション
        secondary_info_group = QGroupBox("Secondary State Info")
        secondary_info_layout = QGridLayout()
        secondary_info_group.setLayout(secondary_info_layout)

        self._add_label_value(secondary_info_layout, 0, "Info:", "secondary_state_info_label")

        main_layout.addWidget(secondary_info_group)

        # チーム情報セクション
        teams_group = QGroupBox("Teams")
        teams_layout = QGridLayout()
        teams_group.setLayout(teams_layout)

        # Team 0
        team0_group = QGroupBox("Team 0")
        team0_layout = QGridLayout()
        team0_group.setLayout(team0_layout)
        self._add_label_value(team0_layout, 0, "Team Number:", "team0_number_label")
        self._add_label_value(team0_layout, 1, "Team Colour:", "team0_colour_label")
        self._add_label_value(team0_layout, 2, "Score:", "team0_score_label")
        teams_layout.addWidget(team0_group, 0, 0)

        # Team 1
        team1_group = QGroupBox("Team 1")
        team1_layout = QGridLayout()
        team1_group.setLayout(team1_layout)
        self._add_label_value(team1_layout, 0, "Team Number:", "team1_number_label")
        self._add_label_value(team1_layout, 1, "Team Colour:", "team1_colour_label")
        self._add_label_value(team1_layout, 2, "Score:", "team1_score_label")
        teams_layout.addWidget(team1_group, 0, 1)

        main_layout.addWidget(teams_group)

        # ロボット情報セクション
        robots_group = QGroupBox("Robot Info")
        robots_layout = QVBoxLayout()
        robots_group.setLayout(robots_layout)

        # チーム選択
        team_select_layout = QHBoxLayout()
        team_select_label = QLabel("Select Team:")
        team_select_layout.addWidget(team_select_label)

        self.team_button_group = QButtonGroup()
        self.team0_radio = QRadioButton("Team 0")
        self.team1_radio = QRadioButton("Team 1")
        self.team0_radio.setChecked(True)
        self.team_button_group.addButton(self.team0_radio, 0)
        self.team_button_group.addButton(self.team1_radio, 1)
        self.team_button_group.idClicked.connect(self.on_team_selected)

        team_select_layout.addWidget(self.team0_radio)
        team_select_layout.addWidget(self.team1_radio)
        team_select_layout.addStretch()
        robots_layout.addLayout(team_select_layout)

        # ロボットID選択
        robot_select_layout = QHBoxLayout()
        robot_select_label = QLabel("Select Robot ID:")
        robot_select_layout.addWidget(robot_select_label)

        self.robot_button_group = QButtonGroup()
        self.robot_buttons = []
        for i in range(5):  # 5台のロボット
            btn = QPushButton(str(i + 1))
            btn.setCheckable(True)
            btn.setMaximumWidth(50)
            btn.setMinimumHeight(35)
            self.robot_button_group.addButton(btn, i)
            self.robot_buttons.append(btn)
            robot_select_layout.addWidget(btn)

        self.robot_buttons[0].setChecked(True)
        self.robot_button_group.idClicked.connect(self.on_robot_selected)
        robot_select_layout.addStretch()
        robots_layout.addLayout(robot_select_layout)

        # ロボット詳細情報（ラベル形式）
        robot_info_grid = QGridLayout()

        self._add_label_value(robot_info_grid, 0, "Penalty:", "robot_penalty_label")
        self._add_label_value(robot_info_grid, 1, "Secs till unpenalised:", "robot_secs_label")
        self._add_label_value(robot_info_grid, 2, "Warnings:", "robot_warnings_label")
        self._add_label_value(robot_info_grid, 3, "Yellow Cards:", "robot_yellow_label")
        self._add_label_value(robot_info_grid, 4, "Red Cards:", "robot_red_label")
        self._add_label_value(robot_info_grid, 5, "Goal Keeper:", "robot_goalkeeper_label")

        robots_layout.addLayout(robot_info_grid)

        main_layout.addWidget(robots_group)

    def _add_label_value(self, layout, row, label_text, var_name):
        """ラベルと値のペアを追加"""
        label = QLabel(label_text)
        label.setFont(QFont("Arial", 10))
        layout.addWidget(label, row, 0)

        value = QLabel("N/A")
        value_font = QFont("Arial", 10)
        value_font.setBold(True)
        value.setFont(value_font)
        layout.addWidget(value, row, 1)

        setattr(self, var_name, value)

    def on_team_selected(self, team_id):
        """チーム選択時のコールバック"""
        self.selected_team = team_id
        self._update_robot_info()

    def on_robot_selected(self, robot_id):
        """ロボットID選択時のコールバック"""
        self.selected_robot = robot_id
        self._update_robot_info()

    def game_control_callback(self, msg):
        """GameControlDataメッセージのコールバック"""
        self.last_msg = msg
        self.update_gui()

    def update_gui(self):
        """GUIを更新"""
        if self.last_msg is None:
            return

        msg = self.last_msg

        # 状態を文字列に変換
        state_str = self._convert_state(msg.state)
        secondary_state_str = self._convert_secondary_state(msg.secondary_state)

        # ゲーム状態を更新
        self.state_label.setText(state_str)
        self.secondary_state_label.setText(secondary_state_str)
        self.first_half_label.setText("First Half" if msg.first_half else "Second Half")
        self.kick_off_team_label.setText(str(msg.kick_off_team))
        self.secs_remaining_label.setText(f"{msg.secs_remaining} sec")
        self.secondary_time_label.setText(f"{msg.secondary_time} sec")

        # Secondary State Infoを更新
        # char[4]を文字列に変換（nullバイトまで読む）
        info_str = ""
        for c in msg.secondary_state_info:
            info_str += str(int(c))

            info_str += ","
        # info_str = bytes(msg.secondary_state_info).decode("utf-8", errors="ignore").rstrip("\x00")
        self.secondary_state_info_label.setText(info_str)

        # チーム情報を更新
        if len(msg.teams) >= 1:
            self.team0_number_label.setText(str(msg.teams[0].team_number))
            self.team0_colour_label.setText(self._get_team_colour_str(msg.teams[0].team_colour))
            self.team0_score_label.setText(str(msg.teams[0].score))

        if len(msg.teams) >= 2:
            self.team1_number_label.setText(str(msg.teams[1].team_number))
            self.team1_colour_label.setText(self._get_team_colour_str(msg.teams[1].team_colour))
            self.team1_score_label.setText(str(msg.teams[1].score))

        # ロボット情報を更新
        self._update_robot_info()

    def _convert_state(self, state_value):
        """stateの数値をGC_CONSTANTSを使って文字列に変換"""
        if state_value in GC_CONSTANTS.STATE:
            return GC_CONSTANTS.STATE[state_value].value
        return f"Unknown ({state_value})"

    def _convert_secondary_state(self, secondary_state_value):
        """secondary_stateの数値をGC_CONSTANTSを使って文字列に変換"""
        if secondary_state_value in GC_CONSTANTS.SECONDARY_STATE:
            return GC_CONSTANTS.SECONDARY_STATE[secondary_state_value].value
        return f"Unknown ({secondary_state_value})"

    def _convert_penalty(self, penalty_value):
        """penaltyの数値をGC_CONSTANTSを使って文字列に変換"""
        if penalty_value in GC_CONSTANTS.PENALTY:
            return GC_CONSTANTS.PENALTY[penalty_value].value
        return f"Unknown ({penalty_value})"

    def _get_team_colour_str(self, colour_value):
        """チームカラーを文字列に変換"""
        colour_map = {
            0: "Blue",
            1: "Red",
            2: "Yellow",
            3: "Black",
            4: "White",
            5: "Green",
            6: "Orange",
            7: "Purple",
            8: "Brown",
            9: "Gray",
        }
        return colour_map.get(colour_value, f"Unknown ({colour_value})")

    def _update_robot_info(self):
        """選択されたロボットの情報を更新"""
        if self.last_msg is None:
            self.robot_penalty_label.setText("N/A")
            self.robot_secs_label.setText("N/A")
            self.robot_warnings_label.setText("N/A")
            self.robot_yellow_label.setText("N/A")
            self.robot_red_label.setText("N/A")
            self.robot_goalkeeper_label.setText("N/A")
            return

        # チームが存在するか確認
        if self.selected_team >= len(self.last_msg.teams):
            self.robot_penalty_label.setText("No team data")
            self.robot_secs_label.setText("-")
            self.robot_warnings_label.setText("-")
            self.robot_yellow_label.setText("-")
            self.robot_red_label.setText("-")
            self.robot_goalkeeper_label.setText("-")
            return

        team = self.last_msg.teams[self.selected_team]

        # ロボットが存在するか確認
        if self.selected_robot >= len(team.players):
            self.robot_penalty_label.setText("No robot data")
            self.robot_secs_label.setText("-")
            self.robot_warnings_label.setText("-")
            self.robot_yellow_label.setText("-")
            self.robot_red_label.setText("-")
            self.robot_goalkeeper_label.setText("-")
            return

        player = team.players[self.selected_robot]
        penalty_str = self._convert_penalty(player.penalty)

        # 選択されたロボットの詳細情報を表示
        self.robot_penalty_label.setText(penalty_str)
        self.robot_secs_label.setText(str(player.secs_till_unpenalised))
        self.robot_warnings_label.setText(str(player.number_of_warnings))
        self.robot_yellow_label.setText(str(player.yellow_card_count))
        self.robot_red_label.setText(str(player.red_card_count))
        self.robot_goalkeeper_label.setText("Yes" if player.goal_keeper else "No")


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)

    main_window = QMainWindow()
    main_window.setWindowTitle("GameControlData Viewer")
    main_window.resize(800, 650)

    node = GameControlDataGUI(main_window)

    main_window.show()

    try:
        app.exec()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
