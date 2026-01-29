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



# 1. チーム間の通信を受信して、それをパースする
# パースする方法。game_communication.teammate_communication.pyに実装されている内容を使う
# 2. パースしたデータを表示する。
# 表示方法
# - 送信元のロボット毎に一覧表示とする
#   - この一覧で表示するのは、ロボットのロールのみ
# - マップのようなものを描画して、ロボットの自己位置と認識しているボールの位置を一覧表示する
# ---------------------------------------------------------------
# | ロボット1|
# |  ロール  |
# |---------|
# | ロボット2|
# |  ロール  |
# |---------|           フィールド
# | ロボット3|
# |  ロール  |
# |---------|
# | ロボット4|
# |  ロール  |
# |---------|
# ---------------------------------------------------------------

import socket
from typing import Optional
import threading
import math

# Try importing PyQt6 first, fall back to PyQt5 if not available
try:
    from PyQt6.QtWidgets import QWidget, QMainWindow, QVBoxLayout, QHBoxLayout, QLabel, QApplication
    from PyQt6.QtCore import Qt, QPointF, QTimer
    from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QPolygonF, QFont
    PYQT_VERSION = 6
except ImportError:
    from PyQt5.QtWidgets import QWidget, QMainWindow, QVBoxLayout, QHBoxLayout, QLabel, QApplication
    from PyQt5.QtCore import Qt, QPointF, QTimer
    from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QPolygonF, QFont
    PYQT_VERSION = 5

from game_communications.teammate_communication import convert_pb_position_to_position, convert_pb_position_to_position_local, convert_pb_role_to_role, BROADCAST_PORT, VALIDATION_COMMUNICATION
from state import ROLE, Position, SharedMessage

import game_communications.teammate_communication_pb2 as pb


# フィールド設定（strategy_sim/scripts/setting.pyから）
# フィールドサイズ（メートル）
FIELD_LENGTH = 14.0  # フィールドの長さ（メートル）
FIELD_WIDTH_M = 9.0  # フィールドの幅（メートル）

# スケール設定
PIXELS_PER_METER = 30  # 1メートル = 30ピクセル（縮小版）

# 表示領域のマージン（フィールド外）
MARGIN_METERS = 2.0  # フィールド外2mまで表示
MARGIN_PIXELS = int(MARGIN_METERS * PIXELS_PER_METER)

# ピクセル単位の総サイズ（フィールド + マージン）
DISPLAY_WIDTH = int((FIELD_LENGTH + 2 * MARGIN_METERS) * PIXELS_PER_METER)
DISPLAY_HEIGHT = int((FIELD_WIDTH_M + 2 * MARGIN_METERS) * PIXELS_PER_METER)

# フィールド開始位置（マージン分ずれている）
FIELD_START_X = MARGIN_PIXELS
FIELD_START_Y = MARGIN_PIXELS
FIELD_PIXEL_WIDTH = int(FIELD_LENGTH * PIXELS_PER_METER)
FIELD_PIXEL_HEIGHT = int(FIELD_WIDTH_M * PIXELS_PER_METER)

FIELD_COLOR = QColor(0, 150, 0)
LINE_COLOR = QColor(255, 255, 255)
GOAL_COLOR = QColor(255, 0, 0)
ROBOT_SIZE = 20  # 縮小版なので小さめ

# 各ロボットとそのボールの色（ロボットごとに区別）
ROBOT_COLORS = [
    QColor(0, 100, 255),    # Robot 1: 青
    QColor(255, 165, 0),    # Robot 2: オレンジ
    QColor(255, 100, 255),  # Robot 3: マゼンタ
]


class FieldWidget(QWidget):
    """
    フィールドを描画するウィジェット
    strategy_simのfield.py、robot.py、ball.pyの描画ロジックをPyQtに移植
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(DISPLAY_WIDTH, DISPLAY_HEIGHT)
        self.messages = []  # SharedMessageのリスト

    def update_messages(self, messages: list[Optional[SharedMessage]]):
        """受信したメッセージを更新"""
        self.messages = messages
        self.update()  # 再描画をトリガー

    def paintEvent(self, event):
        """描画処理"""
        painter = QPainter(self)
        if PYQT_VERSION == 6:
            painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        else:
            painter.setRenderHint(QPainter.Antialiasing)

        # フィールドを描画
        self._draw_field(painter)

        # ロボットとボールを描画
        self._draw_robots_and_balls(painter)

    def _draw_field(self, painter: QPainter):
        """フィールドの線を描画（strategy_sim/scripts/field.pyから移植）"""
        # 背景色（全体）
        painter.fillRect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, QColor(100, 100, 100))

        # フィールド背景色
        painter.fillRect(
            FIELD_START_X, FIELD_START_Y,
            FIELD_PIXEL_WIDTH, FIELD_PIXEL_HEIGHT,
            FIELD_COLOR
        )

        # ペンの設定
        pen = QPen(LINE_COLOR, 2)
        painter.setPen(pen)

        # フィールド外枠
        painter.drawRect(
            FIELD_START_X, FIELD_START_Y,
            FIELD_PIXEL_WIDTH, FIELD_PIXEL_HEIGHT
        )

        # センターライン
        center_x = FIELD_START_X + FIELD_PIXEL_WIDTH // 2
        painter.drawLine(
            int(center_x), FIELD_START_Y,
            int(center_x), FIELD_START_Y + FIELD_PIXEL_HEIGHT
        )

        # センターサークル（半径1.5m）
        center_circle_radius = int(1.5 * PIXELS_PER_METER)
        center_y = FIELD_START_Y + FIELD_PIXEL_HEIGHT // 2
        painter.drawEllipse(
            int(center_x - center_circle_radius),
            int(center_y - center_circle_radius),
            center_circle_radius * 2,
            center_circle_radius * 2
        )

        # ペナルティエリア（3m x 6m）
        penalty_width = int(3.0 * PIXELS_PER_METER)
        penalty_height = int(6.0 * PIXELS_PER_METER)
        penalty_y = FIELD_START_Y + (FIELD_PIXEL_HEIGHT - penalty_height) // 2

        # 左側のペナルティエリア
        painter.drawRect(
            FIELD_START_X,
            int(penalty_y),
            penalty_width,
            penalty_height
        )

        # 右側のペナルティエリア
        painter.drawRect(
            FIELD_START_X + FIELD_PIXEL_WIDTH - penalty_width,
            int(penalty_y),
            penalty_width,
            penalty_height
        )

        # ゴールエリア（1m x 4m）
        goal_area_width = int(1.0 * PIXELS_PER_METER)
        goal_area_height = int(4.0 * PIXELS_PER_METER)
        goal_area_y = FIELD_START_Y + (FIELD_PIXEL_HEIGHT - goal_area_height) // 2

        # 左側のゴールエリア
        painter.drawRect(
            FIELD_START_X,
            int(goal_area_y),
            goal_area_width,
            goal_area_height
        )

        # 右側のゴールエリア
        painter.drawRect(
            FIELD_START_X + FIELD_PIXEL_WIDTH - goal_area_width,
            int(goal_area_y),
            goal_area_width,
            goal_area_height
        )

        # ペナルティスポット（2.1m）
        penalty_spot_distance = int(2.1 * PIXELS_PER_METER)
        penalty_spot_radius = 3
        painter.setBrush(QBrush(LINE_COLOR))
        # 左側
        painter.drawEllipse(
            FIELD_START_X + penalty_spot_distance - penalty_spot_radius,
            int(center_y - penalty_spot_radius),
            penalty_spot_radius * 2,
            penalty_spot_radius * 2
        )
        # 右側
        painter.drawEllipse(
            FIELD_START_X + FIELD_PIXEL_WIDTH - penalty_spot_distance - penalty_spot_radius,
            int(center_y - penalty_spot_radius),
            penalty_spot_radius * 2,
            penalty_spot_radius * 2
        )
        painter.setBrush(QBrush())

        # ゴールの描画
        goal_height = int(2.6 * PIXELS_PER_METER)  # 2.6m
        goal_depth = 5
        goal_pen = QPen(GOAL_COLOR, 3)
        painter.setPen(goal_pen)
        painter.setBrush(QBrush(GOAL_COLOR))

        goal_y = int(center_y - goal_height // 2)

        # 左ゴール
        painter.fillRect(
            FIELD_START_X - goal_depth, goal_y,
            goal_depth, goal_height,
            GOAL_COLOR
        )

        # 右ゴール
        painter.fillRect(
            FIELD_START_X + FIELD_PIXEL_WIDTH, goal_y,
            goal_depth, goal_height,
            GOAL_COLOR
        )

    def _draw_robots_and_balls(self, painter: QPainter):
        """ロボットとボールを描画"""
        # まず各ロボットが認識しているボールを描画（背景）
        for i, msg in enumerate(self.messages):
            if msg is None:
                continue
            # 各ロボットが認識しているボールの描画（ロボットごとに色を変える）
            self._draw_ball(painter, msg, i)

        # 次にロボットを描画（前景）
        for msg in self.messages:
            if msg is None:
                continue
            # ロボットの描画
            self._draw_robot(painter, msg)

    def _draw_robot(self, painter: QPainter, msg: SharedMessage):
        """ロボットを描画（strategy_sim/scripts/robot.pyから移植）"""
        # selfposがNoneの場合は描画しない
        if msg.selfpos is None:
            return

        # 実機座標系（cm単位、中心原点）をメートルに変換
        x_m = msg.selfpos.x / 100.0  # cm から m に変換
        y_m = msg.selfpos.y / 100.0  # cm から m に変換
        angle = msg.selfpos.theta

        # メートル座標系からピクセル座標系に変換
        # フィールド中心は表示領域の中心
        x_qt = FIELD_START_X + (x_m + FIELD_LENGTH / 2) * PIXELS_PER_METER
        y_qt = FIELD_START_Y + (FIELD_WIDTH_M / 2 - y_m) * PIXELS_PER_METER  # Y軸反転

        # 四角形の頂点を計算（中心が原点の場合）
        half_size = ROBOT_SIZE // 2
        corners = [
            (-half_size, -half_size),
            (half_size, -half_size),
            (half_size, half_size),
            (-half_size, half_size),
        ]

        # 回転行列を適用して実際の位置に変換
        polygon_points = []
        for corner_x, corner_y in corners:
            # 回転
            rot_x = corner_x * math.cos(-angle) - corner_y * math.sin(-angle)
            rot_y = corner_x * math.sin(-angle) + corner_y * math.cos(-angle)
            # 平行移動
            polygon_points.append(QPointF(x_qt + rot_x, y_qt + rot_y))

        polygon = QPolygonF(polygon_points)

        # ロボットIDに基づいて色を選択
        robot_color = ROBOT_COLORS[(msg.robot_id - 1) % len(ROBOT_COLORS)]

        # ロボットを描画
        painter.setBrush(QBrush(robot_color))
        painter.setPen(QPen(LINE_COLOR, 2))
        painter.drawPolygon(polygon)

        # ロボットの向きを示す線を描画
        direction_length = ROBOT_SIZE // 2 + 5
        end_x = x_qt + direction_length * math.cos(-angle)
        end_y = y_qt + direction_length * math.sin(-angle)
        painter.setPen(QPen(LINE_COLOR, 2))
        painter.drawLine(int(x_qt), int(y_qt), int(end_x), int(end_y))

        # ロボット番号を描画
        white_color = Qt.GlobalColor.white if PYQT_VERSION == 6 else Qt.white
        painter.setPen(QPen(white_color))
        painter.setFont(QFont("Arial", 8))
        painter.drawText(
            int(x_qt - 5), int(y_qt + 10),
            str(msg.robot_id)
        )

        # ロールを描画
        role_text = str(msg.role)
        painter.setFont(QFont("Arial", 7))
        painter.drawText(
            int(x_qt - 10), int(y_qt - 15),
            role_text
        )

    def _draw_ball(self, painter: QPainter, msg: SharedMessage, robot_index: int):
        """ボールを描画（strategy_sim/scripts/ball.pyから移植）"""
        # ballposがNoneの場合は描画しない
        if msg.ballpos is None:
            return

        # 実機座標系（cm単位、中心原点）をメートルに変換
        x_m = msg.ballpos.x / 100.0  # cm から m に変換
        y_m = msg.ballpos.y / 100.0  # cm から m に変換

        # メートル座標系からピクセル座標系に変換
        x_qt = FIELD_START_X + (x_m + FIELD_LENGTH / 2) * PIXELS_PER_METER
        y_qt = FIELD_START_Y + (FIELD_WIDTH_M / 2 - y_m) * PIXELS_PER_METER  # Y軸反転

        # ロボットIDに基づいて色を選択（ロボットと同じ色）
        ball_color = ROBOT_COLORS[(msg.robot_id - 1) % len(ROBOT_COLORS)]

        # ボールを描画（半透明）
        ball_radius = 6
        transparent_color = QColor(ball_color)
        transparent_color.setAlpha(180)  # 半透明にして重なりを見やすく
        painter.setBrush(QBrush(transparent_color))
        painter.setPen(QPen(ball_color, 1))
        painter.drawEllipse(
            int(x_qt - ball_radius),
            int(y_qt - ball_radius),
            ball_radius * 2,
            ball_radius * 2
        )

        # ロボット番号をボールの横に小さく表示
        white_color = Qt.GlobalColor.white if PYQT_VERSION == 6 else Qt.white
        painter.setPen(QPen(white_color))
        painter.setFont(QFont("Arial", 6))
        painter.drawText(
            int(x_qt + ball_radius + 1),
            int(y_qt - ball_radius),
            f"R{msg.robot_id}"
        )


class MessageReceiver:
    def __init__(self):
        # Sockets
        self._communication_recv_socket: Optional[socket.socket] = None

        # Threads
        self._communication_recv_thread = None

        # Flags
        self._receive_communication_flag = False

        self.latest_msg_lock = threading.Lock()
        self.latest_msg = [None, None, None]  # 3人なので
        self.init_communication_receiver()

    def __del__(self):
        self.cleanup_communication_receiver()

    def init_communication_receiver(self):
        try:
            self._communication_recv_socket = socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM
            )

            # Allow address reuse
            self._communication_recv_socket.setsockopt(
                socket.SOL_SOCKET, socket.SO_REUSEADDR, 1
            )

            # Set socket timeout to allow periodic flag checking
            self._communication_recv_socket.settimeout(0.5)

            # Bind to port
            addr = ("", BROADCAST_PORT)
            self._communication_recv_socket.bind(addr)

            print(f"\033[32mListening for broadcasts on port {BROADCAST_PORT}\033[0m")

            self._receive_communication_flag = True
            self._communication_recv_thread = threading.Thread(
                target=self.spin_communication_receiver
            )
            self._communication_recv_thread.daemon = True  # デーモンスレッドに設定
            self._communication_recv_thread.start()

        except Exception as e:
            print(f"\033[31mFailed to init communication receiver: {e}\033[0m")

    def cleanup_communication_receiver(self):
        self._receive_communication_flag = False
        if self._communication_recv_socket:
            try:
                self._communication_recv_socket.close()
            except Exception as e:
                print(e)
                pass
            self._communication_recv_socket = None
            print("\033[31mCommunication receive socket has been closed.\033[0m")

        if (
            self._communication_recv_thread
            and self._communication_recv_thread.is_alive()
        ):
            self._communication_recv_thread.join(timeout=2.0)  # 最大2秒待つ

    def spin_communication_receiver(self):
        while self._receive_communication_flag:
            try:
                data, addr = self._communication_recv_socket.recvfrom(1024)
                # Parse protobuf message
                msg = pb.TeamCommunicationMsg()
                try:
                    msg.ParseFromString(data)
                except Exception:
                    continue

                if msg.validation != VALIDATION_COMMUNICATION:
                    continue

                with self.latest_msg_lock:
                    self.latest_msg[msg.robot_id - 1] = msg

            except socket.timeout:
                # タイムアウトは正常、フラグをチェックしてループを続ける
                continue
            except Exception as e:
                if self._receive_communication_flag:
                    print(f"\033[31mReceiving broadcast message failed: {e}\033[0m")

    def get_latest_msg(self) -> list[Optional[SharedMessage]]:
        with self.latest_msg_lock:
            result = []
            for msg in self.latest_msg:
                if msg is None:
                    result.append(None)
                else:
                    selfpos = convert_pb_position_to_position(msg.selfpos)
                    ballpos = convert_pb_position_to_position(msg.ballpos)
                    ballpos_local = convert_pb_position_to_position_local(msg.ballpos_local)
                    role = convert_pb_role_to_role(msg.role)
                    id = msg.robot_id
                    tmp = SharedMessage(
                        selfpos=selfpos,
                        ballpos=ballpos,
                        role=role,
                        robot_id=id,
                        ballpos_local=ballpos_local,
                    )
                    result.append(tmp)
            return result


class RobotInfoPanel(QWidget):
    """ロボット情報を表示するサイドパネル"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.robot_labels = []

        # タイトル
        title = QLabel("Robot Information")
        font_weight = QFont.Weight.Bold if PYQT_VERSION == 6 else QFont.Bold
        align_center = Qt.AlignmentFlag.AlignCenter if PYQT_VERSION == 6 else Qt.AlignCenter
        title.setFont(QFont("Arial", 14, font_weight))
        title.setAlignment(align_center)
        self.layout.addWidget(title)

        # 凡例を追加
        legend_widget = QWidget()
        legend_layout = QVBoxLayout(legend_widget)
        legend_layout.setContentsMargins(10, 5, 10, 5)

        legend_title = QLabel("Robot/Ball Color:")
        font_weight = QFont.Weight.Bold if PYQT_VERSION == 6 else QFont.Bold
        legend_title.setFont(QFont("Arial", 10, font_weight))
        legend_layout.addWidget(legend_title)

        for i in range(3):
            color_label = QLabel(f"  ● Robot {i+1}")
            color_label.setFont(QFont("Arial", 9))
            color = ROBOT_COLORS[i]
            color_label.setStyleSheet(f"color: rgb({color.red()}, {color.green()}, {color.blue()});")
            legend_layout.addWidget(color_label)

        legend_widget.setStyleSheet("""
            QWidget {
                background-color: #e0e0e0;
                border: 1px solid #aaa;
                border-radius: 3px;
            }
        """)
        self.layout.addWidget(legend_widget)

        # 各ロボット用のラベルを作成（最大3台）
        for i in range(3):
            robot_widget = QWidget()
            robot_layout = QVBoxLayout(robot_widget)
            robot_layout.setContentsMargins(10, 10, 10, 10)

            robot_title = QLabel(f"Robot {i+1}")
            font_weight = QFont.Weight.Bold if PYQT_VERSION == 6 else QFont.Bold
            robot_title.setFont(QFont("Arial", 12, font_weight))
            robot_layout.addWidget(robot_title)

            role_label = QLabel("Role: ---")
            role_label.setFont(QFont("Arial", 10))
            robot_layout.addWidget(role_label)

            pos_label = QLabel("Position: ---")
            pos_label.setFont(QFont("Arial", 10))
            robot_layout.addWidget(pos_label)

            ball_label = QLabel("Ball: ---")
            ball_label.setFont(QFont("Arial", 10))
            robot_layout.addWidget(ball_label)

            robot_widget.setStyleSheet("""
                QWidget {
                    background-color: #f0f0f0;
                    border: 2px solid #ccc;
                    border-radius: 5px;
                }
            """)

            self.robot_labels.append({
                'role': role_label,
                'pos': pos_label,
                'ball': ball_label
            })

            self.layout.addWidget(robot_widget)

        self.layout.addStretch()

    def update_robot_info(self, messages: list[Optional[SharedMessage]]):
        """ロボット情報を更新"""
        for i, msg in enumerate(messages):
            if i >= len(self.robot_labels):
                break

            if msg is not None:
                self.robot_labels[i]['role'].setText(f"Role: {msg.role}")
                if msg.selfpos is not None:
                    self.robot_labels[i]['pos'].setText(
                        f"Position: ({msg.selfpos.x:.1f}, {msg.selfpos.y:.1f}, {msg.selfpos.theta:.2f}) cm"
                    )
                else:
                    self.robot_labels[i]['pos'].setText("Position: ---")
                if msg.ballpos is not None:
                    self.robot_labels[i]['ball'].setText(
                        f"Ball: ({msg.ballpos.x:.1f}, {msg.ballpos.y:.1f}) cm"
                    )
                else:
                    self.robot_labels[i]['ball'].setText("Ball: ---")
            else:
                self.robot_labels[i]['role'].setText("Role: ---")
                self.robot_labels[i]['pos'].setText("Position: ---")
                self.robot_labels[i]['ball'].setText("Ball: ---")


class TeamCommMonitorWindow(QMainWindow):
    """チーム通信モニターのメインウィンドウ"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Team Communication Monitor")

        # MessageReceiverを初期化
        self.receiver = MessageReceiver()

        # 中央ウィジェットとレイアウトを設定
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # ロボット情報パネル（左側）
        self.info_panel = RobotInfoPanel()
        self.info_panel.setMinimumWidth(300)
        self.info_panel.setMaximumWidth(350)
        main_layout.addWidget(self.info_panel)

        # フィールドウィジェット（右側）
        self.field_widget = FieldWidget()
        main_layout.addWidget(self.field_widget)

        # タイマーで定期的に更新（100ms = 10Hz）
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)

        # ウィンドウサイズを調整
        # フィールド表示エリア + 情報パネル
        total_width = DISPLAY_WIDTH + 350
        total_height = DISPLAY_HEIGHT + 100
        self.resize(total_width, total_height)

    def update_display(self):
        """表示を更新"""
        messages = self.receiver.get_latest_msg()
        self.field_widget.update_messages(messages)
        self.info_panel.update_robot_info(messages)

    def closeEvent(self, event):
        """ウィンドウを閉じる時のクリーンアップ"""
        # タイマーを停止
        self.update_timer.stop()
        # 受信スレッドをクリーンアップ
        self.receiver.cleanup_communication_receiver()
        event.accept()


if __name__ == "__main__":
    import sys

    app = QApplication(sys.argv)
    window = TeamCommMonitorWindow()
    window.show()
    sys.exit(app.exec())
