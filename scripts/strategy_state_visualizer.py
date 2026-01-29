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


import argparse
import json
import math
import sys
from threading import Thread

import rclpy
from booster_interface.msg import RemoteControllerState
from booster_msgs.msg import RpcReqMsg
from futbol_msgs.msg import ObjectPosition, StrategyState, DecisionLog, StrategyDecision
from localization_msgs.msg import LocalizationResult, ParticleCloud
from localization_msgs.srv import SetInitialPose
from sensor_msgs.msg import PointCloud2
from vision_interface.msg import Detections
import struct

# Try importing PyQt6 first, fall back to PyQt5 if not available
try:
    from PyQt6.QtCore import QDateTime, QLineF, QPointF, QRectF, Qt, QThread, QTimer, pyqtSignal
    from PyQt6.QtGui import QBrush, QColor, QFont, QPainter, QPainterPath, QPen, QPolygonF
    from PyQt6.QtWidgets import (
        QApplication,
        QComboBox,
        QDockWidget,
        QDoubleSpinBox,
        QGraphicsLineItem,
        QGraphicsPathItem,
        QGraphicsRectItem,
        QGraphicsScene,
        QGraphicsTextItem,
        QGraphicsView,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QPushButton,
        QSplitter,
        QTabWidget,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
    PYQT_VERSION = 6
except ImportError:
    from PyQt5.QtCore import QDateTime, QLineF, QPointF, QRectF, Qt, QThread, QTimer, pyqtSignal
    from PyQt5.QtGui import QBrush, QColor, QFont, QPainter, QPainterPath, QPen, QPolygonF
    from PyQt5.QtWidgets import (
        QApplication,
        QComboBox,
        QDockWidget,
        QDoubleSpinBox,
        QGraphicsLineItem,
        QGraphicsPathItem,
        QGraphicsRectItem,
        QGraphicsScene,
        QGraphicsTextItem,
        QGraphicsView,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QPushButton,
        QSplitter,
        QTabWidget,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
    PYQT_VERSION = 5

from rclpy.node import Node

def is_object_none(obj):
    """
    ros2のメッセージはNoneを持てないので、全ての値が0のメッセージはNone扱いにする
    """
    if obj is None:
        return True
    if math.isclose(obj.x, 0.0, abs_tol=1e-5) and math.isclose(obj.y, 0.0, abs_tol=1e-5) and math.isclose(obj.theta, 0.0, abs_tol=1e-5):
        return True
    return False

def parse_pointcloud2(cloud_msg):
    """
    Parse PointCloud2 message and extract x, y, z coordinates
    Returns a list of (x, y, z) tuples
    """
    points = []
    if cloud_msg is None:
        return points

    # Get field offsets for x, y, z
    x_offset = None
    y_offset = None
    z_offset = None

    for field in cloud_msg.fields:
        if field.name == 'x':
            x_offset = field.offset
        elif field.name == 'y':
            y_offset = field.offset
        elif field.name == 'z':
            z_offset = field.offset

    if x_offset is None or y_offset is None:
        return points

    # Parse points
    point_step = cloud_msg.point_step
    for i in range(0, len(cloud_msg.data), point_step):
        if i + point_step > len(cloud_msg.data):
            break

        # Extract x, y coordinates (assume float32)
        x = struct.unpack_from('f', cloud_msg.data, i + x_offset)[0]
        y = struct.unpack_from('f', cloud_msg.data, i + y_offset)[0]
        z = 0.0
        if z_offset is not None and i + z_offset + 4 <= len(cloud_msg.data):
            z = struct.unpack_from('f', cloud_msg.data, i + z_offset)[0]

        points.append((x, y, z))

    return points


class ClassBox(QGraphicsRectItem):
    def __init__(self, x, y, width, height, title, methods=None,
                 title_bg_color=None, methods_bg_color=None):
        super().__init__(0, 0, width, height)
        self.setPos(x, y)

        # デフォルトの色を設定
        if title_bg_color is None:
            title_bg_color = QColor(180, 200, 255)  # 濃い青
        if methods_bg_color is None:
            methods_bg_color = QColor(220, 235, 255)  # 薄い青

        # 全体の枠線（背景は透明）
        self.setPen(QPen(Qt.GlobalColor.black if PYQT_VERSION == 6 else Qt.black, 2))
        transparent = Qt.GlobalColor.transparent if PYQT_VERSION == 6 else Qt.transparent
        self.setBrush(QBrush(transparent))

        # タイトル部分の背景
        self.title_bg = QGraphicsRectItem(0, 0, width, 30, self)
        self.title_bg.setBrush(QBrush(title_bg_color))
        self.title_bg.setPen(QPen(transparent))

        # メソッド部分の背景
        self.methods_bg = QGraphicsRectItem(0, 30, width, height - 30, self)
        self.methods_bg.setBrush(QBrush(methods_bg_color))
        self.methods_bg.setPen(QPen(transparent))

        # ドラッグ可能にする
        if PYQT_VERSION == 6:
            self.setFlag(QGraphicsRectItem.GraphicsItemFlag.ItemIsMovable)
            self.setFlag(QGraphicsRectItem.GraphicsItemFlag.ItemIsSelectable)
            self.setFlag(QGraphicsRectItem.GraphicsItemFlag.ItemSendsGeometryChanges)
        else:
            self.setFlag(QGraphicsRectItem.ItemIsMovable)
            self.setFlag(QGraphicsRectItem.ItemIsSelectable)
            self.setFlag(QGraphicsRectItem.ItemSendsGeometryChanges)

        # タイトル部分（クラス名）
        self.title_text = QGraphicsTextItem(title, self)
        self.title_text.setPos(5, 5)
        self.title_text.setDefaultTextColor(Qt.GlobalColor.black if PYQT_VERSION == 6 else Qt.black)

        # 区切り線
        self.separator = QGraphicsLineItem(0, 30, width, 30, self)
        self.separator.setPen(QPen(Qt.GlobalColor.black if PYQT_VERSION == 6 else Qt.black, 1))

        # メソッド一覧
        y_offset = 35
        if methods:
            for method in methods:
                method_text = QGraphicsTextItem(method, self)
                method_text.setPos(5, y_offset)
                method_text.setDefaultTextColor(Qt.GlobalColor.black if PYQT_VERSION == 6 else Qt.black)
                # テキスト幅を設定して自動折り返しを有効にする
                method_text.setTextWidth(width - 10)  # 左右のパディング分を引く
                # 実際のテキストの高さを取得して次の位置を計算
                text_height = method_text.boundingRect().height()
                y_offset += text_height + 2  # 2ピクセルのマージン

        # ボックスの高さを動的に調整
        actual_height = max(height, y_offset + 5)  # 最小でも指定された高さ、またはコンテンツに応じて拡大
        self.setRect(0, 0, width, actual_height)

        # メソッド部分の背景の高さを調整
        self.methods_bg.setRect(0, 30, width, actual_height - 30)

        # 背景を最背面に移動
        self.methods_bg.setZValue(-1)
        self.title_bg.setZValue(-1)

        self.connections = []  # この箱に接続されている線のリスト

    def itemChange(self, change, value):
        # 位置が変更されたら接続線を更新
        item_position_change = QGraphicsRectItem.GraphicsItemChange.ItemPositionHasChanged if PYQT_VERSION == 6 else QGraphicsRectItem.ItemPositionHasChanged
        if change == item_position_change:
            for connection in self.connections:
                connection.update_position()
        return super().itemChange(change, value)

    def add_connection(self, connection):
        self.connections.append(connection)


class Connection(QGraphicsLineItem):
    def __init__(self, start_box, end_box, connection_type="association"):
        super().__init__()
        self.start_box = start_box
        self.end_box = end_box
        self.connection_type = connection_type

        self.setPen(QPen(Qt.GlobalColor.black if PYQT_VERSION == 6 else Qt.black, 2))
        self.setZValue(-1)  # 箱の後ろに描画

        # 接続を登録
        start_box.add_connection(self)
        end_box.add_connection(self)

        # 初期位置は後で設定

    def update_position(self):
        """ボックスの位置に基づいて線の位置を更新"""
        start_center = self.start_box.sceneBoundingRect().center()
        end_center = self.end_box.sceneBoundingRect().center()

        self.setLine(QLineF(start_center, end_center))


class Arrow(QGraphicsLineItem):
    """矢印付きの接続線"""
    def __init__(self, start_box, end_box, arrow_type="inheritance"):
        super().__init__()
        self.start_box = start_box
        self.end_box = end_box
        self.arrow_type = arrow_type
        self.arrow_head = None

        pen = QPen(Qt.GlobalColor.black if PYQT_VERSION == 6 else Qt.black, 2)
        if arrow_type == "dependency":
            pen.setStyle(Qt.PenStyle.DashLine if PYQT_VERSION == 6 else Qt.DashLine)
        self.setPen(pen)
        self.setZValue(-1)

        # 接続を登録
        start_box.add_connection(self)
        end_box.add_connection(self)

        # 初期位置は後で設定（sceneに追加された後）

    def update_position(self):
        """ボックスの位置に基づいて線と矢印の位置を更新"""
        start_center = self.start_box.sceneBoundingRect().center()
        end_center = self.end_box.sceneBoundingRect().center()

        self.setLine(QLineF(start_center, end_center))

        # sceneがない場合は何もしない
        if not self.scene():
            return

        # 既存の矢印を削除
        if self.arrow_head and self.arrow_head.scene():
            self.scene().removeItem(self.arrow_head)

        # 矢印の頭を作成
        angle = math.atan2(end_center.y() - start_center.y(),
                          end_center.x() - start_center.x())
        arrow_size = 15

        arrow_p1 = QPointF(
            end_center.x() - arrow_size * math.cos(angle - math.pi / 6),
            end_center.y() - arrow_size * math.sin(angle - math.pi / 6)
        )
        arrow_p2 = QPointF(
            end_center.x() - arrow_size * math.cos(angle + math.pi / 6),
            end_center.y() - arrow_size * math.sin(angle + math.pi / 6)
        )

        # 継承の場合は三角形、それ以外は線
        path = QPainterPath()
        if self.arrow_type == "inheritance":
            path.moveTo(end_center)
            path.lineTo(arrow_p1)
            path.lineTo(arrow_p2)
            path.closeSubpath()
            white_color = Qt.GlobalColor.white if PYQT_VERSION == 6 else Qt.white
            black_color = Qt.GlobalColor.black if PYQT_VERSION == 6 else Qt.black
            self.arrow_head = self.scene().addPath(path, QPen(black_color, 2),
                                                   QBrush(white_color))
        else:
            path.moveTo(end_center)
            path.lineTo(arrow_p1)
            path.moveTo(end_center)
            path.lineTo(arrow_p2)
            black_color = Qt.GlobalColor.black if PYQT_VERSION == 6 else Qt.black
            self.arrow_head = self.scene().addPath(path, QPen(black_color, 2))

        self.arrow_head.setZValue(-1)


class DiagramView(QGraphicsView):
    def __init__(self):
        super().__init__()
        self.scene = QGraphicsScene()
        self.setScene(self.scene)

        # 背景を白に
        white_color = Qt.GlobalColor.white if PYQT_VERSION == 6 else Qt.white
        self.setBackgroundBrush(QBrush(white_color))

        # シーンのサイズを設定
        self.scene.setSceneRect(0, 0, 1000, 800)

        self.box_positions = [(50, 50 + i * 150) for i in range(5)] + [(400, 50 + i * 150) for i in range(5)]
        self.box_size = (300, 100)
        self.current_items = []
        self.current_arrows = []  # 矢印を追跡するリスト

        self.ACTION_TITLE_COLOR = QColor(180, 200, 255)  # 濃い青
        self.METHOD_TITLE_COLOR = QColor(251, 208, 29)  # 黄色

        self.PRECONDITION_FAILED_COLOR = QColor(255, 180, 180)  # 赤
        self.EXECUTED_COLOR = QColor(180, 255, 180)  # 緑
        self.POSTCONDITION_SATISFIED_COLOR = QColor(200, 200, 200)  # 灰色

        self.help_position = (800, 50)

        # クラスボックスを作成
        # animal = ClassBox(100, 100, 150, 100, "Animal",
        #                  ["+eat()", "+sleep()"])
        # dog = ClassBox(100, 300, 150, 80, "Dog",
        #               ["+bark()"])
        # cat = ClassBox(350, 300, 150, 80, "Cat",
        #               ["+meow()"])
        # owner = ClassBox(550, 200, 150, 80, "Owner",
        #                 ["+name: str"])

        # self.scene.addItem(animal)
        # self.scene.addItem(dog)
        # self.scene.addItem(cat)
        # self.scene.addItem(owner)

        # # 接続線を作成してシーンに追加
        # arrow1 = Arrow(dog, animal, "inheritance")
        # arrow2 = Arrow(cat, animal, "inheritance")
        # conn1 = Connection(owner, dog, "association")
        # conn2 = Connection(owner, cat, "association")

        # self.scene.addItem(arrow1)
        # self.scene.addItem(arrow2)
        # self.scene.addItem(conn1)
        # self.scene.addItem(conn2)

        # # シーンに追加された後で位置を更新
        # arrow1.update_position()
        # arrow2.update_position()
        # conn1.update_position()
        # conn2.update_position()

        # ズーム機能
        drag_mode = QGraphicsView.DragMode.RubberBandDrag if PYQT_VERSION == 6 else QGraphicsView.RubberBandDrag
        self.setDragMode(drag_mode)
        self.draw_help()

    def draw_help(self):
        help_texts = [
            "Help:",
            "- Use mouse wheel to zoom in/out.",
            "- Drag boxes to rearrange.",
            "- Boxes color-coded by type and result.",
            "  * Blue: Action",
            "  * Yellow: Method",
            "  * Red: Precondition Failed",
            "  * Green: Executed",
            "  * Gray: Postcondition Satisfied",
        ]
        y_offset = 0
        for line in help_texts:
            help_item = QGraphicsTextItem(line)
            help_item.setPos(self.help_position[0], self.help_position[1] + y_offset)
            # Per-line text color
            line_lower = line.lower()
            if "blue" in line_lower:
                color = self.ACTION_TITLE_COLOR.darker(150)
            elif "yellow" in line_lower:
                color = self.METHOD_TITLE_COLOR.darker(150)
            elif "red" in line_lower:
                color = self.PRECONDITION_FAILED_COLOR.darker(150)
            elif "green" in line_lower:
                color = self.EXECUTED_COLOR.darker(150)
            elif "gray" in line_lower:
                color = self.POSTCONDITION_SATISFIED_COLOR.darker(150)
            else:
                color = QColor(Qt.GlobalColor.black) if PYQT_VERSION == 6 else QColor(Qt.black)
            help_item.setDefaultTextColor(color)
            self.scene.addItem(help_item)
            y_offset += 20

    def add_decision_item(self,decision :StrategyDecision):
        current_item_index = len(self.current_items)

        # decision_detailsが文字列の場合はリストに変換
        details = decision.decision_details
        if isinstance(details, str):
            # 複数行の場合は改行で分割、単一行の場合はリストに変換
            details = details.split('\n') if '\n' in details else [details]

        # デフォルト値を設定
        title_color = self.ACTION_TITLE_COLOR
        methods_bg_color = QColor(220, 235, 255)  # デフォルトの薄い青

        # タイトルの色を決定
        if decision.decision_type == StrategyDecision.DECISION_TYPE_ACTION:
            title_color = self.ACTION_TITLE_COLOR
        else:
            title_color = self.METHOD_TITLE_COLOR

        # メソッド部分の色を決定
        if decision.decision_result == StrategyDecision.DECISION_RESULT_PRECONDITION_FAILED:
            methods_bg_color = self.PRECONDITION_FAILED_COLOR
        elif decision.decision_result == StrategyDecision.DECISION_RESULT_EXECUTED:
            methods_bg_color = self.EXECUTED_COLOR
        elif decision.decision_result == StrategyDecision.DECISION_RESULT_POSTCONDITION_ALREADY_SATISFIED:
            methods_bg_color = self.POSTCONDITION_SATISFIED_COLOR

        class_item = ClassBox(self.box_positions[current_item_index][0],
                              self.box_positions[current_item_index][1],
                              self.box_size[0], self.box_size[1],
                              decision.decision_name,
                              details,
                              title_bg_color=title_color,
                              methods_bg_color=methods_bg_color)
        self.current_items.append(class_item)
        self.scene.addItem(class_item)
        if len(self.current_items) >= 2:
            arrow = Connection(self.current_items[-2], self.current_items[-1])
            self.scene.addItem(arrow)
            arrow.update_position()
            self.current_arrows.append(arrow)

    def clear_decision_items(self):
        for arrow in self.current_arrows:
            self.scene.removeItem(arrow)
        self.current_arrows.clear()
        for item in self.current_items:
            self.scene.removeItem(item)
        self.current_items.clear()

    def wheelEvent(self, event):
        """マウスホイールでズーム"""
        zoom_factor = 1.15
        if event.angleDelta().y() > 0:
            self.scale(zoom_factor, zoom_factor)
        else:
            self.scale(1 / zoom_factor, 1 / zoom_factor)


class FieldWidget(QWidget):
    """Widget for visualizing the soccer field and robot/ball positions"""

    # Signal for clicked position (x, y, theta in field coordinates)
    position_clicked = pyqtSignal(float, float, float)

    def __init__(self, enable_click=False, scale_factor=0.9):
        super().__init__()
        self.setMinimumSize(600, 400)
        self.enable_click = enable_click
        self.clicked_position = None  # Store clicked position in field coordinates (x, y, theta)
        self.scale_factor = scale_factor  # Scale factor for field size (default 0.9)
        self.is_dragging = False  # Track if currently dragging
        self.drag_start_pos = None  # Screen position where drag started

        # Field dimensions (in meters)
        # Based on strategy_sim/scripts/setting.py: WIDTH=1400, HEIGHT=900 (1px = 1cm)
        self.field_length = 14.0  # WIDTH in meters
        self.field_width = 9.0  # HEIGHT in meters
        self.goal_width = 2.6  # 260 pixels / 100 = 2.6m
        self.goal_depth = 0.6
        self.penalty_area_length = 3.0  # PENALTY_WIDTH = 300 pixels = 3.0m
        self.penalty_area_width = 6.0  # PENALTY_HEIGHT = 600 pixels = 6.0m
        self.goal_area_length = 1.0  # GOALAREA_WIDTH = 100 pixels = 1.0m
        self.goal_area_width = 4.0  # GOALAREA_HEIGHT = 400 pixels = 4.0m
        self.penalty_spot_distance = 2.1  # 210 pixels = 2.1m from goal line
        self.center_circle_radius = 1.5  # 150 pixels = 1.5m

        # Positions
        self.robot_position = None
        self.ball_position = None
        self.shared_ball_global = None
        self.ball_detected_by_camera = None
        self.opponent_positions = []
        self.allies_positions = []
        self.opponent_goalpost = []
        self.opponent_goalpost_global = []
        self.normal_obstacles = []
        self.target_position = None
        self.target_position_timestamp = None
        self.particle_cloud = None
        self.white_line_particles = None
        self.detected_line_marks = []  # Store detected line marks (TCross, LCross, XCross, PenaltyPoint)

        # Timer for clearing target position after 3 seconds
        self.target_clear_timer = QTimer()
        self.target_clear_timer.timeout.connect(self.clear_target_position)
        self.target_clear_timer.setSingleShot(True)

        # Colors
        self.field_color = QColor(0, 150, 0)
        self.line_color = Qt.GlobalColor.white if PYQT_VERSION == 6 else Qt.white
        self.robot_color = QColor(0, 0, 255)
        self.ball_color = QColor(255, 165, 0)  # オレンジ色（ローカル観測）
        self.shared_ball_color = QColor(0, 255, 255)  # シアン色（共有グローバル）
        self.ball_detected_by_camera_color = QColor(255, 105, 180)  # ホットピンク色（カメラ検出）
        self.opponent_color = QColor(255, 0, 0)
        self.ally_color = QColor(0, 255, 0)
        self.target_color = QColor(255, 0, 255)  # マゼンタ色

    def update_positions(
        self,
        robot_pos,
        ball_pos,
        shared_ball_pos,
        ball_detected_by_camera_pos,
        opponents,
        allies,
        goalposts,
        goalposts_global=None,
        normal_obstacles=None,
        target_pos=None,
        particle_cloud=None,
        white_line_particles=None,
        detected_line_marks=None,
    ):
        self.robot_position = robot_pos
        self.ball_position = ball_pos
        self.shared_ball_global = shared_ball_pos
        self.ball_detected_by_camera = ball_detected_by_camera_pos
        self.opponent_positions = opponents
        self.allies_positions = allies
        self.opponent_goalpost = goalposts
        self.opponent_goalpost_global = goalposts_global if goalposts_global else []
        self.normal_obstacles = normal_obstacles if normal_obstacles else []
        # Don't overwrite target_position if target_pos is None
        # This preserves the target position until the timer clears it
        if target_pos is not None:
            self.target_position = target_pos
        if particle_cloud is not None:
            self.particle_cloud = particle_cloud
        if white_line_particles is not None:
            self.white_line_particles = white_line_particles
        if detected_line_marks is not None:
            self.detected_line_marks = detected_line_marks
        self.update()

    def update_target_position(self, target_pos):
        # Only update if we receive a valid target position
        if target_pos and hasattr(target_pos, "x") and hasattr(target_pos, "y"):
            self.target_position = target_pos
            self.target_position_timestamp = QDateTime.currentDateTime()
            self.target_clear_timer.stop()
            self.target_clear_timer.start(1500)
            self.update()
        # If target_pos is None or invalid, keep displaying the previous value
        # The timer will continue counting down from its last start

    def clear_target_position(self):
        self.target_position = None
        self.target_position_timestamp = None
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        if PYQT_VERSION == 6:
            painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        else:
            painter.setRenderHint(QPainter.Antialiasing)

        # Calculate scale
        widget_width = self.width()
        widget_height = self.height()
        scale_x = widget_width / (self.field_length + 2.0)
        scale_y = widget_height / (self.field_width + 2.0)
        scale = min(scale_x, scale_y) * self.scale_factor

        # Center the field
        offset_x = (widget_width - self.field_length * scale) / 2
        offset_y = (widget_height - self.field_width * scale) / 2

        # Draw field
        painter.fillRect(
            int(offset_x),
            int(offset_y),
            int(self.field_length * scale),
            int(self.field_width * scale),
            self.field_color,
        )

        # Draw field lines
        pen = QPen(self.line_color, 2)
        painter.setPen(pen)

        # Field boundary
        painter.drawRect(
            int(offset_x),
            int(offset_y),
            int(self.field_length * scale),
            int(self.field_width * scale),
        )

        # Center line
        center_x = offset_x + (self.field_length * scale) / 2
        painter.drawLine(
            int(center_x),
            int(offset_y),
            int(center_x),
            int(offset_y + self.field_width * scale),
        )

        # Center circle
        center_y = offset_y + (self.field_width * scale) / 2
        painter.drawEllipse(
            int(center_x - self.center_circle_radius * scale),
            int(center_y - self.center_circle_radius * scale),
            int(2 * self.center_circle_radius * scale),
            int(2 * self.center_circle_radius * scale),
        )

        # Goals
        goal_height = self.goal_width * scale
        goal_y = center_y - goal_height / 2

        # Left goal
        painter.drawRect(
            int(offset_x - self.goal_depth * scale),
            int(goal_y),
            int(self.goal_depth * scale),
            int(goal_height),
        )

        # Right goal
        painter.drawRect(
            int(offset_x + self.field_length * scale),
            int(goal_y),
            int(self.goal_depth * scale),
            int(goal_height),
        )

        # Penalty areas
        penalty_y = center_y - (self.penalty_area_width * scale) / 2

        # Left penalty area
        painter.drawRect(
            int(offset_x),
            int(penalty_y),
            int(self.penalty_area_length * scale),
            int(self.penalty_area_width * scale),
        )

        # Right penalty area
        painter.drawRect(
            int(
                offset_x + self.field_length * scale - self.penalty_area_length * scale
            ),
            int(penalty_y),
            int(self.penalty_area_length * scale),
            int(self.penalty_area_width * scale),
        )

        # Goal areas
        goal_area_y = center_y - (self.goal_area_width * scale) / 2

        # Left goal area
        painter.drawRect(
            int(offset_x),
            int(goal_area_y),
            int(self.goal_area_length * scale),
            int(self.goal_area_width * scale),
        )

        # Right goal area
        painter.drawRect(
            int(
                offset_x + self.field_length * scale - self.goal_area_length * scale
            ),
            int(goal_area_y),
            int(self.goal_area_length * scale),
            int(self.goal_area_width * scale),
        )

        # Penalty spots
        # Left penalty spot
        painter.drawEllipse(
            int(offset_x + self.penalty_spot_distance * scale - 3),
            int(center_y - 3),
            6,
            6,
        )

        # Right penalty spot
        painter.drawEllipse(
            int(offset_x + (self.field_length - self.penalty_spot_distance) * scale - 3),
            int(center_y - 3),
            6,
            6,
        )

        # Draw positions
        def draw_position(pos, color, label="", is_robot=True, is_local=False):
            if pos and hasattr(pos, "x") and hasattr(pos, "y"):
                # Convert from cm to m
                x_m = pos.x / 100.0
                y_m = pos.y / 100.0

                # Convert from local to global coordinates if needed
                if is_local and self.robot_position:
                    # Get robot's global position and orientation
                    robot_x = self.robot_position.x / 100.0
                    robot_y = self.robot_position.y / 100.0
                    robot_theta = self.robot_position.theta

                    # Apply rotation and translation
                    # Transform from robot's local coordinate system to global coordinate system
                    cos_theta = math.cos(robot_theta)
                    sin_theta = math.sin(robot_theta)
                    global_x = robot_x + x_m * cos_theta - y_m * sin_theta
                    global_y = robot_y + x_m * sin_theta + y_m * cos_theta

                    x_m = global_x
                    y_m = global_y

                x = offset_x + (x_m + self.field_length / 2) * scale
                y = (
                    offset_y + (-y_m + self.field_width / 2) * scale
                )  # Invert y for screen coordinates

                if is_robot:
                    # Draw robot as triangle pointing in theta direction
                    size = 15
                    theta = pos.theta if hasattr(pos, "theta") else 0

                    # For local objects other than self, theta is already in global coordinates
                    # Only self robot needs no conversion since it's already global

                    # Check if this is the robot itself (not a local object)
                    if not is_local:
                        # Robot's own position - reverse rotation for display
                        # Calculate triangle points (時計回りが正に対応)
                        points = []
                        for angle in [0, 2.356, 3.927]:  # 0, 135, 225 degrees
                            px = x + size * math.cos(-theta + angle)
                            py = y + size * math.sin(-theta + angle)
                            points.append(QPointF(px, py))
                    else:
                        # Other objects - keep original rotation
                        # Calculate triangle points (時計回りが正に対応)
                        points = []
                        for angle in [0, 2.356, 3.927]:  # 0, 135, 225 degrees
                            px = x + size * math.cos(-theta + angle)
                            py = y + size * math.sin(-theta + angle)
                            points.append(QPointF(px, py))

                    black_color = Qt.GlobalColor.black if PYQT_VERSION == 6 else Qt.black
                    painter.setBrush(QBrush(color))
                    painter.setPen(QPen(black_color, 1))
                    painter.drawPolygon(QPolygonF(points))
                else:
                    # Draw ball as circle
                    black_color = Qt.GlobalColor.black if PYQT_VERSION == 6 else Qt.black
                    painter.setBrush(QBrush(color))
                    painter.setPen(QPen(black_color, 1))
                    painter.drawEllipse(int(x - 8), int(y - 8), 16, 16)

                if label:
                    black_color = Qt.GlobalColor.black if PYQT_VERSION == 6 else Qt.black
                    painter.setPen(QPen(black_color))
                    painter.drawText(int(x + 10), int(y - 10), label)

        # Draw robot
        if self.robot_position:
            draw_position(self.robot_position, self.robot_color, "Robot")

        # Draw ball (local observation)
        if self.ball_position and not is_object_none(self.ball_position):
            draw_position(
                self.ball_position,
                self.ball_color,
                "Ball",
                is_robot=False,
                is_local=True,
            )
        
        # Draw shared ball global
        if self.shared_ball_global and not is_object_none(self.shared_ball_global):
            draw_position(
                self.shared_ball_global,
                self.shared_ball_color,
                "Shared Ball",
                is_robot=False,
                is_local=False,  # Already in global coordinates
            )

        # Draw ball detected by camera
        if self.ball_detected_by_camera and not is_object_none(self.ball_detected_by_camera):
            draw_position(
                self.ball_detected_by_camera,
                self.ball_detected_by_camera_color,
                "Camera Ball",
                is_robot=False,
                is_local=True,  # Convert from local to global coordinates
            )

        # Draw opponents
        for i, opp in enumerate(self.opponent_positions):
            draw_position(opp, self.opponent_color, f"O{i + 1}", is_local=True)

        # Draw allies
        for i, ally in enumerate(self.allies_positions):
            draw_position(ally, self.ally_color, f"A{i + 1}", is_local=True)

        # Draw goalposts
        for i, post in enumerate(self.opponent_goalpost):
            if post and hasattr(post, "x") and hasattr(post, "y"):
                # Convert from cm to m
                x_m = post.x / 100.0
                y_m = post.y / 100.0

                # Convert from local to global coordinates
                if self.robot_position:
                    robot_x = self.robot_position.x / 100.0
                    robot_y = self.robot_position.y / 100.0
                    robot_theta = self.robot_position.theta

                    # Transform from robot's local coordinate system to global coordinate system
                    cos_theta = math.cos(robot_theta)
                    sin_theta = math.sin(robot_theta)
                    global_x = robot_x + x_m * cos_theta - y_m * sin_theta
                    global_y = robot_y + x_m * sin_theta + y_m * cos_theta

                    x_m = global_x
                    y_m = global_y

                x = offset_x + (x_m + self.field_length / 2) * scale
                y = (
                    offset_y + (-y_m + self.field_width / 2) * scale
                )  # Invert y for screen coordinates
                yellow_color = Qt.GlobalColor.yellow if PYQT_VERSION == 6 else Qt.yellow
                black_color = Qt.GlobalColor.black if PYQT_VERSION == 6 else Qt.black
                painter.setBrush(QBrush(yellow_color))
                painter.setPen(QPen(black_color, 2))
                painter.drawRect(int(x - 5), int(y - 5), 10, 10)

        # Draw global goalposts (in cyan to distinguish from local goalposts)
        for i, post in enumerate(self.opponent_goalpost_global):
            if post and hasattr(post, "x") and hasattr(post, "y"):
                # Already in global coordinates, just convert from cm to m
                x_m = post.x / 100.0
                y_m = post.y / 100.0

                x = offset_x + (x_m + self.field_length / 2) * scale
                y = (
                    offset_y + (-y_m + self.field_width / 2) * scale
                )  # Invert y for screen coordinates
                cyan_color = Qt.GlobalColor.cyan if PYQT_VERSION == 6 else Qt.cyan
                black_color = Qt.GlobalColor.black if PYQT_VERSION == 6 else Qt.black
                painter.setBrush(QBrush(cyan_color))
                painter.setPen(QPen(black_color, 2))
                painter.drawRect(int(x - 5), int(y - 5), 10, 10)

        # Draw normal obstacles (in gray)
        for i, obstacle in enumerate(self.normal_obstacles):
            if obstacle and hasattr(obstacle, "x") and hasattr(obstacle, "y"):
                # Convert from cm to m
                x_m = obstacle.x / 100.0
                y_m = obstacle.y / 100.0

                # Convert from local to global coordinates
                if self.robot_position:
                    robot_x = self.robot_position.x / 100.0
                    robot_y = self.robot_position.y / 100.0
                    robot_theta = self.robot_position.theta

                    # Transform from robot's local coordinate system to global coordinate system
                    cos_theta = math.cos(robot_theta)
                    sin_theta = math.sin(robot_theta)
                    global_x = robot_x + x_m * cos_theta - y_m * sin_theta
                    global_y = robot_y + x_m * sin_theta + y_m * cos_theta

                    x_m = global_x
                    y_m = global_y

                x = offset_x + (x_m + self.field_length / 2) * scale
                y = (
                    offset_y + (-y_m + self.field_width / 2) * scale
                )  # Invert y for screen coordinates
                gray_color = Qt.GlobalColor.gray if PYQT_VERSION == 6 else Qt.gray
                black_color = Qt.GlobalColor.black if PYQT_VERSION == 6 else Qt.black
                painter.setBrush(QBrush(gray_color))
                painter.setPen(QPen(black_color, 2))
                painter.drawRect(int(x - 5), int(y - 5), 10, 10)

        # Draw target position
        if self.target_position:
            draw_position(
                self.target_position,
                self.target_color,
                "Target",
                is_robot=True,
                is_local=False,  # Target position is in global coordinates
            )

        # Draw particle cloud
        if self.particle_cloud and hasattr(self.particle_cloud, 'particles'):
            # Find max weight for normalization
            max_weight = max([p.weight for p in self.particle_cloud.particles]) if self.particle_cloud.particles else 1.0

            for particle in self.particle_cloud.particles:
                # Convert from m to m (particles are already in meters)
                x_m = particle.x
                y_m = particle.y
                theta = particle.theta

                # Convert to screen coordinates
                x = offset_x + (x_m + self.field_length / 2) * scale
                y = offset_y + (-y_m + self.field_width / 2) * scale  # Invert y for screen coordinates

                # Calculate alpha based on weight (normalized)
                normalized_weight = particle.weight / max_weight if max_weight > 0 else 0
                alpha = int(50 + 205 * normalized_weight)  # Range from 50 to 255

                # Size based on weight
                arrow_length = 10 + 15 * normalized_weight  # Range from 10 to 25 pixels
                arrow_head_length = 5 + 5 * normalized_weight  # Range from 5 to 10 pixels
                arrow_head_angle = 0.5  # radians

                # Draw particle as arrow
                particle_color = QColor(255, 255, 0, alpha)  # Yellow with variable alpha

                # Main arrow line
                end_x = x + arrow_length * math.cos(-theta)  # Negative because screen y is inverted
                end_y = y + arrow_length * math.sin(-theta)

                pen_width = 1 + int(2 * normalized_weight)  # Range from 1 to 3
                painter.setPen(QPen(particle_color, pen_width))
                painter.drawLine(int(x), int(y), int(end_x), int(end_y))

                # Arrow head (two lines)
                head_angle1 = -theta + math.pi - arrow_head_angle
                head_angle2 = -theta + math.pi + arrow_head_angle

                head1_x = end_x + arrow_head_length * math.cos(head_angle1)
                head1_y = end_y + arrow_head_length * math.sin(head_angle1)
                head2_x = end_x + arrow_head_length * math.cos(head_angle2)
                head2_y = end_y + arrow_head_length * math.sin(head_angle2)

                painter.drawLine(int(end_x), int(end_y), int(head1_x), int(head1_y))
                painter.drawLine(int(end_x), int(end_y), int(head2_x), int(head2_y))

        # Draw white line particles
        if self.white_line_particles:
            points = parse_pointcloud2(self.white_line_particles)
            white_color = QColor(200, 200, 200, 120)  # Light gray with alpha
            painter.setPen(QPen(white_color, 0))
            painter.setBrush(QBrush(white_color))

            for point_x, point_y, point_z in points:
                # Convert from robot local coordinates to global coordinates
                if self.robot_position:
                    robot_x = self.robot_position.x / 100.0  # Convert from cm to m
                    robot_y = self.robot_position.y / 100.0
                    robot_theta = self.robot_position.theta

                    # Transform from robot's local coordinate system to global coordinate system
                    cos_theta = math.cos(robot_theta)
                    sin_theta = math.sin(robot_theta)
                    global_x = robot_x + point_x * cos_theta - point_y * sin_theta
                    global_y = robot_y + point_x * sin_theta + point_y * cos_theta

                    # Convert to screen coordinates
                    x = offset_x + (global_x + self.field_length / 2) * scale
                    y = offset_y + (-global_y + self.field_width / 2) * scale  # Invert y for screen coordinates

                    # Draw as small circle
                    painter.drawEllipse(int(x - 2), int(y - 2), 4, 4)

        # Draw detected line marks (TCross, LCross, XCross, PenaltyPoint)
        if self.detected_line_marks and self.robot_position:
            robot_x = self.robot_position.x / 100.0  # Convert from cm to m
            robot_y = self.robot_position.y / 100.0
            robot_theta = self.robot_position.theta

            for mark in self.detected_line_marks:
                if len(mark['position']) >= 2:
                    # mark['position'] is in robot's local coordinates (meters)
                    local_x = mark['position'][0]
                    local_y = mark['position'][1]

                    # Transform from robot's local coordinate system to global coordinate system
                    cos_theta = math.cos(robot_theta)
                    sin_theta = math.sin(robot_theta)
                    global_x = robot_x + local_x * cos_theta - local_y * sin_theta
                    global_y = robot_y + local_x * sin_theta + local_y * cos_theta

                    # Convert to screen coordinates
                    x = offset_x + (global_x + self.field_length / 2) * scale
                    y = offset_y + (-global_y + self.field_width / 2) * scale  # Invert y for screen coordinates

                    # Set color based on label
                    label = mark['label']
                    if label == 'TCross':
                        mark_color = QColor(255, 0, 255)  # Magenta
                    elif label == 'LCross':
                        mark_color = QColor(0, 255, 255)  # Cyan
                    elif label == 'XCross':
                        mark_color = QColor(255, 255, 0)  # Yellow
                    elif label == 'PenaltyPoint':
                        mark_color = QColor(255, 128, 0)  # Orange
                    else:
                        continue  # Skip other labels

                    # Draw marker as circle with label
                    black_color = Qt.GlobalColor.black if PYQT_VERSION == 6 else Qt.black
                    painter.setBrush(QBrush(mark_color))
                    painter.setPen(QPen(black_color, 2))
                    painter.drawEllipse(int(x - 8), int(y - 8), 16, 16)

                    # Draw label text
                    painter.setPen(QPen(black_color))
                    painter.setFont(QFont("Arial", 8, QFont.Weight.Bold if PYQT_VERSION == 6 else QFont.Bold))
                    painter.drawText(int(x + 10), int(y - 10), label)

        # Draw clicked position as arrow
        if self.clicked_position:
            x_m, y_m, theta = self.clicked_position
            x = offset_x + (x_m + self.field_length / 2) * scale
            y = offset_y + (-y_m + self.field_width / 2) * scale

            # Draw arrow pointing in theta direction
            arrow_length = 40
            arrow_head_length = 15
            arrow_head_angle = 0.5  # radians

            # Main arrow line
            end_x = x + arrow_length * math.cos(-theta)  # Negative because screen y is inverted
            end_y = y + arrow_length * math.sin(-theta)

            red_color = Qt.GlobalColor.red if PYQT_VERSION == 6 else Qt.red
            painter.setPen(QPen(red_color, 3))
            painter.drawLine(int(x), int(y), int(end_x), int(end_y))

            # Arrow head (two lines)
            head_angle1 = -theta + math.pi - arrow_head_angle
            head_angle2 = -theta + math.pi + arrow_head_angle

            head1_x = end_x + arrow_head_length * math.cos(head_angle1)
            head1_y = end_y + arrow_head_length * math.sin(head_angle1)
            head2_x = end_x + arrow_head_length * math.cos(head_angle2)
            head2_y = end_y + arrow_head_length * math.sin(head_angle2)

            painter.drawLine(int(end_x), int(end_y), int(head1_x), int(head1_y))
            painter.drawLine(int(end_x), int(end_y), int(head2_x), int(head2_y))

            # Draw circle at base
            painter.setBrush(QBrush(red_color))
            painter.drawEllipse(int(x - 5), int(y - 5), 10, 10)

            # Draw coordinate text
            black_color = Qt.GlobalColor.black if PYQT_VERSION == 6 else Qt.black
            white_color = Qt.GlobalColor.white if PYQT_VERSION == 6 else Qt.white
            painter.setPen(QPen(black_color))
            painter.setFont(QFont("Arial", 10))
            text = f"({x_m:.2f}, {y_m:.2f}, θ={math.degrees(theta):.1f}°)"
            # Draw background for text
            text_rect = painter.fontMetrics().boundingRect(text)
            text_rect.moveTopLeft(QPointF(x + 10, y - 20).toPoint())
            painter.fillRect(text_rect.adjusted(-2, -2, 2, 2), white_color)
            painter.drawText(int(x + 10), int(y - 10), text)

    def mousePressEvent(self, event):
        """Handle mouse press events"""
        if not self.enable_click:
            return

        # Get click position
        click_x = event.pos().x() if PYQT_VERSION == 6 else event.x()
        click_y = event.pos().y() if PYQT_VERSION == 6 else event.y()

        # Calculate scale and offset (same as in paintEvent)
        widget_width = self.width()
        widget_height = self.height()
        scale_x = widget_width / (self.field_length + 2.0)
        scale_y = widget_height / (self.field_width + 2.0)
        scale = min(scale_x, scale_y) * self.scale_factor

        offset_x = (widget_width - self.field_length * scale) / 2
        offset_y = (widget_height - self.field_width * scale) / 2

        # Convert screen coordinates to field coordinates (in meters)
        x_m = (click_x - offset_x) / scale - self.field_length / 2
        y_m = -((click_y - offset_y) / scale - self.field_width / 2)

        # Store clicked position with initial theta = 0
        theta = 0.0
        self.clicked_position = (x_m, y_m, theta)
        self.is_dragging = True
        self.drag_start_pos = (click_x, click_y)

        # Emit signal with theta
        self.position_clicked.emit(x_m, y_m, theta)

        # Update display
        self.update()

    def mouseMoveEvent(self, event):
        """Handle mouse move events for dragging"""
        if not self.enable_click or not self.is_dragging or not self.clicked_position:
            return

        # Get current mouse position
        current_x = event.pos().x() if PYQT_VERSION == 6 else event.x()
        current_y = event.pos().y() if PYQT_VERSION == 6 else event.y()

        # Calculate angle from click position to current position
        x_m, y_m, _ = self.clicked_position

        # Calculate scale and offset
        widget_width = self.width()
        widget_height = self.height()
        scale_x = widget_width / (self.field_length + 2.0)
        scale_y = widget_height / (self.field_width + 2.0)
        scale = min(scale_x, scale_y) * self.scale_factor

        offset_x = (widget_width - self.field_length * scale) / 2
        offset_y = (widget_height - self.field_width * scale) / 2

        # Get screen position of clicked point
        screen_x = offset_x + (x_m + self.field_length / 2) * scale
        screen_y = offset_y + (-y_m + self.field_width / 2) * scale

        # Calculate angle (in field coordinates, not screen coordinates)
        dx = current_x - screen_x
        dy = current_y - screen_y

        if abs(dx) > 1 or abs(dy) > 1:  # Only update if dragged far enough
            # Calculate theta (negative dy because screen y is inverted)
            theta = math.atan2(-dy, dx)

            # Update clicked position with new angle
            self.clicked_position = (x_m, y_m, theta)

            # Emit signal with updated theta
            self.position_clicked.emit(x_m, y_m, theta)

            # Update display
            self.update()

    def mouseReleaseEvent(self, event):
        """Handle mouse release events"""
        if not self.enable_click:
            return

        self.is_dragging = False
        self.drag_start_pos = None


class ROS2Thread(QThread):
    strategy_state_received = pyqtSignal(object)
    target_position_received = pyqtSignal(object)
    particle_cloud_received = pyqtSignal(object)
    white_line_particles_received = pyqtSignal(object)
    localization_result_received = pyqtSignal(object)
    detections_received = pyqtSignal(object)
    decision_log_received = pyqtSignal(object)

    def __init__(self, sim_robot_id=None):
        super().__init__()
        self.node = None
        self.strategy_subscription = None
        self.target_position_subscription = None
        self.particle_cloud_subscription = None
        self.white_line_particles_subscription = None
        self.localization_result_subscription = None
        self.detections_subscription = None
        self.decision_subscription = None
        self.sim_robot_id = sim_robot_id  # デフォルトはNoneにする。この場合は実機。

        # パブリッシャーの初期化
        self.publisher1 = None
        self.publisher2 = None
        self.publisher3 = None
        self.publisher4 = None
        self.publisher5 = None
        self.publisher6 = None
        self.publisher_real_robot = None
        self.publisher_remote_controller = None

    def run(self):
        rclpy.init()
        self.node = Node("strategy_state_visualizer")
        if self.sim_robot_id is not None:
            self.strategy_subscription = self.node.create_subscription(
                StrategyState,
                "/StrategyState_" + str(self.sim_robot_id),
                self.strategy_state_callback,
                10,
            )
            self.target_position_subscription = self.node.create_subscription(
                ObjectPosition,
                "/TargetPosition_" + str(self.sim_robot_id),
                self.target_position_callback,
                10,
            )
            self.particle_cloud_subscription = self.node.create_subscription(
                ParticleCloud,
                "/particle_cloud_" + str(self.sim_robot_id),
                self.particle_cloud_callback,
                10,
            )
            self.white_line_particles_subscription = self.node.create_subscription(
                PointCloud2,
                "/whiteline_points_" + str(self.sim_robot_id),
                self.white_line_particles_callback,
                10,
            )
            self.localization_result_subscription = self.node.create_subscription(
                LocalizationResult,
                "/localization_result_" + str(self.sim_robot_id),
                self.localization_result_callback,
                10,
            )
            self.detections_subscription = self.node.create_subscription(
                Detections,
                "/booster_vision/detection_" + str(self.sim_robot_id),
                self.detections_callback,
                10,
            )
            self.decision_subscription = self.node.create_subscription(
                DecisionLog,
                "/DecisionLog_" + str(self.sim_robot_id),
                self.decision_log_callback,
                10,
            )
            self.set_pose_client = self.node.create_client(
                SetInitialPose,
                'set_initial_pose_' + str(self.sim_robot_id)
                )
            # パブリッシャーの作成
            self.publisher1 = self.node.create_publisher(RpcReqMsg, "/LocoApiTopicReq_1", 10)
            self.publisher2 = self.node.create_publisher(RpcReqMsg, "/LocoApiTopicReq_2", 10)
            self.publisher3 = self.node.create_publisher(RpcReqMsg, "/LocoApiTopicReq_3", 10)
            self.publisher4 = self.node.create_publisher(RpcReqMsg, "/LocoApiTopicReq_4", 10)
            self.publisher5 = self.node.create_publisher(RpcReqMsg, "/LocoApiTopicReq_5", 10)
            self.publisher6 = self.node.create_publisher(RpcReqMsg, "/LocoApiTopicReq_6", 10)
        else:
            self.strategy_subscription = self.node.create_subscription(
                StrategyState, "/StrategyState", self.strategy_state_callback, 10
            )
            self.target_position_subscription = self.node.create_subscription(
                ObjectPosition, "/TargetPosition", self.target_position_callback, 10
            )
            self.particle_cloud_subscription = self.node.create_subscription(
                ParticleCloud, "/particle_cloud", self.particle_cloud_callback, 10
            )
            self.white_line_particles_subscription = self.node.create_subscription(
                PointCloud2, "/whiteline_points", self.white_line_particles_callback, 10
            )
            self.localization_result_subscription = self.node.create_subscription(
                LocalizationResult, "/localization_result", self.localization_result_callback, 10
            )
            self.detections_subscription = self.node.create_subscription(
                Detections, "/booster_vision/detection", self.detections_callback, 10
            )
            self.decision_subscription = self.node.create_subscription(
                DecisionLog, "/DecisionLog", self.decision_log_callback, 10,
            )
            self.set_pose_client = self.node.create_client(
                SetInitialPose,
                'set_initial_pose'
                )

        # パブリッシャーは共通
        self.publisher_real_robot = self.node.create_publisher(RpcReqMsg, "/LocoApiTopicReq", 10)
        self.publisher_remote_controller = self.node.create_publisher(
            RemoteControllerState, "/remote_controller_state", 10
        )

        rclpy.spin(self.node)

    def strategy_state_callback(self, msg):
        self.strategy_state_received.emit(msg)

    def target_position_callback(self, msg):
        self.target_position_received.emit(msg)

    def particle_cloud_callback(self, msg):
        self.particle_cloud_received.emit(msg)

    def white_line_particles_callback(self, msg):
        self.white_line_particles_received.emit(msg)

    def localization_result_callback(self, msg):
        self.localization_result_received.emit(msg)

    def detections_callback(self, msg):
        self.detections_received.emit(msg)

    def decision_log_callback(self, msg):
        self.decision_log_received.emit(msg)

    def send_request(self,x,y,theta):
        request = SetInitialPose.Request()
        request.x = float(x)
        request.y = float(y)
        request.theta = float(theta)
        request.reset_particles = True
        resp = self.set_pose_client.call_async(request)
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

    def get_publisher(self, index):
        """指定されたインデックスのパブリッシャーを返す"""
        publishers = [
            self.publisher1,
            self.publisher2,
            self.publisher3,
            self.publisher4,
            self.publisher5,
            self.publisher6,
            self.publisher_real_robot,
        ]
        if 0 <= index < len(publishers):
            return publishers[index]
        else:
            raise IndexError("Publisher index out of range")

    def create_msg(self, vx, vy, vyaw):
        """RpcReqMsgメッセージを作成"""
        command_msg = RpcReqMsg()
        command_msg.header = json.dumps({"api_id": 2001})
        command_msg.uuid = json.dumps({"uuid": "123456789"})
        command_msg.body = json.dumps({"vx": vx, "vy": vy, "vyaw": vyaw})
        return command_msg

    def stop(self):
        if self.node:
            self.node.destroy_subscription(self.strategy_subscription)
            self.node.destroy_subscription(self.target_position_subscription)
            self.node.destroy_subscription(self.particle_cloud_subscription)
            self.node.destroy_subscription(self.white_line_particles_subscription)
            self.node.destroy_subscription(self.localization_result_subscription)
            self.node.destroy_subscription(self.detections_subscription)
            self.node.destroy_node()
        rclpy.shutdown()


class StrategyStateVisualizer(QMainWindow):
    def __init__(self, sim_robot_id=None):
        super().__init__()
        title = "Strategy State Visualizer"
        if sim_robot_id is not None:
            title += f" - Robot {sim_robot_id}"
        self.setWindowTitle(title)
        self.setGeometry(100, 100, 1200, 800)
        self.setDockNestingEnabled(True)

        # Tab 1: Full view (Field + Status)
        tab1 = QWidget()
        tab1_layout = QHBoxLayout(tab1)

        # Left side - Field visualization
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)

        # Field widget
        self.field_widget = FieldWidget()
        left_layout.addWidget(self.field_widget)

        # Right side - Status information
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)

        # Status group
        status_group = QGroupBox("Status Information")
        status_layout = QGridLayout()

        # Create labels for status
        self.labels = {}
        status_fields = [
            ("ball_position_relative", "Ball Position (Relative)"),
            ("shared_ball_global", "Shared Ball Global"),
            ("current_robot_field", "Current Robot Field"),
            ("game_state", "Game State"),
            ("secondary_state", "Secondary State"),
            ("penalty", "Penalty"),
            ("our_field", "Our Field"),
            ("team_color", "Team Color"),
            ("role", "Role"),
            ("robot_mode", "Robot Mode"),
            ("neck_yaw_rad", "Neck Yaw (rad)"),
            ("target_position", "Target Position"),
        ]

        for i, (field, label) in enumerate(status_fields):
            status_layout.addWidget(QLabel(f"{label}:"), i, 0)
            self.labels[field] = QLabel("--")
            status_layout.addWidget(self.labels[field], i, 1)

        status_group.setLayout(status_layout)
        right_layout.addWidget(status_group)

        # Position information
        position_group = QGroupBox("Position Information")
        position_layout = QVBoxLayout()

        self.position_text = QTextEdit()
        self.position_text.setReadOnly(True)
        self.position_text.setMinimumHeight(400)
        self.position_text.setMaximumHeight(1200)
        position_layout.addWidget(self.position_text)

        position_group.setLayout(position_layout)
        right_layout.addWidget(position_group)

        # Add stretch
        right_layout.addStretch()

        # Add widgets to splitter
        horizontal_orientation = Qt.Orientation.Horizontal if PYQT_VERSION == 6 else Qt.Horizontal
        splitter = QSplitter(horizontal_orientation)
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 1)

        tab1_layout.addWidget(splitter)
        
        dock1 = QDockWidget("Full View", self)
        dock1.setWidget(tab1)
        dock1.setFloating(False)
        # Enable movable and floatable features for both PyQt5 and PyQt6
        if PYQT_VERSION == 6:
            from PyQt6.QtWidgets import QDockWidget as QDockWidgetPyQt6
            dock1.setFeatures(QDockWidgetPyQt6.DockWidgetFeature.DockWidgetMovable | QDockWidgetPyQt6.DockWidgetFeature.DockWidgetFloatable)
        else:
            dock1.setFeatures(QDockWidget.AllDockWidgetFeatures)
        self.addDockWidget(Qt.DockWidgetArea.TopDockWidgetArea, dock1)

        # Tab 2: Localization cfg (Field only with click position)
        tab2 = QWidget()
        tab2_layout = QVBoxLayout(tab2)

        # Field widget for tab 2 with click enabled and larger scale
        self.field_widget_only = FieldWidget(enable_click=True, scale_factor=0.98)
        tab2_layout.addWidget(self.field_widget_only)

        # Position label and send button
        click_pos_layout = QHBoxLayout()
        click_pos_layout.addWidget(QLabel("Clicked Position (Field Coordinates):"))
        self.clicked_position_label = QLabel("--")
        self.clicked_position_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        click_pos_layout.addWidget(self.clicked_position_label)
        click_pos_layout.addStretch()

        # Send position button
        self.send_pos_button = QPushButton("Send Position")
        self.send_pos_button.clicked.connect(self.on_send_position_clicked)
        click_pos_layout.addWidget(self.send_pos_button)

        tab2_layout.addLayout(click_pos_layout)

        dock2 = QDockWidget("Localization cfg", self)
        dock2.setWidget(tab2)
        dock2.setFloating(False)
        # Enable movable and floatable features for both PyQt5 and PyQt6
        if PYQT_VERSION == 6:
            from PyQt6.QtWidgets import QDockWidget as QDockWidgetPyQt6
            dock2.setFeatures(QDockWidgetPyQt6.DockWidgetFeature.DockWidgetMovable | QDockWidgetPyQt6.DockWidgetFeature.DockWidgetFloatable)
        else:
            dock2.setFeatures(QDockWidget.AllDockWidgetFeatures)
        self.addDockWidget(Qt.DockWidgetArea.TopDockWidgetArea, dock2)

        # Connect click signal
        self.field_widget_only.position_clicked.connect(self.on_field_clicked)

        # Tab 3: Velocity Control
        tab3 = QWidget()
        tab3_layout = QVBoxLayout(tab3)

        # Publisher selection section
        publisher_group = QGroupBox("Publisher Selection")
        publisher_layout = QHBoxLayout()

        publisher_label = QLabel("Select Publisher:")
        self.publisher_combo = QComboBox()
        self.publisher_combo.addItems(
            [
                "Publisher 1 (/LocoApiTopicReq_1)",
                "Publisher 2 (/LocoApiTopicReq_2)",
                "Publisher 3 (/LocoApiTopicReq_3)",
                "Publisher 4 (/LocoApiTopicReq_4)",
                "Publisher 5 (/LocoApiTopicReq_5)",
                "Publisher 6 (/LocoApiTopicReq_6)",
                "Publisher Real Robot (/LocoApiTopicReq)",
            ]
        )
        self.publisher_combo.currentIndexChanged.connect(self.on_publisher_changed)

        publisher_layout.addWidget(publisher_label)
        publisher_layout.addWidget(self.publisher_combo)
        publisher_group.setLayout(publisher_layout)
        tab3_layout.addWidget(publisher_group)

        # Velocity settings section
        velocity_group = QGroupBox("Velocity Settings")
        velocity_layout = QGridLayout()

        # X velocity
        velocity_layout.addWidget(QLabel("X Velocity (m/s):"), 0, 0)
        self.x_input = QDoubleSpinBox()
        self.x_input.setMinimum(-5.0)
        self.x_input.setMaximum(5.0)
        self.x_input.setSingleStep(0.1)
        self.x_input.setDecimals(2)
        self.x_input.setValue(0.0)
        velocity_layout.addWidget(self.x_input, 0, 1)

        # Y velocity
        velocity_layout.addWidget(QLabel("Y Velocity (m/s):"), 1, 0)
        self.y_input = QDoubleSpinBox()
        self.y_input.setMinimum(-5.0)
        self.y_input.setMaximum(5.0)
        self.y_input.setSingleStep(0.1)
        self.y_input.setDecimals(2)
        self.y_input.setValue(0.0)
        velocity_layout.addWidget(self.y_input, 1, 1)

        # Theta velocity (angular velocity)
        velocity_layout.addWidget(QLabel("Theta Velocity (rad/s):"), 2, 0)
        self.theta_input = QDoubleSpinBox()
        self.theta_input.setMinimum(-3.14)
        self.theta_input.setMaximum(3.14)
        self.theta_input.setSingleStep(0.1)
        self.theta_input.setDecimals(2)
        self.theta_input.setValue(0.0)
        velocity_layout.addWidget(self.theta_input, 2, 1)

        velocity_group.setLayout(velocity_layout)
        tab3_layout.addWidget(velocity_group)

        # Control buttons
        button_layout = QHBoxLayout()

        self.send_velocity_button = QPushButton("Send Velocity")
        self.send_velocity_button.clicked.connect(self.send_velocity)

        self.stop_robot_button = QPushButton("Stop Robot")
        self.stop_robot_button.clicked.connect(self.stop_robot)

        self.clear_fields_button = QPushButton("Clear Fields")
        self.clear_fields_button.clicked.connect(self.clear_fields)

        button_layout.addWidget(self.send_velocity_button)
        button_layout.addWidget(self.stop_robot_button)
        button_layout.addWidget(self.clear_fields_button)

        tab3_layout.addLayout(button_layout)

        # Status label
        self.velocity_status_label = QLabel("Ready to send commands")
        tab3_layout.addWidget(self.velocity_status_label)

        # Additional Functions section
        additional_group = QGroupBox("Additional Functions")
        additional_layout = QVBoxLayout()

        self.no_gc_mode_button = QPushButton("NO_GC mode")
        self.no_gc_mode_button.clicked.connect(self.on_no_gc_mode_clicked)
        additional_layout.addWidget(self.no_gc_mode_button)

        self.stop_mode_button = QPushButton("STOP mode")
        self.stop_mode_button.clicked.connect(self.on_stop_mode_clicked)
        additional_layout.addWidget(self.stop_mode_button)

        self.soccer_game_mode_button = QPushButton("SOCCER_GAME mode")
        self.soccer_game_mode_button.clicked.connect(self.on_soccer_game_mode_clicked)
        additional_layout.addWidget(self.soccer_game_mode_button)

        additional_group.setLayout(additional_layout)
        tab3_layout.addWidget(additional_group)

        # Add stretch
        tab3_layout.addStretch()

        dock3 = QDockWidget("Velocity Control", self)
        dock3.setWidget(tab3)
        dock3.setFloating(False)
        # Enable movable and floatable features for both PyQt5 and PyQt6
        if PYQT_VERSION == 6:
            from PyQt6.QtWidgets import QDockWidget as QDockWidgetPyQt6
            dock3.setFeatures(QDockWidgetPyQt6.DockWidgetFeature.DockWidgetMovable | QDockWidgetPyQt6.DockWidgetFeature.DockWidgetFloatable)
        else:
            dock3.setFeatures(QDockWidget.AllDockWidgetFeatures)
        self.addDockWidget(Qt.DockWidgetArea.TopDockWidgetArea, dock3)


        # Tab 4: Decision Log
        tab4 = QWidget()
        tab4_layout = QVBoxLayout(tab4)

        # Decision log diagram view
        self.decision_diagram_view = DiagramView()
        tab4_layout.addWidget(self.decision_diagram_view)

        dock4 = QDockWidget("Decision Log", self)
        dock4.setWidget(tab4)
        dock4.setFloating(False)
        # Enable movable and floatable features for both PyQt5 and PyQt6
        if PYQT_VERSION == 6:
            from PyQt6.QtWidgets import QDockWidget as QDockWidgetPyQt6
            dock4.setFeatures(QDockWidgetPyQt6.DockWidgetFeature.DockWidgetMovable | QDockWidgetPyQt6.DockWidgetFeature.DockWidgetFloatable)
        else:
            dock4.setFeatures(QDockWidget.AllDockWidgetFeatures)
        self.addDockWidget(Qt.DockWidgetArea.TopDockWidgetArea, dock4)

        # Tabify all dock widgets (initial state)
        self.tabifyDockWidget(dock1, dock2)
        self.tabifyDockWidget(dock2, dock3)
        self.tabifyDockWidget(dock3, dock4)

        # Set dock1 as the active tab initially
        self.setTabPosition(Qt.DockWidgetArea.TopDockWidgetArea if PYQT_VERSION == 6 else Qt.TopDockWidgetArea,
                           QTabWidget.TabPosition.North if PYQT_VERSION == 6 else QTabWidget.North)
        dock1.raise_()

        # Store current publisher index
        self.current_publisher_index = 0

        # ROS2 thread
        self.ros_thread = ROS2Thread(sim_robot_id)
        self.ros_thread.strategy_state_received.connect(self.update_display)
        self.ros_thread.target_position_received.connect(self.update_target_position)
        self.ros_thread.particle_cloud_received.connect(self.update_particle_cloud)
        self.ros_thread.white_line_particles_received.connect(self.update_white_line_particles)
        self.ros_thread.localization_result_received.connect(self.update_localization_result)
        self.ros_thread.detections_received.connect(self.update_detections)
        self.ros_thread.decision_log_received.connect(self.update_decision_log)
        self.ros_thread.start()

        # Timer for clearing target position display after 3 seconds
        self.target_display_timer = QTimer()
        self.target_display_timer.timeout.connect(self.clear_target_display)
        self.target_display_timer.setSingleShot(True)

        # Store particle cloud, white line particles, localization result, and detections
        self.current_particle_cloud = None
        self.current_white_line_particles = None
        self.current_localization_result = None
        self.current_detections = None

    def update_display(self, msg):
        # Update status labels
        self.labels["ball_position_relative"].setText(msg.ball_position_relative)
        # Update shared ball global label
        if msg.shared_ball_global and hasattr(msg.shared_ball_global, 'x') and hasattr(msg.shared_ball_global, 'y'):
            self.labels["shared_ball_global"].setText(
                f"x={msg.shared_ball_global.x:.2f}, y={msg.shared_ball_global.y:.2f}"
            )
        else:
            self.labels["shared_ball_global"].setText("--")
        
        self.labels["current_robot_field"].setText(msg.current_robot_field)
        self.labels["game_state"].setText(msg.game_state)
        self.labels["secondary_state"].setText(msg.secondary_state)
        self.labels["penalty"].setText(msg.penalty)
        self.labels["our_field"].setText(msg.our_field)
        self.labels["team_color"].setText(msg.team_color)
        self.labels["role"].setText(msg.role)
        self.labels["robot_mode"].setText(msg.robot_mode)
        self.labels["neck_yaw_rad"].setText(f"{msg.neck_yaw_rad:.3f}")

        # Update position text
        position_text = []

        if msg.self_position_global:
            position_text.append(
                f"Self Position (Global): x={msg.self_position_global.x:.2f}, "
                f"y={msg.self_position_global.y:.2f}, "
                f"theta={msg.self_position_global.theta:.2f}, "
                f"coord={msg.self_position_global.coord}"
            )

        if msg.ball_position_local:
            position_text.append(
                f"Ball Position (Local): x={msg.ball_position_local.x:.2f}, "
                f"y={msg.ball_position_local.y:.2f}, "
                f"coord={msg.ball_position_local.coord}"
            )

        if msg.shared_ball_global:
            position_text.append(
                f"Shared Ball (Global): x={msg.shared_ball_global.x:.2f}, "
                f"y={msg.shared_ball_global.y:.2f}, "
                f"coord={msg.shared_ball_global.coord}"
            )

        position_text.append(f"\nOpponents ({len(msg.opponent_position_local)}):")
        for i, opp in enumerate(msg.opponent_position_local):
            position_text.append(f"  {i + 1}: x={opp.x:.2f}, y={opp.y:.2f}")

        position_text.append(f"\nAllies ({len(msg.allies_position_local)}):")
        for i, ally in enumerate(msg.allies_position_local):
            position_text.append(f"  {i + 1}: x={ally.x:.2f}, y={ally.y:.2f}")

        position_text.append(
            f"\nOpponent Goalposts Local ({len(msg.opponent_goalpost_local)}):"
        )
        for i, post in enumerate(msg.opponent_goalpost_local):
            position_text.append(f"  {i + 1}: x={post.x:.2f}, y={post.y:.2f}")

        position_text.append(
            f"\nOpponent Goalposts Global ({len(msg.opponent_goalpost_global)}):"
        )
        for i, post in enumerate(msg.opponent_goalpost_global):
            position_text.append(f"  {i + 1}: x={post.x:.2f}, y={post.y:.2f}")

        position_text.append(
            f"\nNormal Obstacles Local ({len(msg.normal_obstacles_position_local)}):"
        )
        for i, obstacle in enumerate(msg.normal_obstacles_position_local):
            position_text.append(f"  {i + 1}: x={obstacle.x:.2f}, y={obstacle.y:.2f}")

        self.position_text.setText("\n".join(position_text))

        # Update field visualization (Full View tab - without particle cloud)
        self.field_widget.update_positions(
            msg.self_position_global,
            msg.ball_position_local,
            msg.shared_ball_global,
            msg.ball_detected_by_camera,
            msg.opponent_position_local,
            msg.allies_position_local,
            msg.opponent_goalpost_local,
            msg.opponent_goalpost_global,
            msg.normal_obstacles_position_local,
        )
        # Localization cfg tab: only show robot position from localization_result, particle cloud, and white line particles
        # If localization_result is available, use it; otherwise, use self_position_global from StrategyState
        robot_pos_for_loc_tab = None
        if self.current_localization_result and hasattr(self.current_localization_result, 'pose'):
            # Create ObjectPosition-like object from LocalizationResult
            robot_pos_for_loc_tab = ObjectPosition()
            robot_pos_for_loc_tab.x = self.current_localization_result.pose.x * 100.0  # Convert from meters to centimeters
            robot_pos_for_loc_tab.y = self.current_localization_result.pose.y * 100.0  # Convert from meters to centimeters
            robot_pos_for_loc_tab.theta = self.current_localization_result.pose.theta
            robot_pos_for_loc_tab.coord = "global"  # Global coordinate
        else:
            # Fallback to StrategyState's self_position_global
            robot_pos_for_loc_tab = msg.self_position_global

        self.field_widget_only.update_positions(
            robot_pos_for_loc_tab,
            None,  # ball_position_local
            None,  # shared_ball_global
            None,  # ball_detected_by_camera
            [],    # opponent_position_local
            [],    # allies_position_local
            [],    # opponent_goalpost_local
            [],    # opponent_goalpost_global
            [],    # normal_obstacles_position_local
            particle_cloud=self.current_particle_cloud,
            white_line_particles=self.current_white_line_particles,
            detected_line_marks=self.current_detections,
        )

    def update_decision_log(self, msg):
        """Update decision log diagram"""
        self.decision_diagram_view.clear_decision_items()
        for decision in msg.decisions:
            self.decision_diagram_view.add_decision_item(decision)

    def update_target_position(self, msg):
        """Update target position display"""
        # Only update if we receive a valid target position
        if msg and hasattr(msg, "x") and hasattr(msg, "y") and hasattr(msg, "theta"):
            # Update status label
            self.labels["target_position"].setText(
                f"x={msg.x:.2f}, y={msg.y:.2f}, θ={msg.theta:.2f}"
            )

            # Restart the timer for 3 seconds
            self.target_display_timer.stop()
            self.target_display_timer.start(3000)  # 3000 milliseconds = 3 seconds

            # Update field visualization (only full view tab)
            self.field_widget.update_target_position(msg)
        # If msg is None or invalid, keep displaying the previous value
        # The timer will continue counting down from its last start

    def clear_target_display(self):
        """Clear target position display after timeout"""
        self.labels["target_position"].setText("--")
        # Field widget will clear its own target position via its timer

    def update_particle_cloud(self, msg):
        """Update particle cloud display"""
        self.current_particle_cloud = msg
        # Update Localization cfg tab to display particle cloud immediately
        self.field_widget_only.particle_cloud = msg
        self.field_widget_only.update()

    def update_white_line_particles(self, msg):
        """Update white line particles display"""
        self.current_white_line_particles = msg
        # Update Localization cfg tab to display white line particles immediately
        self.field_widget_only.white_line_particles = msg
        self.field_widget_only.update()

    def update_localization_result(self, msg):
        """Update localization result for Localization cfg tab"""
        self.current_localization_result = msg
        # Update Localization cfg tab to display robot position from localization_result
        if msg and hasattr(msg, 'pose'):
            # Create ObjectPosition-like object from LocalizationResult
            # LocalizationResult.pose has x, y, theta in meters
            # ObjectPosition has x, y in centimeters and theta in radians
            robot_pos = ObjectPosition()
            robot_pos.x = msg.pose.x * 100.0  # Convert from meters to centimeters
            robot_pos.y = msg.pose.y * 100.0  # Convert from meters to centimeters
            robot_pos.theta = msg.pose.theta
            robot_pos.coord = "global"  # Global coordinate

            # Update only the Localization cfg tab
            self.field_widget_only.update_positions(
                robot_pos,
                None,  # ball_position_local
                None,  # shared_ball_global
                None,  # ball_detected_by_camera
                [],    # opponent_position_local
                [],    # allies_position_local
                [],    # opponent_goalpost_local
                [],    # opponent_goalpost_global
                [],    # normal_obstacles_position_local
                particle_cloud=self.current_particle_cloud,
                white_line_particles=self.current_white_line_particles,
                detected_line_marks=self.current_detections,
            )

    def update_detections(self, msg):
        """Update detected line marks for Localization cfg tab"""
        # Filter detections to only include line marks
        self.line_mark_labels = {'TCross', 'LCross', 'XCross', 'PenaltyPoint'}
        detected_marks = []

        if msg and hasattr(msg, 'detected_objects'):
            for obj in msg.detected_objects:
                if obj.label in self.line_mark_labels and len(obj.position_projection) >= 2:
                    detected_marks.append({
                        'label': obj.label,
                        'position': obj.position_projection,
                    })

        self.current_detections = detected_marks

        # Update Localization cfg tab immediately
        self.field_widget_only.detected_line_marks = detected_marks
        self.field_widget_only.update()

    def on_field_clicked(self, x, y, theta):
        """Handle field click event"""
        theta_deg = math.degrees(theta)
        self.clicked_position_label.setText(f"x = {x:.2f} m, y = {y:.2f} m, θ = {theta_deg:.1f}°")

    def on_send_position_clicked(self):
        """Handle send position button click"""
        if self.field_widget_only.clicked_position:
            x, y, theta = self.field_widget_only.clicked_position
            self.ros_thread.send_request(x, y, theta)

    def on_publisher_changed(self, index):
        """Handle publisher combo box change"""
        self.current_publisher_index = index
        self.velocity_status_label.setText(f"Selected Publisher {index + 1}")

    def send_velocity(self):
        """Send velocity command"""
        try:
            x_vel = self.x_input.value()
            y_vel = self.y_input.value()
            theta_vel = self.theta_input.value()

            command_msg = self.ros_thread.create_msg(x_vel, y_vel, theta_vel)

            publisher = self.ros_thread.get_publisher(self.current_publisher_index)
            publisher.publish(command_msg)

            self.velocity_status_label.setText(
                f"Sent: x={x_vel:.2f}, y={y_vel:.2f}, theta={theta_vel:.2f} "
                f"to Publisher {self.current_publisher_index + 1}"
            )

        except ValueError:
            self.velocity_status_label.setText("Error: Invalid input values")

    def stop_robot(self):
        """Stop robot (set all velocities to 0)"""
        stop_msg = self.ros_thread.create_msg(0.0, 0.0, 0.0)

        publisher = self.ros_thread.get_publisher(self.current_publisher_index)
        publisher.publish(stop_msg)

        # Update UI
        self.x_input.setValue(0.0)
        self.y_input.setValue(0.0)
        self.theta_input.setValue(0.0)

        self.velocity_status_label.setText(
            f"Stop command sent to Publisher {self.current_publisher_index + 1}"
        )

    def clear_fields(self):
        """Clear input fields"""
        self.x_input.setValue(0.0)
        self.y_input.setValue(0.0)
        self.theta_input.setValue(0.0)
        self.velocity_status_label.setText("Fields cleared")

    def on_no_gc_mode_clicked(self):
        """Handle NO_GC mode button click"""
        msg = RemoteControllerState()
        msg.rb = True
        msg.b = True
        self.ros_thread.publisher_remote_controller.publish(msg)
        print("NO_GC mode activated")

    def on_stop_mode_clicked(self):
        """Handle STOP mode button click"""
        msg = RemoteControllerState()
        msg.rb = True
        msg.a = True
        self.ros_thread.publisher_remote_controller.publish(msg)
        print("STOP mode activated")

    def on_soccer_game_mode_clicked(self):
        """Handle SOCCER_GAME mode button click"""
        msg = RemoteControllerState()
        msg.rb = True
        msg.x = True
        self.ros_thread.publisher_remote_controller.publish(msg)
        print("SOCCER_GAME mode activated")

    def closeEvent(self, event):
        self.ros_thread.stop()
        self.ros_thread.wait()
        event.accept()


def main():
    parser = argparse.ArgumentParser(description="Strategy State Visualizer")
    parser.add_argument(
        "--sim-robot-id",
        type=int,
        default=None,
        help="Simulation robot ID (optional, for simulation mode)",
    )
    args = parser.parse_args()

    app = QApplication(sys.argv)
    visualizer = StrategyStateVisualizer(args.sim_robot_id)
    visualizer.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
