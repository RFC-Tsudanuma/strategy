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


import os
import sys

# Add parent directory to path for importing modules
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import socket
import struct
import threading
import time

from constants import ROLE
from setting import DEFAULT_STRATEGY_CONFIG, STRATEGY_SHARED_CONFIG
from state import Position, SharedMessage

from .teammate_communication import TeammateCommunication, convert_role_to_pb_role

# Constants
GAMECONTROLLER_DATA_PORT = 3838
GAMECONTROLLER_RETURN_PORT = 3939

GAMECONTROLLER_STRUCT_HEADER = b"RGme"
GAMECONTROLLER_STRUCT_VERSION = 12

GAMECONTROLLER_RETURN_STRUCT_HEADER = b"RGrt"
GAMECONTROLLER_RETURN_STRUCT_VERSION = 2

GAMECONTROLLER_RETURN_MSG_MAN_PENALISE = 0
GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE = 1
GAMECONTROLLER_RETURN_MSG_ALIVE = 2

# Intervals in milliseconds
BROADCAST_GAME_CONTROL_INTERVAL_MS = 300

BROADCAST_INFOSHARE_INTERVAL_S = 0.2


class RoboCupGameControlReturnData:
    def __init__(self):
        self.header = GAMECONTROLLER_RETURN_STRUCT_HEADER
        self.version = GAMECONTROLLER_RETURN_STRUCT_VERSION
        self.team = 0
        self.player = 0
        self.message = GAMECONTROLLER_RETURN_MSG_ALIVE

    def pack(self) -> bytes:
        return struct.pack(
            "4sBBBB", self.header, self.version, self.team, self.player, self.message
        )


class BrainCommunication:
    def __init__(self,is_simulation: bool = False):
        # Sockets
        self._gc_send_socket = -1

        # Threads
        self._gamecontrol_broadcast_thread = None

        # Flags
        self._broadcast_gamecontrol_flag = False

        # GameController return data
        self.gc_return_data = RoboCupGameControlReturnData()

        # Teammate communication handler
        self._teammate_communication = None

        self.config = DEFAULT_STRATEGY_CONFIG
        self.is_simulation = is_simulation

    def __del__(self):
        try:
            self.cleanup()
        except (KeyboardInterrupt, Exception):
            pass

    def cleanup(self):
        try:
            self.clearup_game_controller_broadcast()
        except (KeyboardInterrupt, Exception):
            pass

        if self._teammate_communication:
            try:
                self._teammate_communication.cleanup()
            except (KeyboardInterrupt, Exception):
                pass

    def init_udp_broadcast(self):
        enable_com = self.config["common_config"]["enable_team_communication"]
        if enable_com:
            print("\033[31m<GC> InfoShare enabled.\033[0m")

            # Initialize teammate communication
            self._teammate_communication = TeammateCommunication(
                player_id=self.config["common_config"]["robot_id"]
            )
            self._teammate_communication.init()
        else:
            print("\033[31m<GC> InfoShare disabled.\033[0m")

        self.init_game_controller_broadcast()

    def init_game_controller_broadcast(self):
        self._gc_send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Enable broadcast
        self._gc_send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._gc_send_socket.settimeout(0.5)

        # Target address
        if self.is_simulation:
            # simの場合は単なるブロードキャスト
            self._gcsaddr = (STRATEGY_SHARED_CONFIG["sim_game_controller_address"], GAMECONTROLLER_RETURN_PORT)
            print(f"\033[32m<GC> Broadcasting to SIM GameController at {self._gcsaddr[0]}:{GAMECONTROLLER_RETURN_PORT}\033[0m")
        else:
            # こちらは実際のゲームコントローラーのアドレス
            self._gcsaddr = (STRATEGY_SHARED_CONFIG["game_controller_address"], GAMECONTROLLER_RETURN_PORT)
            print(f"\033[32m<GC> Broadcasting to GameController at {self._gcsaddr[0]}:{GAMECONTROLLER_RETURN_PORT}\033[0m")

        self._broadcast_gamecontrol_flag = True
        self._gamecontrol_broadcast_thread = threading.Thread(
            target=self.broadcast_to_game_controller
        )
        self._gamecontrol_broadcast_thread.daemon = True
        self._gamecontrol_broadcast_thread.start()

    def clearup_game_controller_broadcast(self):
        self._broadcast_gamecontrol_flag = False

        # スレッドの終了を待つ（タイムアウト付き）
        if (
            self._gamecontrol_broadcast_thread
            and self._gamecontrol_broadcast_thread.is_alive()
        ):
            self._gamecontrol_broadcast_thread.join(timeout=2.0)

        # ソケットを閉じる
        if self._gc_send_socket != -1:
            try:
                self._gc_send_socket.close()
            except:
                pass
            self._gc_send_socket = -1
            print("\033[31m<GC> GameControl send socket has been closed.\033[0m")

    def broadcast_to_game_controller(self):
        while self._broadcast_gamecontrol_flag:
            self.gc_return_data.team = self.config["common_config"]["team_id"]
            self.gc_return_data.player = self.config["common_config"][
                "robot_id"
            ]  # player number starts with 1
            self.gc_return_data.message = GAMECONTROLLER_RETURN_MSG_ALIVE

            try:
                data = self.gc_return_data.pack()
                self._gc_send_socket.sendto(data, self._gcsaddr)
            except Exception as e:
                print(f"\033[31m<GC> gc sendto failed: {e}\033[0m")

            time.sleep(BROADCAST_GAME_CONTROL_INTERVAL_MS / 1000.0)

    def get_teammate_addresses(self):
        """Get current teammate addresses from teammate communication handler"""
        if self._teammate_communication:
            return self._teammate_communication.get_teammate_addresses()
        return {}

    def send_infoshare(self, ballpos: Position, selfpos: Position, selfpos_local: Position, role: ROLE, strategy_hz: float):
        if self._teammate_communication:
            self._teammate_communication.send_infoshare(
                ballpos, selfpos, convert_role_to_pb_role(role),
                selfpos_local,strategy_hz=strategy_hz
            )

    def get_latest_shared_msg(self)-> list[SharedMessage]:
        if self._teammate_communication:
            return self._teammate_communication.get_latest_msg()
        return []
