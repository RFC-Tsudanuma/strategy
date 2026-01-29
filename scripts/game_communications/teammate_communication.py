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


import socket
import sys
import threading
from typing import Optional

# Import generated protobuf classes
from . import teammate_communication_pb2 as pb

sys.path.append("../")
from state import ROLE, Position, SharedMessage
from setting import STRATEGY_SHARED_CONFIG

# Constants
VALIDATION_COMMUNICATION = 52149

# Intervals in milliseconds
BROADCAST_INTERVAL_MS = 100

# Broadcast configuration
BROADCAST_PORT = 52149


def convert_to_pb_position(pos: Optional[Position]) -> pb.GlobalPosition:
    if pos is None:
        return pb.GlobalPosition(valid=False)
    if pos.coord == Position.COORD_GLOBAL:
        return pb.GlobalPosition(x=pos.x, y=pos.y, yaw=pos.theta, valid=True)
    else:
        raise ValueError("Position must be in global coordinates for communication.")

def convert_to_pb_position_local(pos: Optional[Position]) -> pb.LocalPosition:
    if pos is None:
        return pb.LocalPosition(valid=False)
    if pos.coord == Position.COORD_LOCAL:
        return pb.LocalPosition(x=pos.x, y=pos.y, yaw=pos.theta, valid=True)
    else:
        raise ValueError("Position must be in local coordinates for communication.")

def convert_pb_position_to_position(pb_pos: pb.GlobalPosition) -> Position:
    if pb_pos.valid:
        return Position(Position.COORD_GLOBAL, pb_pos.x, pb_pos.y, pb_pos.yaw)
    else:
        return None

def convert_pb_position_to_position_local(pb_pos: pb.LocalPosition) -> Position:
    if pb_pos.valid:
        return Position(Position.COORD_LOCAL, pb_pos.x, pb_pos.y, pb_pos.yaw)
    else:
        return None

def convert_role_to_pb_role(role: ROLE) -> pb.Role:
    if role == ROLE.KEEPER:
        return pb.Role.KEEPER
    elif role == ROLE.ATTACKER:
        return pb.Role.ATTACKER
    elif role == ROLE.DEFENDER:
        return pb.Role.DEFENDER
    elif role == ROLE.NEUTRAL:
        return pb.Role.NEUTRAL
    else:
        raise ValueError("convert_role_to_pb_role():: This can't happen!")


def convert_pb_role_to_role(pb_role: pb.Role) -> ROLE:
    if pb_role == pb.Role.KEEPER:
        return ROLE.KEEPER
    elif pb_role == pb.Role.ATTACKER:
        return ROLE.ATTACKER
    elif pb_role == pb.Role.DEFENDER:
        return ROLE.DEFENDER
    elif pb_role == pb.Role.NEUTRAL:
        return ROLE.NEUTRAL
    else:
        raise ValueError("convert_pb_role_to_role():: This can't happen!")


class TeammateCommunication:
    def __init__(
        self, player_id: int
    ):
        self.player_id = player_id
        self.broadcast_address = STRATEGY_SHARED_CONFIG["infoshare_broadcast_address"]

        # Sockets
        self._broadcast_socket: Optional[socket.socket] = None
        self._communication_recv_socket: Optional[socket.socket] = None

        # Threads
        self._communication_recv_thread = None

        # Flags
        self._receive_communication_flag = False

        # Message IDs
        self._team_communication_msg_id = 0

        self.latest_msg_lock = threading.Lock()
        self.latest_msg = [None, None, None]  # 3人なので

    def __del__(self):
        self.cleanup()

    def cleanup(self):
        self.cleanup_communication_broadcast()
        self.cleanup_communication_receiver()

    def init(self):
        """Initialize all communication components"""
        self.init_communication_broadcast()
        self.init_communication_receiver()

    def init_communication_broadcast(self):
        self._broadcast_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Enable broadcast
        self._broadcast_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._broadcast_socket.settimeout(0.5)

        # Configure broadcast address
        self._broadcast_addr = (self.broadcast_address, BROADCAST_PORT)

        print(
            f"\033[32m<InfoShare> Broadcasting to subnet {self.broadcast_address}:{BROADCAST_PORT}\033[0m"
        )

    def cleanup_communication_broadcast(self):
        if self._broadcast_socket:
            try:
                self._broadcast_socket.close()
            except Exception:
                pass
            self._broadcast_socket = None
            print("\033[31m<InfoShare> Communication broadcast socket has been closed.\033[0m")

    def send_infoshare(
        self, ballpos: Optional[Position], selfpos: Optional[Position], role: pb.Role, ballpos_local: Optional[Position],
        strategy_hz: float
    ):
        # Create protobuf message
        msg = pb.TeamCommunicationMsg()
        msg.validation = VALIDATION_COMMUNICATION
        msg.robot_id = self.player_id
        msg.ballpos.CopyFrom(convert_to_pb_position(ballpos))
        msg.selfpos.CopyFrom(convert_to_pb_position(selfpos))
        msg.ballpos_local.CopyFrom(convert_to_pb_position_local(ballpos_local))
        msg.role = role
        msg.strategy_hz = strategy_hz

        self._team_communication_msg_id += 1

        try:
            # Serialize and broadcast
            data = msg.SerializeToString()

            # Non-blocking send using the socket set to non-blocking mode
            if self._broadcast_socket:
                try:
                    self._broadcast_socket.sendto(data, self._broadcast_addr)
                except (socket.error, BlockingIOError) as e:
                    # Non-blocking socket might raise BlockingIOError if buffer is full
                    # In this case, we just skip this message
                    pass
        except Exception as e:
            print(f"\033[31m<InfoShare> Broadcast failed: {e}\033[0m")


    def init_communication_receiver(self):
        self._communication_recv_socket = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM
        )

        # Allow address reuse
        self._communication_recv_socket.setsockopt(
            socket.SOL_SOCKET, socket.SO_REUSEADDR, 1
        )

        # Set socket timeout to prevent blocking indefinitely
        self._communication_recv_socket.settimeout(1.0)

        # Bind to port
        addr = ("", BROADCAST_PORT)
        self._communication_recv_socket.bind(addr)

        print(f"\033[32m<InfoShare> Listening for broadcasts on port {BROADCAST_PORT}\033[0m")

        self._receive_communication_flag = True
        self._communication_recv_thread = threading.Thread(
            target=self.spin_communication_receiver
        )
        self._communication_recv_thread.daemon = True
        self._communication_recv_thread.start()

    def cleanup_communication_receiver(self):
        self._receive_communication_flag = False
        if self._communication_recv_socket:
            try:
                self._communication_recv_socket.close()
            except Exception:
                pass
            self._communication_recv_socket = None
            print("\033[31m<InfoShare> Communication receive socket has been closed.\033[0m")

        if (
            self._communication_recv_thread
            and self._communication_recv_thread.is_alive()
        ):
            self._communication_recv_thread.join(timeout=2.0)
            if self._communication_recv_thread.is_alive():
                print("\033[31m<InfoShare> Warning: Communication receiver thread did not finish in time\033[0m")

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

                if msg.robot_id == self.player_id:
                    # This is our own broadcast, ignore it
                    continue

                with self.latest_msg_lock:
                    self.latest_msg[msg.robot_id - 1] = msg

            except socket.timeout:
                # Timeout is expected, just continue checking the flag
                continue
            except Exception as e:
                if self._receive_communication_flag:
                    print(f"\033[31m<InfoShare> Receiving broadcast message failed: {e}\033[0m")

    def get_latest_msg(self) -> list[SharedMessage]:
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
