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


import sys
import termios
import tty

import rclpy
from booster_interface.msg import RemoteControllerState
from rclpy.node import Node


class KeyboardInputNode(Node):
    def __init__(self):
        super().__init__("keyboard_input_node")

        # Create publisher
        self.publisher = self.create_publisher(
            RemoteControllerState, "/remote_controller_state", 10
        )

        # Print instructions
        print("\n=== Keyboard Input Node ===")
        print("Keyboard mapping to controller buttons:")
        print("  Movement: WASD -> Left stick")
        print("  Camera: Arrow keys -> Right stick")
        print("  Buttons: Space->A, F->B, E->X, R->Y")
        print("  Shoulder: U->LB, I->RB, O->LT, P->RT")
        print("  System: TAB->Back, Enter->Start")
        print("  Note: Other keys trigger default controller state")
        print("  Quit: q or ESC")
        print("=" * 40)

    def get_key(self):
        """Get a single keypress"""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)

            # Handle arrow keys (3-byte sequences)
            if key == "\x1b":
                key += sys.stdin.read(2)
                if key == "\x1b[A":
                    return "UP"
                elif key == "\x1b[B":
                    return "DOWN"
                elif key == "\x1b[C":
                    return "RIGHT"
                elif key == "\x1b[D":
                    return "LEFT"

            return key
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def create_controller_state(self, key):
        """Create RemoteControllerState from keyboard input"""
        msg = RemoteControllerState()

        # Initialize all fields
        msg.event = 0  # SDL_EventType (0 for keyboard simulation)
        msg.lx = 0.0
        msg.ly = 0.0
        msg.rx = 0.0
        msg.ry = 0.0
        msg.a = False
        msg.b = False
        msg.x = False
        msg.y = False
        msg.lb = False
        msg.rb = False
        msg.lt = False
        msg.rt = False
        msg.ls = False
        msg.rs = False
        msg.back = False
        msg.start = False
        msg.hat_c = True  # Default: centered
        msg.hat_u = False
        msg.hat_d = False
        msg.hat_l = False
        msg.hat_r = False
        msg.hat_lu = False
        msg.hat_ld = False
        msg.hat_ru = False
        msg.hat_rd = False
        msg.hat_pos = 0

        # Face buttons
        if key == "a":
            msg.a = True
        elif key == "b":
            msg.b = True
        elif key == "x":
            msg.x = True
        elif key == "y":
            msg.y = True

        # Shoulder buttons
        elif key == "u":
            msg.lb = True
        elif key == "i":
            msg.rb = True
        elif key == "o":
            msg.lt = True
        elif key == "p":
            msg.rt = True

        # System buttons
        elif key == "\t":  # TAB
            msg.back = True
        elif key == "\r":  # Enter
            msg.start = True
        return msg

    def run(self):
        """Main loop"""
        while rclpy.ok():
            key = self.get_key()

            # Check for quit
            if key == "q" or key == "\x1b":  # q or ESC
                print("\nExiting...")
                break

            # Create and publish controller state
            msg = self.create_controller_state(key)
            self.publisher.publish(msg)

            # Debug output
            if msg.lx != 0 or msg.ly != 0:
                print(f"Left stick: ({msg.lx:.1f}, {msg.ly:.1f})")
            elif msg.rx != 0 or msg.ry != 0:
                print(f"Right stick: ({msg.rx:.1f}, {msg.ry:.1f})")
            elif any([msg.a, msg.b, msg.x, msg.y]):
                buttons = []
                if msg.a:
                    buttons.append("A")
                if msg.b:
                    buttons.append("B")
                if msg.x:
                    buttons.append("X")
                if msg.y:
                    buttons.append("Y")
                print(f"Buttons: {', '.join(buttons)}")
            elif any([msg.lb, msg.rb, msg.lt, msg.rt]):
                shoulders = []
                if msg.lb:
                    shoulders.append("LB")
                if msg.rb:
                    shoulders.append("RB")
                if msg.lt:
                    shoulders.append("LT")
                if msg.rt:
                    shoulders.append("RT")
                print(f"Shoulders: {', '.join(shoulders)}")
            elif not msg.hat_c:
                print(f"D-Pad: position {msg.hat_pos}")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = KeyboardInputNode()
        node.run()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt")
    finally:
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
