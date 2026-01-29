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

sys.path.append("../")
import gtpyhop

domain_name = "test_htn"
gtpyhop.current_domain = gtpyhop.Domain(domain_name)


world = gtpyhop.State("world")
world.states = {
    "ball_position_relative": ["in_kick_range", "distant"],
    "role": ["attacker", "defender", "neutral"],
}

initial_state = gtpyhop.State("world")
initial_state.ball_position_relative = "in_kick_range"
initial_state.role = "defender"

####################################################
# helper functions


def is_a(variable, type):
    return variable in world.states[type]


#####################################################
# actions


def kick_ball(s, pos_x, pos_y):
    """
    If the ball is in kick range, then kick it.
    """
    if (
        s.ball_position_relative == "in_kick_range"
        and isinstance(pos_x, (int, float))
        and isinstance(pos_y, (int, float))
    ):
        s.ball_position_relative = "distant"
        print(f"Kicking the ball to position ({pos_x}, {pos_y})")
        return s


def approach_ball(s, pos_x=None, pos_y=None):
    """
    If the ball is distant, then approach it.
    """
    if (
        s.ball_position_relative == "distant"
        and isinstance(pos_x, (int, float))
        and isinstance(pos_y, (int, float))
    ):
        s.ball_position_relative = "in_kick_range"
        print(f"Approaching the ball to position ({pos_x}, {pos_y})")
        return s


def change_role(s, new_role):
    """
    Change the role to a new role.
    """
    if is_a(new_role, "role"):
        s.role = new_role
        print(f"Changed role to {new_role}")
        return s
    else:
        return False


gtpyhop.declare_actions(kick_ball, approach_ball, change_role)


####################################################
# methods
def defence_method(s):
    """
    If the role is defender, then approach the ball if needed and kick it.
    """
    if s.role == "defender":
        app_x: float = 765
        app_y: float = 765
        kick_x: float = 1000
        kick_y: float = 1000
        if s.ball_position_relative == "distant":
            return [("approach_ball", app_x, app_y), ("kick_ball", kick_x, kick_y)]
        elif s.ball_position_relative == "in_kick_range":
            return [("kick_ball", kick_x, kick_y)]


def attack_method(s):
    """
    If the role is attacker, then approach the ball if needed and kick it.
    """
    if s.role == "attacker":
        kick_x: float = 800
        kick_y: float = 800
        app_x: float = 283
        app_y: float = 283
        if s.ball_position_relative == "distant":
            return [("approach_ball", app_x, app_y), ("kick_ball", kick_x, kick_y)]
        elif s.ball_position_relative == "in_kick_range":
            return [("kick_ball", kick_x, kick_y)]


gtpyhop.declare_task_methods("defence_kick", defence_method)
gtpyhop.declare_task_methods("attack_kick", attack_method)


def select_kick_method(s):
    """
    Select appropriate kick based on role
    """
    if s.role == "defender":
        return [("defence_kick",)]
    elif s.role == "attacker":
        return [("attack_kick",)]
    else:
        return False


gtpyhop.declare_task_methods("select_kick", select_kick_method)


def main(do_pauses=True):
    """
    Run various examples.
    main() will pause occasionally to let you examine the output.
    main(False) will run straight through to the end, without stopping.
    """

    # If we've changed to some other domain, this will change us back.

    gtpyhop.print_domain()

    initial_state.display("\nInitial state is")
    gtpyhop.verbose = 3
    plan = gtpyhop.find_plan(initial_state, [("select_kick",)])

    if plan:
        print("Plan found:")
        for step in plan:
            print(step)
    else:
        print("No plan found.")


if __name__ == "__main__":
    main(do_pauses=True)
