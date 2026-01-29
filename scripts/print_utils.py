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

STRATEGY_PREFIX = "[ Strategy ]"
REAL_ACTION_PREFIX = "\033[34m<real_action>\033[0m"
TASK_METHOD_PREFIX = "\033[32m<task_method>\033[0m"
NOTICE_PREFIX = "\034[33m"
DEBUG_PREFIX = "\033[35m"
RESET_SUFFIX = "\033[0m"


def print_in_real_action(action_name: str, message: str):
    """
    実アクションプリフィックス付きの出力
    """
    sys.stdout.write(f"{STRATEGY_PREFIX} {REAL_ACTION_PREFIX}::{action_name}() - {message}\n")

def print_in_task_method(task_name: str, message: str):
    """
    タスクメソッドプリフィックス付きの出力
    """
    sys.stdout.write(f"{STRATEGY_PREFIX} {TASK_METHOD_PREFIX}::{task_name}() {message}\n")

def print_in_strategy(message: str):
    """
    戦略プリフィックス付きの出力
    """
    sys.stdout.write(f"{STRATEGY_PREFIX} {message}\n")

def print_without_prefix(message: str):
    """
    プリフィックス無しでの普通の出力
    """
    sys.stdout.write(f"{message}\n")

def wrapp_notice(message: str) -> str:
    """
    Wraps the message with notice prefix and suffix.
    """
    return f"{NOTICE_PREFIX}{message}{RESET_SUFFIX}"

def debug_print(message: str):
    """
    Prints debug messages to standard output.
    """
    sys.stdout.write(f"{DEBUG_PREFIX}[DEBUG] {message}{RESET_SUFFIX}\n")