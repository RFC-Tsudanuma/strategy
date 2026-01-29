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

import inspect
from futbol_msgs.msg import StrategyDecision, DecisionLog


class DecisionLogger():
    def __init__(self):
        self.decision_log = DecisionLog()

    def get_decision_log(self):
        return self.decision_log
    
    def clear_decision_log(self):
        self.decision_log = DecisionLog()

    def action_precondition_not_satisfied(self, details=""):
        decision = StrategyDecision()
        decision.decision_result = StrategyDecision.DECISION_RESULT_PRECONDITION_FAILED
        decision.decision_type = StrategyDecision.DECISION_TYPE_ACTION
        decision.decision_details = details
        try:
            frame = inspect.currentframe()
            caller_frame = frame.f_back
            decision.decision_name = caller_frame.f_code.co_name
        finally:
            del frame
        self.decision_log.decisions.append(decision)

    def action_executed(self, details=""):
        decision = StrategyDecision()
        decision.decision_result = StrategyDecision.DECISION_RESULT_EXECUTED
        decision.decision_type = StrategyDecision.DECISION_TYPE_ACTION
        decision.decision_details = details
        try:
            frame = inspect.currentframe()
            caller_frame = frame.f_back
            decision.decision_name = caller_frame.f_code.co_name
        finally:
            del frame
        self.decision_log.decisions.append(decision)

    def action_postcondition_already_satisfied(self, details=""):
        decision = StrategyDecision()
        decision.decision_result = StrategyDecision.DECISION_RESULT_POSTCONDITION_ALREADY_SATISFIED
        decision.decision_type = StrategyDecision.DECISION_TYPE_ACTION
        decision.decision_details = details
        try:
            frame = inspect.currentframe()
            caller_frame = frame.f_back
            decision.decision_name = caller_frame.f_code.co_name
        finally:
            del frame
        self.decision_log.decisions.append(decision)

    def task_executed(self, details=""):
        decision = StrategyDecision()
        decision.decision_result = StrategyDecision.DECISION_RESULT_EXECUTED
        decision.decision_type = StrategyDecision.DECISION_TYPE_METHOD
        decision.decision_details = details
        try:
            frame = inspect.currentframe()
            caller_frame = frame.f_back
            decision.decision_name = caller_frame.f_code.co_name
        finally:
            del frame
        self.decision_log.decisions.append(decision)


    def task_precondition_not_satisfied(self, details=""):
        decision = StrategyDecision()
        decision.decision_result = StrategyDecision.DECISION_RESULT_PRECONDITION_FAILED
        decision.decision_type = StrategyDecision.DECISION_TYPE_METHOD
        decision.decision_details = details
        try:
            frame = inspect.currentframe()
            caller_frame = frame.f_back
            decision.decision_name = caller_frame.f_code.co_name
        finally:
            del frame
        self.decision_log.decisions.append(decision)
    
    def task_postcondition_already_satisfied(self, details=""):
        decision = StrategyDecision()
        decision.decision_result = StrategyDecision.DECISION_RESULT_POSTCONDITION_ALREADY_SATISFIED
        decision.decision_type = StrategyDecision.DECISION_TYPE_METHOD
        decision.decision_details = details
        try:
            frame = inspect.currentframe()
            caller_frame = frame.f_back
            decision.decision_name = caller_frame.f_code.co_name
        finally:
            del frame
        self.decision_log.decisions.append(decision)