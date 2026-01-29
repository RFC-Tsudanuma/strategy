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
# - Masafumi Horiguchi

from abc import ABC,abstractmethod
from state import Position

class Filter(ABC):
    def __init__(self):
        super().__init__()

    def non_filter(self,pos:Position) -> Position:
        """フィルタをかけない場合そのまま返す"""
        return pos

    def lpf(self,x:float,x_tm1:float,alpha:float) -> float:
        return alpha * x + (1-alpha) * x_tm1
    


class LPF(Filter):
    def __init__(self,alpha:float=0.5):
        super().__init__()
        self.alpha = alpha
        self.local_pre_val:Position = Position(Position.COORD_LOCAL,0,0,0)
        self.global_pre_val:Position = Position(Position.COORD_GLOBAL,None,None,None)

    def local_filter(self,pos:Position) -> Position:
        if self.local_pre_val.x is None or self.local_pre_val.y is None or self.local_pre_val.theta is None:
            self.local_pre_val = pos
        x = self.lpf(pos.x,self.local_pre_val.x,self.alpha)
        y = self.lpf(pos.y,self.local_pre_val.y,self.alpha)
        theta = self.lpf(pos.theta,self.local_pre_val.theta,self.alpha)
        self.local_pre_val = pos
        return Position(Position.COORD_LOCAL,x,y,theta)
    
    def reset_local_pos(self) -> None:
        self.local_pre_val = Position(Position.COORD_LOCAL,0,0,0)