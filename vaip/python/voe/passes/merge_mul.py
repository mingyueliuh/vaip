##
##  Copyright (C) 2023 – 2024 Advanced Micro Devices, Inc.
##
##  Licensed under the Apache License, Version 2.0 (the "License");
##  you may not use this file except in compliance with the License.
##  You may obtain a copy of the License at
##
##  http://www.apache.org/licenses/LICENSE-2.0
##
##  Unless required by applicable law or agreed to in writing, software
##  distributed under the License is distributed on an "AS IS" BASIS,
##  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
##  See the License for the specific language governing permissions and
##  limitations under the License.
##

import sys

import glog as log
import numpy as np
from voe.anchor_point import CONST
from voe.pattern import node, wildcard, xir_const_op
from voe.rule_ext import Rule, same_as

"""
  test case model 112

  merge mul pass
  From : mul(mul(input,const_a),const_b)
  To  : mul(input,const_a*const_b)
"""


def convert_data(const_a, const_b):
    # const_a type list(float)
    return [x * y for x, y in zip(const_a, const_b)]


class MergeMul(Rule):
    def pattern(self):
        input = wildcard()
        const_a = xir_const_op()
        mul_a = node("com.xilinx:mul", input, const_a)
        const_b = xir_const_op()
        mul_b = node("com.xilinx:mul", mul_a, const_b)
        return mul_b.build(locals())

    def where(self, mul_a, const_a, const_b, **kwargs):
        if len(mul_a.get_consumers()) > 1:
            return False
        return const_a.shape() == const_b.shape()

    def action(self, input, const_a, mul_a, const_b, mul_b, **_others):
        new_const_b = self.create_node(
            op_type="com.xilinx:const",
            data_type=same_as(const_b),
            shape=same_as(const_b),
            anchor_point=(const_b, CONST),
        )

        new_const_b.create_const(
            convert_data(const_a.const_data(), const_b.const_data())
        )

        return self.create_node(
            op_type=same_as(mul_b),
            inputs=[input, new_const_b],
            attrs=same_as(mul_b),
            anchor_point=mul_b,
        )


def rules():
    return [MergeMul()]
