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

from voe.pattern import node, wildcard
from voe.rule_ext import Node, Rule, same_as

"""
  test case model enc_0423_4096x2160

  remove fix before relu
  From : relu(fix(input))
  To  : relu(input)
"""


class RemoveFixBeforeRelu(Rule):
    def pattern(self):
        before_fix = node("com.xilinx:fix", wildcard())
        relu = node("com.xilinx:relu", before_fix)
        return relu.build(locals())

    def where(self, before_fix: Node, relu: Node, **kwargs):
        consumers = relu.get_consumers()
        for c in consumers:
            if "fix" != c.op_type() or before_fix.attr("fix_point") != c.attr(
                "fix_point"
            ):
                return False
        return True

    def action(self, relu: Node, before_fix: Node, **kwargs):
        return self.create_node(
            op_type=same_as(relu),
            inputs=same_as(before_fix),
            attrs=same_as(relu),
            anchor_point=relu,
        )


def rules():
    return [RemoveFixBeforeRelu()]
