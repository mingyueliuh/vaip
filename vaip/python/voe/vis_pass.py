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

import argparse
import importlib
import json
import sys

from voe.rule_ext.node import Node
from voe.tools import visualize_pass
from voe.tools.visualize_pass import VisualizeFakeGraph


def main(args=None):
    """The main routine."""

    if args is None:
        args = sys.argv[1:]

    parser = argparse.ArgumentParser(description="VOE tool to visualize a pass.")
    parser.add_argument(
        "-m", "--module", help="the python module for a voe pass", required=True
    )
    parser.add_argument("-o", "--output", help="output jpeg file", required=True)
    parser.add_argument(
        "-t", "--type", default="png", help="output type, e.g. png, jpg, svg, gv etc"
    )

    options = parser.parse_args(args)
    module = importlib.import_module(options.module)
    rules = module.rules()
    for i in range(len(rules)):
        rule = rules[i]
        (pattern_nodes, g) = visualize_pass.show(json.loads(rule.pattern()))
        graph = VisualizeFakeGraph()
        nodes = {
            k: Node(None, graph, graph.build_with_node(v))
            for k, v in pattern_nodes.items()
        }
        rule.initialize(graph, None, None)
        rule.action(**nodes)
        nodes = graph.nodes
        visualize_pass.show_action(g, nodes, pattern_nodes)
        filename = options.output + "." + str(i)
        g.render(filename, format=options.type)
        print(f"writing to {filename}.{options.type}")


if __name__ == "__main__":
    sys.exit(main())

# cd /workspace/vaip/vaip/python/voe/passes
# for m in `ls *py | grep -v init | grep -v convert_transpose | grep -v convert_softmax | sed 's/.py//'`;
# do
# echo python -m voe.vis_pass -m voe.passes.$m -o $m -t svg
# python -m voe.vis_pass -m voe.passes.$m -o $m -t svg
# done
