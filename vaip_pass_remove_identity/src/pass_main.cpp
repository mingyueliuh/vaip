/*
 *     The Xilinx Vitis AI Vaip in this distribution are provided under the
 * following free and permissive binary-only license, but are not provided in
 * source code form.  While the following free and permissive license is similar
 * to the BSD open source license, it is NOT the BSD open source license nor
 * other OSI-approved open source license.
 *
 *      Copyright (C) 2022 Xilinx, Inc. All rights reserved.
 *      Copyright (C) 2023 – 2024 Advanced Micro Devices, Inc. All rights
 * reserved.
 *
 *      Redistribution and use in binary form only, without modification, is
 * permitted provided that the following conditions are met:
 *
 *      1. Redistributions must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution.
 *
 *      2. The name of Xilinx, Inc. may not be used to endorse or promote
 * products redistributed with this software without specific prior written
 * permission.
 *
 *      THIS SOFTWARE IS PROVIDED BY XILINX, INC. "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL XILINX, INC. BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *      PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 */

#include "vaip/vaip.hpp"
#include "vitis/ai/env_config.hpp"
#include <glog/logging.h>
DEF_ENV_PARAM(DEBUG_REMOVE_IDENTITY, "0")
#define MY_LOG(n) LOG_IF(INFO, ENV_PARAM(DEBUG_REMOVE_IDENTITY) >= n)

using namespace vaip_core;
namespace {

static bool is_identity_node(const Node& node) {
  auto ret = false;
  if (node_is_op(node, "transpose", "com.xilinx")) {
    auto order = node_get_attr_ints(node, "order");
    auto expected = 0;
    ret = true;
    for (auto i : order) {
      ret = ret && (i == expected);
      expected = expected + 1;
    }
  } else if (node_is_op(node, "Transpose", "")) {
    if (node_has_attr(node, "perm")) {
      auto order = node_get_attr_ints(node, "perm");
      auto expected = 0;
      ret = true;
      for (auto i : order) {
        ret = ret && (i == expected);
        expected = expected + 1;
      }
    }
  } else if (node_is_op(node, AnchorPoint::IDENTITY_OP, "com.xilinx")) {
    ret = true;
  } else if (node_is_op(node, "reshape", "com.xilinx")) {
    auto& input_node_arg = *node_get_inputs(node)[0].node_arg;
    auto input_shape_ptr = node_arg_get_shape_i64(input_node_arg);
    CHECK(input_shape_ptr != nullptr)
        << node_arg_as_string(input_node_arg) << " shape absent";
    auto input_shape = *input_shape_ptr;
    auto output_shape = node_get_output_shape(node, 0);
    ret = input_shape == output_shape;
  }
  return ret;
}

struct RemoveIdentity {
  RemoveIdentity(IPass& self) : self_{self} {}
  void process(IPass& self, Graph& graph) {
    for (auto node_idx : graph_get_node_in_topoligical_order(graph)) {
      auto node_ptr = VAIP_ORT_API(graph_get_node)(graph, node_idx);
      CHECK(node_ptr != nullptr);
      auto& node = *node_ptr;
      auto inputs = node_get_inputs(node);
      auto input_node_args = node_inputs_2_node_args(inputs);
      auto size = inputs.size();
      auto identity_nodes_indices = std::vector<size_t>();
      identity_nodes_indices.reserve(inputs.size());
      for (auto i = 0u; i < size; ++i) {
        if (inputs[i].node == nullptr) {
          // it might be a graph inputs.
          continue;
        }

        auto& input_node = *inputs[i].node;
        if (is_identity_node(input_node)) {
          identity_nodes_indices.push_back(i);
        }
      }

      if (identity_nodes_indices.empty()) {
        continue;
      }
      for (auto i : identity_nodes_indices) {
        auto& identity_node = *inputs[i].node;
        PASS_LOG(self, 1) << "remove identity node: "
                          << node_as_string(identity_node)     //
                          << " the " << i << " arg of node: "  //
                          << VAIP_ORT_API(node_get_name)(node) //
                          << " "                               //
                          << node_as_string(node);
      }

      for (auto i : identity_nodes_indices) {
        auto& identity_node = *inputs[i].node;
        auto inputs_of_identity_node = node_get_inputs(identity_node);
        CHECK_EQ(inputs_of_identity_node.size(), 1u)
            << " node=" << node_as_string(node);
        auto input_node_arg_of_identity_node =
            inputs_of_identity_node[0].node_arg;
        //
        input_node_args[i] = input_node_arg_of_identity_node;
        PASS_LOG(self, 1) << node_arg_as_string(
            *input_node_arg_of_identity_node);
      }
      auto outputs = node_get_output_node_args(node);
      if (outputs.size() == 1) {
        NodeBuilder(graph, self_)
            .clone_node(node)
            .set_input_node_args(input_node_args)
            .set_anchor_point1(node)
            .build();
      }
    }
  }
  IPass& self_;
};
} // namespace

DEFINE_VAIP_PASS(RemoveIdentity, vaip_pass_remove_identity)
