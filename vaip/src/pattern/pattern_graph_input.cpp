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

#include "./pattern_graph_input.hpp"
#include <sstream>

#include "vaip/graph.hpp"
#include <vaip/vaip_ort_api.h>

#include "./pattern_log.hpp"
#include "vaip/pattern.pb.h"

namespace vaip_core {
PatternGraphInput::PatternGraphInput(int id) : Pattern(id) {}
PatternGraphInput::~PatternGraphInput() {}

std::string PatternGraphInput::debug_string() const {
  auto ret = std::string("#");
  ret += std::to_string(this->get_id()) + std::string("(");
  ret += std::string("GraphInput");
  ret += std::string(")");
  return ret;
}

std::string PatternGraphInput::virtualize_label() const {
  std::ostringstream str;
  str << "[" << this->get_id() << "] GraphInput";
  return str.str();
}

BinderBuilderPtr
PatternGraphInput::match_uncached(const onnxruntime::Graph& graph,
                                  const NodeInput& node_input,
                                  const BinderBuilder& binder) const {
  auto ret = BinderBuilderPtr();
  if (node_input.node == nullptr) {
    auto inputs = graph_get_inputs(graph);
    auto it = std::find(inputs.begin(), inputs.end(), node_input.node_arg);
    if (it != inputs.end()) {
      ret = binder.add(get_id(), node_input);
    }
  }
  if (ret == nullptr) {
    MATCH_FAILED << "not a graph input: "
                 << (node_input.node != nullptr
                         ? node_as_string(*node_input.node)
                         : node_arg_as_string(*node_input.node_arg));
  } else {
    MY_LOG(1) << "MATCH OK. ID=" << get_id() << ", graph input matched."
              << (node_input.node != nullptr
                      ? node_as_string(*node_input.node)
                      : node_arg_as_string(*node_input.node_arg));
  }
  return ret;
}
void PatternGraphInput::dump_to_proto_imp(RootPatternProto& pattern_proto,
                                          PatternProto& this_proto) const {
  this_proto.mutable_graph_input();
}
} // namespace vaip_core