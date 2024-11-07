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
// clang-format off
#include "./graph_holder.hpp"

#include <glog/logging.h>

#include <fstream>
#include <vitis/ai/env_config.hpp>
#include "../../encryption/src/encryption.hpp"
#include "vaip/xir_headers.hpp"
// clang-format on
namespace vaip_core {

GraphHolder::GraphHolder(const vaip_core::PassContext& context,
                         const std::string& filename,
                         std::string& decryption_key) {
  auto ret = std::shared_ptr<GraphHolder>();
  auto log_dir = context.get_log_dir();
  auto path = log_dir / filename;
  auto full_filename = path.u8string();
  auto maybe_xmodel_content = context.read_file_c8(filename);
  CHECK(maybe_xmodel_content.has_value())
      << "read cache file error: can't read " << filename;
  auto xmodel_context = maybe_xmodel_content.value();
  std::string s;
  if (decryption_key != "") {
    // TODO : aes_decryption change the first argument to accecpt gsl::span,
    // not vector<char>
    s = vaip_encryption::aes_decryption(
        std::string(xmodel_context.begin(), xmodel_context.end()),
        decryption_key);
    graph_ = xir::Graph::deserialize_from_memory(s.data(), s.size());
  } else {
    graph_ = xir::Graph::deserialize_from_memory(xmodel_context.data(),
                                                 xmodel_context.size());
  }
  init_subgraph();
}

void GraphHolder::init_subgraph() {
  auto root = graph_->get_root_subgraph();
  auto children = root->children_topological_sort();
  for (auto c : children) {
    CHECK(c->has_attr("device"));
    auto device = c->get_attr<std::string>("device");
    if (device == "DPU") {
      subgraphs_.emplace_back(c);
    }
  }
}

const xir::Graph* GraphHolder::get_graph() const { //
  return graph_.get();
}

std::vector<const xir::Subgraph*> GraphHolder::get_subgraphs() const {
  return subgraphs_;
}

} // namespace vaip_core
