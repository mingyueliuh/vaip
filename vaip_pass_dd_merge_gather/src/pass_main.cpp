/*
 *     The Xilinx Vitis AI Vaip in this distribution are provided under the
 * following free and permissive binary-only license, but are not provided in
 * source code form.  While the following free and permissive license is similar
 * to the BSD open source license, it is NOT the BSD open source license nor
 * other OSI-approved open source license.
 *
 *      Copyright (C) 2023 – 2024 Advanced Micro Devices, Inc.
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
#include <glog/logging.h>

#ifdef __GNUC__
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wpedantic"
#  pragma GCC diagnostic ignored "-Wconversion"
#endif
#include "vaip/dd/coeffs.hpp"
#include "vaip/dd/dd_utils.hpp"
#include <fstream>
#include <iostream>

#ifdef __GNUC__
#  pragma GCC diagnostic pop
#endif

#include "vaip/pattern_zoo.hpp"
#include "vaip/vaip.hpp"
#include "vitis/ai/env_config.hpp"

DEF_ENV_PARAM(DEBUG_DD_MERGE_GATHER, "0")
#define MY_LOG(n) LOG_IF(INFO, ENV_PARAM(DEBUG_DD_MERGE_GATHER) >= n)

/**
 * test case: <???>
 *
 *
 * Replace pattern:
 *
 * From: <???>
 * To  : <???>
 */

// add the following line in your vaip_config.json
/*
    { "name": "vaip_pass_dd_merge_gather",
       "plugin": "vaip-pass_dd_merge_gather",
       "disabled": false
    }
*/
namespace {
using namespace vaip_core;
struct Dd_merge_gather {
  Dd_merge_gather(IPass& self) : self_{self} {}
  std::unique_ptr<Rule> create_rule(IPass* self) {

    auto gather = vaip::pattern_zoo::get_pattern("m_gather");
    CHECK(gather != nullptr) << "Pattern returned is null";

    return Rule::create_rule(
        gather, [=](onnxruntime::Graph* graph, binder_t& binder) -> bool {
          auto input_ind_node = binder["input_ind"];
          auto input_data_node = binder["input_data"];
          auto out_node = binder["gather"];
          bool is_const = false;
          auto node_name = node_arg_get_name(*out_node.node_arg);
          if ((*node_arg_is_constant)(*graph, *input_ind_node.node_arg)) {
            // return false;
            is_const = true;
          }
          MY_LOG(1) << "Matched gather";

          // for generic params
          auto ind_shape =
              *(node_arg_get_shape_i64(*input_ind_node.node_arg).get());
          auto data_shape =
              *(node_arg_get_shape_i64(*input_data_node.node_arg).get());
          int ifm_dim_0 = (int)data_shape[0];
          int ifm_dim_1 = (int)data_shape[1];

          auto in_data =
              node_arg_get_const_data_as_u8s(*graph, *input_data_node.node_arg);
          auto log_dir = self->get_log_path();
          std::string name = node_arg_get_name(*input_data_node.node_arg);
          std::string valid_chars =
              "-_.()"
              "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
          name.erase(std::remove_if(name.begin(), name.end(),
                                    [&valid_chars](char c) {
                                      return valid_chars.find(c) ==
                                             std::string::npos;
                                    }),
                     name.end());
          std::replace(name.begin(), name.end(), ' ', '_');
          std::string data_bin = name + ".bin";
          auto path = log_dir / data_bin;
          std::string data_file{path.u8string()};
          auto char_data = gsl::span<const char>(
              reinterpret_cast<const char*>(in_data.data()),
              in_data.size() * sizeof(uint8_t));
          self_.get_context()->write_file(data_bin, char_data);

          std::string idata_file;
          if (is_const) {
            auto in_indeces = node_arg_get_const_data_as_i64s(
                *graph, *input_ind_node.node_arg);
            log_dir = self->get_log_path();
            name = node_arg_get_name(*input_ind_node.node_arg);
            valid_chars = "-_.()"
                          "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"
                          "0123456789";
            name.erase(std::remove_if(name.begin(), name.end(),
                                      [&valid_chars](char c) {
                                        return valid_chars.find(c) ==
                                               std::string::npos;
                                      }),
                       name.end());
            std::replace(name.begin(), name.end(), ' ', '_');
            data_bin = name + ".bin";
            char_data = gsl::span<const char>(
                reinterpret_cast<const char*>(in_indeces.data()),
                in_indeces.size() * sizeof(int64_t));
            self_.get_context()->write_file(data_bin, char_data);
          }

          std::vector<std::string> inputs = {
              node_arg_get_name(*input_data_node.node_arg),
              node_arg_get_name(*input_ind_node.node_arg)};

          std::vector<std::string> outputs = {node_name};

          auto constant_initializers = std::vector<std::string>{};
          auto [meta_def, err] =
              self->try_fuse(*graph, "gather_" + node_name, inputs, outputs,
                             constant_initializers, "GATHER");

          if (meta_def == nullptr) {
            LOG(FATAL) << "Cannot fuse gather:  " << err.comments;
          }

          auto& generic_param = *meta_def->mutable_generic_param();
          generic_param["indeces_shape"] = std::to_string(ind_shape.back());
          generic_param["data_shape"] = std::to_string(data_shape.back());
          generic_param["ind_shape"] = std::to_string(ind_shape.back());
          generic_param["ifm_dim_0"] = std::to_string(ifm_dim_0);
          generic_param["ifm_dim_1"] = std::to_string(ifm_dim_1);
          generic_param["data_file"] = data_file;
          generic_param["ind_is_const"] = std::to_string(is_const);
          generic_param["idata_file"] =
              idata_file; // indices data file, to be used only if input to
                          // gather is constant

          [[maybe_unused]] auto& fused_node =
              self->fuse(*graph, std::move(*meta_def));
          return false; // return true if graph is modified.
        });
  }
  // apply the rule
  void process(IPass& self, Graph& graph) {
    MY_LOG(1) << self_.get_pass_proto().name() << "["
              << self_.get_pass_proto().plugin() << "] start processing graph";
    create_rule(&self)->apply(&graph);
    MY_LOG(1) << self_.get_pass_proto().name() << "["
              << self_.get_pass_proto().plugin() << "] finish processing graph";
  }

  IPass& self_;
};
} // namespace

DEFINE_VAIP_PASS(Dd_merge_gather, vaip_pass_dd_merge_gather)
