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

#include "vaip/vaip.hpp"
#include "vitis/ai/env_config.hpp"

DEF_ENV_PARAM(DEBUG_DD_MERGE_DQCASTGATHER, "0")
#define MY_LOG(n) LOG_IF(INFO, ENV_PARAM(DEBUG_DD_MERGE_DQCASTGATHER) >= n)

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
    { "name": "vaip_pass_dd_merge_quant",
       "plugin": "vaip-pass_dd_merge_quant",
       "disabled": false
    }
*/
namespace {
using namespace vaip_core;
struct Dd_merge_dqcastgather {
  Dd_merge_dqcastgather(IPass& self) : self_{self} {}
  std::unique_ptr<Rule> create_rule(IPass* self) {
    auto ret = std::shared_ptr<vaip_core::Pattern>();
    auto builder = PatternBuilder();
    auto input_0 = builder.wildcard();
    auto Cast_0 = builder.node2("Cast", {input_0});
    auto input_1 = builder.wildcard();
    auto constant_0 = builder.constant();
    auto constant_1 = builder.constant();
    auto DequantizeLinear_0 =
        builder.node2("DequantizeLinear", {input_1, constant_0, constant_1});

    auto Gather_0 = builder.node2("Gather", {DequantizeLinear_0, Cast_0});
    auto QuantizeLinear_0 =
        builder.node2("QuantizeLinear", {Gather_0, constant_0, constant_1});
    auto pattern_ = QuantizeLinear_0;
    // auto gather = builder.node2("Gather", {indeces, input_data});
    return Rule::create_rule(
        pattern_, [=](onnxruntime::Graph* graph, binder_t& binder) -> bool {
          auto attr_nodes = vaip::dd::get_node_names(graph, binder);
          auto indeces_node = binder[input_0->get_id()];
          auto data_node = binder[input_1->get_id()];
          auto in_scale_node = binder[constant_0->get_id()];
          auto in_zp_node = binder[constant_1->get_id()];
          auto out_node = binder[pattern_->get_id()];
          auto in_scale =
              node_arg_get_const_data_as_float(*graph, *in_scale_node.node_arg);
          auto in_zero_point =
              node_arg_get_const_data_as_i8s(*graph, *in_zp_node.node_arg);
          auto indeces_shape =
              *(node_arg_get_shape_i64(*indeces_node.node_arg).get());
          auto data_shape =
              *(node_arg_get_shape_i64(*data_node.node_arg).get());
          int ifm_dim_0 = (int)data_shape[0];
          int ifm_dim_1 = (int)data_shape[1];
          auto in_data =
              node_arg_get_const_data_as_i8s(*graph, *data_node.node_arg);
          auto log_dir = self->get_log_path();
          std::string name = node_arg_get_name(*data_node.node_arg);
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
              in_data.size() * sizeof(int8_t));
          self_.get_context()->write_file(data_bin, char_data);

          std::string zp_name = node_arg_get_name(*in_zp_node.node_arg);
          zp_name.erase(std::remove_if(zp_name.begin(), zp_name.end(),
                                       [&valid_chars](char c) {
                                         return valid_chars.find(c) ==
                                                std::string::npos;
                                       }),
                        zp_name.end());
          std::replace(zp_name.begin(), zp_name.end(), ' ', '_');

          std::string zp_bin = zp_name + ".bin";
          auto zp_path = log_dir / zp_bin;
          std::string zp_file{zp_path.u8string()};
          char_data = gsl::span<const char>(
              reinterpret_cast<const char*>(in_zero_point.data()),
              in_zero_point.size() * sizeof(int8_t));
          self_.get_context()->write_file(zp_bin, char_data);

          auto node_name = node_arg_get_name(*out_node.node_arg);
          std::vector<std::string> inputs = {
              node_arg_get_name(*indeces_node.node_arg),
              node_arg_get_name(*data_node.node_arg),
              node_arg_get_name(*in_scale_node.node_arg),
              node_arg_get_name(*in_zp_node.node_arg)};
          std::vector<std::string> outputs = {node_name};
          auto constant_initializers = std::vector<std::string>{};
          auto [meta_def, err] =
              self->try_fuse(*graph, "dqcastgather" + node_name, inputs,
                             outputs, constant_initializers, "DQCASTGATHER");
          if (meta_def == nullptr) {
            LOG(FATAL) << "Cannot fuse matmul_nbits:  " << err.comments;
          }
          auto& generic_param = *meta_def->mutable_generic_param();
          generic_param["in_scale"] = std::to_string(in_scale);
          generic_param["data_shape"] = std::to_string(data_shape.back());
          generic_param["indeces_shape"] = std::to_string(indeces_shape.back());
          generic_param["ifm_dim_0"] = std::to_string(ifm_dim_0);
          generic_param["ifm_dim_1"] = std::to_string(ifm_dim_1);
          generic_param["data_file"] = data_file;
          generic_param["zp_file"] = zp_file;

          [[maybe_unused]] auto& fused_node =
              self->fuse(*graph, std::move(*meta_def));

          return true;
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

DEFINE_VAIP_PASS(Dd_merge_dqcastgather, vaip_pass_dd_merge_dqcastgather)