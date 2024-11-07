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
struct Dd_merge_gather_add {
  Dd_merge_gather_add(IPass& self) : self_{self} {}

  std::string get_file_name(std::string name) {
    std::string valid_chars =
        "-_.()"
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    name.erase(std::remove_if(name.begin(), name.end(),
                              [&valid_chars](char c) {
                                return valid_chars.find(c) == std::string::npos;
                              }),
               name.end());
    std::replace(name.begin(), name.end(), ' ', '_');
    std::string data_bin = name + ".bin";
    return data_bin;
  }

  std::unique_ptr<Rule> create_rule(IPass* self) {
    auto builder = PatternBuilder();
    auto input_ind = builder.wildcard();
    auto input_data = builder.wildcard();
    auto gather = builder.node2("Gather", {input_data, input_ind});
    auto dq1_sc = builder.constant();
    auto dq1_zp = builder.constant();
    auto dq1 = builder.node2("DequantizeLinear", {gather, dq1_sc, dq1_zp});
    auto const_inp = builder.wildcard();
    auto dq2_sc = builder.constant();
    auto dq2_zp = builder.constant();
    auto dq2 = builder.node2("DequantizeLinear", {const_inp, dq2_sc, dq2_zp});
    auto add = builder.commutable_node("Add", dq1, dq2);

    std::shared_ptr<Pattern> pattern_ = add;
    return Rule::create_rule(
        pattern_, [=](onnxruntime::Graph* graph, binder_t& binder) -> bool {
          auto input_ind_node = binder[input_ind->get_id()];
          auto input_data_node = binder[input_data->get_id()];
          auto act_scale_node = binder[dq1_sc->get_id()];
          auto act_zp_node = binder[dq1_zp->get_id()];
          auto wts_scale_node = binder[dq2_sc->get_id()];
          auto wts_zp_node = binder[dq2_zp->get_id()];
          auto out_node = binder[pattern_->get_id()];
          // bool is_const = false;
          auto node_name = node_arg_get_name(*out_node.node_arg);
          // if ((*node_arg_is_constant)(*graph, *input_ind_node.node_arg)) {
          //   // return false;
          //   is_const = true;
          // }
          LOG(INFO) << "Matched gather Add ";

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
          auto data_bin = get_file_name(name);
          auto char_data = gsl::span<const char>(
              reinterpret_cast<const char*>(in_data.data()),
              in_data.size() * sizeof(uint8_t));
          self_.get_context()->write_file(data_bin, char_data);

          std::string wts_file;
          auto const_inp_node = binder[const_inp->get_id()];
          if ((*node_arg_is_constant)(*graph, *const_inp_node.node_arg)) {
          } else {
            LOG(INFO) << "NOT CONSTANT";
          }
          auto wts_data =
              node_arg_get_const_data_as_u8s(*graph, *const_inp_node.node_arg);

          std::string wname = node_arg_get_name(*const_inp_node.node_arg);
          auto wts_bin = get_file_name(wname);
          char_data = gsl::span<const char>(
              reinterpret_cast<const char*>(wts_data.data()),
              wts_data.size() * sizeof(uint8_t));
          self_.get_context()->write_file(wts_bin, char_data);

          auto act_sc = node_arg_get_const_data_as_float(
              *graph, *act_scale_node.node_arg);
          auto act_zp =
              vaip::dd::get_zp_from_node(*graph, *act_zp_node.node_arg);

          auto wts_sc = node_arg_get_const_data_as_float(
              *graph, *wts_scale_node.node_arg);
          auto wts_zp =
              vaip::dd::get_zp_from_node(*graph, *wts_zp_node.node_arg);

          std::vector<std::string> inputs = {
              node_arg_get_name(*input_data_node.node_arg),
              node_arg_get_name(*input_ind_node.node_arg)};

          std::vector<std::string> outputs = {node_name};

          auto constant_initializers = std::vector<std::string>{};
          auto [meta_def, err] =
              self->try_fuse(*graph, "gather_" + node_name, inputs, outputs,
                             constant_initializers, "GATHER_ADD");

          if (meta_def == nullptr) {
            LOG(FATAL) << "Cannot fuse gather:  " << err.comments;
          }

          auto& generic_param = *meta_def->mutable_generic_param();
          generic_param["indeces_shape"] = std::to_string(ind_shape.back());
          generic_param["data_shape"] = std::to_string(data_shape.back());
          generic_param["ind_shape"] = std::to_string(ind_shape.back());
          generic_param["ifm_dim_0"] = std::to_string(ifm_dim_0);
          generic_param["ifm_dim_1"] = std::to_string(ifm_dim_1);
          generic_param["act_scale"] = std::to_string(act_sc);
          generic_param["wts_scale"] = std::to_string(wts_sc);
          generic_param["wts_zp"] = std::to_string(wts_zp);
          generic_param["act_zp"] = std::to_string(act_zp);
          generic_param["data_file"] = data_bin;
          generic_param["wts_file"] = wts_bin;
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

DEFINE_VAIP_PASS(Dd_merge_gather_add, vaip_pass_dd_merge_gather_add)
