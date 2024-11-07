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
#include "vaip/vaip.hpp"
#include "../../encryption/src/encryption.hpp"
#include "./compile_model.hpp"
#include "./export_to_xir.hpp"
#include "./subgraph_processer.hpp"
#include "vaip/dpu_sg_report.pb.h"
#include "vitis/ai/env_config.hpp"
#include <filesystem>
#include <glog/logging.h>
#include <google/protobuf/util/json_util.h>
#include <set>
#include <sstream>
#include <vaip/vaip_ort_api.h>
#include "vaip/capability.pb.h"
#include "vaip/with_current_graph.hpp"
// clang-format on

DEF_ENV_PARAM(DEBUG_LEVEL1_DPU, "0")
DEF_ENV_PARAM_2(XLNX_model_clone_external_data_threshold, "128", int64_t)
DEF_ENV_PARAM(XLNX_ENABLE_DUMP_XIR_MODEL, "0")
DEF_ENV_PARAM(XLNX_ENABLE_DUMP_CONSTANT, "0")
DEF_ENV_PARAM(VAIP_COMPILE_RESERVE_CONST_DATA, "0")
DEF_ENV_PARAM(DEBUG_SKIP_COMPILE_XMODEL, "0")
DEF_ENV_PARAM(DEBUG_SKIP_EXPORT_TO_XIR, "0")
DEF_ENV_PARAM_2(XLNX_ONNX_EP_DPU_REPORT_FILE, "", std::string)
DEF_ENV_PARAM(XLNX_ONNX_EP_VERBOSE, "0")
DEF_ENV_PARAM(XLNX_MINIMUM_NUM_OF_CONV, "-2")

#define MY_LOG(n) LOG_IF(INFO, ENV_PARAM(DEBUG_LEVEL1_DPU) >= n)
#define LOG_VERBOSE(n)                                                         \
  LOG_IF(INFO, ENV_PARAM(XLNX_ONNX_EP_VERBOSE) >= n)                           \
      << "[XLNX_ONNX_EP_VERBOSE] "
namespace {
using namespace vaip_core;

static std::vector<const xir::Subgraph*>
get_dpu_subgraphs(const xir::Graph* graph) {
  auto root = graph->get_root_subgraph();
  auto children = root->children_topological_sort();
  auto ret = std::vector<const xir::Subgraph*>();

  for (auto c : children) {
    if (c->has_attr("device")) {
      auto device = c->get_attr<std::string>("device");
      if (device == "DPU") {
        ret.emplace_back(c);
      }
    }
  }
  return ret;
}

static std::map<std::string, int32_t>
collect_subgraph_stat(const xir::Graph* graph) {
  std::map<std::string, int32_t> subgraph_count;
  auto root = graph->get_root_subgraph();
  auto children = root->children_topological_sort();
  for (auto c : children) {
    if (c->has_attr("device")) {
      auto device = c->get_attr<std::string>("device");
      if (device == "DPU") {
        device = "IPU";
      }
      if (device == "CPU") {
        continue;
      }
      if (device != "USER") {
        ++subgraph_count[device];
      }
    }
  }
  return subgraph_count;
}

static void save_xmodel(const xir::Graph& graph, const std::string& filename,
                        const IPass& pass) {
  auto context = pass.get_context();
  const auto& key = pass.get_config_proto().encryption_key();
  std::string s;
  graph.serialize_to_string(&s);
  if (key != "") {
    s = vaip_encryption::aes_encryption(s, key);
  }
  context->write_file(std::filesystem::path(filename).filename().u8string(), s);
}
static std::unique_ptr<xir::Graph> load_xmodel(const std::string& filename,
                                               const IPass& pass) {
  const auto& key = pass.get_config_proto().encryption_key();
  std::ifstream ifs(filename, std::ios::binary);
  std::vector<char> file_char_array((std::istreambuf_iterator<char>(ifs)),
                                    std::istreambuf_iterator<char>());
  ifs.close();
  std::string s(file_char_array.begin(), file_char_array.end());
  if (key != "") {
    s = vaip_encryption::aes_decryption(s, key);
  }
  return xir::Graph::deserialize_from_string(s);
}

// identical to the function at vitisai_compile_model.cpp
static void save_protobuf_message(const fs::path& filename,
                                  const google::protobuf::Message& msg) {
  google::protobuf::util::JsonPrintOptions options;
  options.add_whitespace = true;
  auto json_str = std::string();
  auto status =
      google::protobuf::util::MessageToJsonString(msg, &json_str, options);
  CHECK(status.ok()) << "cannot write json string:" << msg.DebugString();
  CHECK(std::ofstream(filename).write(&json_str[0], json_str.size()).good())
      << "failed to write " << filename;
}

static void try_save_report(const IPass& pass, DpuSubgraphReportProto& proto) {
  auto filename = ENV_PARAM(XLNX_ONNX_EP_DPU_REPORT_FILE);
  if (!filename.empty()) {
    save_protobuf_message(pass.get_cache_file_name(filename), proto);
  }
}
// check CNN / RNN models.
// If only 1 conv, we will not invoke xcompiler.
static bool check_min_num_of_conv(const Graph& graph,
                                  const PassDpuParamProto& dpu_param) {
  auto min_num_of_conv = dpu_param.minimum_num_of_conv();
  auto min_num_of_conv_from_env = ENV_PARAM(XLNX_MINIMUM_NUM_OF_CONV);
  // -2 is a special value which means the environment variable is not set
  min_num_of_conv = min_num_of_conv_from_env == -2 ? min_num_of_conv
                                                   : min_num_of_conv_from_env;
  auto graph_num_of_conv = 0;
  for (auto node : graph_nodes(graph)) {
    if (node_op_type(*node) == "Conv") {
      graph_num_of_conv++;
    }
  }
  return graph_num_of_conv >= min_num_of_conv;
}
struct Level1Dpu {
  Level1Dpu(IPass& self) : self_{self}, log_dir_{self.get_log_path()} {}
  void process_run_passes(Graph& cloned_graph) {
    auto& pass_proto = self_.get_pass_proto();
    all_passes_ = IPass::create_passes(self_.get_context(),
                                       pass_proto.pass_dpu_param().sub_pass());
    // NOTE: we must keep passes alive until export_to_xir, because
    // constant folding leave many laze evaluation functions in the
    // closure.
    IPass::run_passes(all_passes_, cloned_graph);
  }

  void process_export_to_xir(Graph& cloned_graph) {
    auto file = log_dir_ / "xir.xmodel";
    if (ENV_PARAM(DEBUG_SKIP_EXPORT_TO_XIR)) {
      xir_graph_ = load_xmodel(file.u8string(), self_);
    } else {
      xir_graph_ = vaip_core::export_to_xir(self_, cloned_graph);
      if (ENV_PARAM(XLNX_ENABLE_DUMP_CONSTANT)) {
        self_.dump_fix_info("fix_info_final.txt");
        self_.dump_const_info("const_info_final.txt");
        self_.dump_const_data("const.bin");
      }
      if (ENV_PARAM(XLNX_ENABLE_DUMP_XIR_MODEL)) {
        auto file = log_dir_ / "xir.xmodel";
        LOG(INFO) << "save xir model to " << file;
        save_xmodel(*xir_graph_, file.u8string(), self_);
      }
    }
  }

  std::unique_ptr<xir::Graph> clone(xir::Graph* xir_graph) {
    std::string buffer = "";
    xir_graph->serialize_to_string(&buffer);
    std::unique_ptr<xir::Graph> cloned_xir_graph =
        xir::Graph::deserialize_from_string(buffer);
    return cloned_xir_graph;
  }

  void process_xcompile(const IPass& pass) {
    std::string subfix =
        ENV_PARAM(VAIP_COMPILE_RESERVE_CONST_DATA) == 1 ? "_fat" : "";

    auto xcompiler_fingerprint = get_xcompiler_fingerprint(
        *pass.get_context(), self_.get_pass_proto().pass_dpu_param());

    auto compiled_xmodel_file =
        log_dir_ /
        (std::string("compiled.") + xcompiler_fingerprint + subfix + ".xmodel");
    auto& pass_proto = self_.get_pass_proto();
    if (ENV_PARAM(DEBUG_SKIP_COMPILE_XMODEL)) {
      compiled_xir_graph_ = load_xmodel(compiled_xmodel_file.u8string(), pass);
    } else {
      auto cloned_xir_graph_ = clone(xir_graph_.get());
      compiled_xir_graph_ = vaip_core::compiler_xir_model(
          std::move(cloned_xir_graph_), *pass.get_context(),
          pass_proto.pass_dpu_param());
      save_xmodel(*compiled_xir_graph_, compiled_xmodel_file.u8string(), pass);
    }
  }

  TensorBufferParam*
  find_onnx_tensor_buffer_param(std::vector<TensorBufferParam>& onnx_input_tbs,
                                const std::string& node_arg_name) {
    for (auto& tb : onnx_input_tbs) {
      if (tb.tensor_name() == node_arg_name)
        return &tb;
    }
    return nullptr;
  }

  void process(IPass& self, Graph& graph) {
    const auto& model = graph_get_model(graph);
    auto cloned_model = model_clone(
        model, VAIP_PROVIDER_OPTION(*self.get_context(),
                                    XLNX_model_clone_external_data_threshold));
    auto& cloned_graph = VAIP_ORT_API(model_main_graph)(*cloned_model);

    if (!check_min_num_of_conv(cloned_graph,
                               self_.get_pass_proto().pass_dpu_param())) {
      // testcase issue#1363
      LOG(INFO) << "[VITIS AI EP] This model is not a supported CNN model "
                   "which will not "
                   "be compiled with DPU.";
      return;
    }
    // use std::ignore would cause it to destruct immediately
    auto graph_saver = WithCurrentGraph(&graph, &self);
    (void)graph_saver;
    try {
      process_run_passes(cloned_graph);
      process_export_to_xir(cloned_graph);
      process_xcompile(self);
    } catch (const std::exception& e) {
      LOG(WARNING) << "Unexcepted exception: "
                   << "(Exception type: " << typeid(e).name() << ") "
                   << e.what();
      return;
    }
    auto dpu_subgraphs = get_dpu_subgraphs(compiled_xir_graph_.get());
    auto map_of_xcompiler_attrs =
        self.get_pass_proto().pass_dpu_param().xcompiler_attrs();
    std::string dpu_subgraph_num_in_config_file = std::string("N/A");
    if (map_of_xcompiler_attrs.find("dpu_subgraph_num") !=
        map_of_xcompiler_attrs.end()) {
      dpu_subgraph_num_in_config_file = std::to_string(
          map_of_xcompiler_attrs.find("dpu_subgraph_num")->second.uint_value());
    }

    DpuSubgraphReportProto report;
    std::unordered_map<const xir::Subgraph*,
                       std::unique_ptr<vaip_level1_dpu::DPUSubgraphProcessor>>
        subgraph_processors;
    std::unordered_map<const xir::Subgraph*, vaip_level1_dpu::ProcessInfo>
        caches;
    for (auto& subgraph : dpu_subgraphs) {
      auto subgraph_processor =
          std::make_unique<vaip_level1_dpu::DPUSubgraphProcessor>(
              graph, self, xir_graph_.get(), compiled_xir_graph_.get());
      auto ret = subgraph_processor->find_xir_anchor_point(subgraph);

      caches[subgraph] = std::move(ret);
      subgraph_processors[subgraph] = std::move(subgraph_processor);
    }

    auto pdi_subgraph_num = 0;
    auto num_of_subgraph_in_dpu = 0;
    for (auto& subgraph : dpu_subgraphs) {
      auto& ret = caches[subgraph];
      auto& subgraph_processor = subgraph_processors[subgraph];
      if (!ret.status) {
        auto* added_entry = report.add_subgraphs();
        added_entry->Swap(subgraph_processor->get_proto());
      } else {
        auto meta_def = subgraph_processor->process(subgraph, ret);
        auto* added_entry = report.add_subgraphs();
        added_entry->Swap(subgraph_processor->get_proto());
        if (meta_def) {
          MY_LOG(1) << "FUSE: " << meta_def->DebugString();
          self.fuse(graph, std::move(*meta_def));
          if (subgraph->has_attr("num_of_pdi_subgraphs")) {
            num_of_subgraph_in_dpu += 1;
            pdi_subgraph_num +=
                subgraph->get_attr<std::int32_t>("num_of_pdi_subgraphs");
          }
          if (ENV_PARAM(DEBUG_LEVEL1_DPU) >= 2) {
            auto filename = "vaip_fuse_dpu_" +
                            std::to_string(num_of_subgraph_in_dpu) + ".onnx";
            auto filepath = log_dir_ / filename;
#if _WIN32
            auto dat_filepath = std::string("NUL");
#else
            auto dat_filepath =
                std::filesystem::relative("/dev/null", filepath);
#endif
            VAIP_ORT_API(graph_save)
            (graph, filepath.u8string(), dat_filepath, 128u);
          }
        }
      }
    }
    LOG_VERBOSE(1) << "total_pdi_subgraphs: " << pdi_subgraph_num;
    LOG_VERBOSE(1) << "total_pdi_swaps: "
                   << pdi_subgraph_num - num_of_subgraph_in_dpu;

    // for show log: No. of Subgraph count
    //  [Vitis AI EP] No. of Subgraphs : CPU XX DPU XX Actually runing on IPU
    for (auto& it : collect_subgraph_stat(compiled_xir_graph_.get())) {
      self_.add_subgraph_device_count(it.first, it.second);
    }

    try_save_report(self, report);
  }
  IPass& self_;

  const std::filesystem::path& log_dir_;
  std::vector<std::shared_ptr<IPass>> all_passes_;
  std::unique_ptr<xir::Graph> xir_graph_;
  std::unique_ptr<xir::Graph> compiled_xir_graph_;
};
} // namespace

DEFINE_VAIP_PASS(Level1Dpu, vaip_pass_level1_dpu)
