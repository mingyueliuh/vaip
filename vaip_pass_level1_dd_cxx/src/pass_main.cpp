/*
 *     The Xilinx Vitis AI Vaip in this distribution are provided under the
 * following free and permissive binary-only license, but are not provided in
 * source code form.  While the following free and permissive license is similar
 * to the BSD open source license, it is NOT the BSD open source license nor
 * other OSI-approved open source license.
 *
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
#include "../../vaip/src/stat.hpp"
#include "../../encryption/src/encryption.hpp"
#include "vitis/ai/env_config.hpp"
#include <filesystem>
#include <glog/logging.h>
#include <google/protobuf/util/json_util.h>
#include <set>
#include <sstream>
#include <fstream>
#include <vaip/vaip_ort_api.h>
#include "vaip/capability.pb.h"
#include "vaip/with_current_graph.hpp"
#include "./fuse.hpp"
#include <op_fuser/fusion_rt.hpp>
// clang-format on

using namespace std::string_literals;

DEF_ENV_PARAM(DEBUG_LEVEL1_SHELL_1, "0")
DEF_ENV_PARAM_2(XLNX_model_clone_external_data_threshold, "128", int64_t)
DEF_ENV_PARAM(XLNX_ENABLE_DUMP_XIR_MODEL, "0")
DEF_ENV_PARAM(XLNX_ENABLE_DUMP_CONSTANT, "0")
DEF_ENV_PARAM(VAIP_COMPILE_RESERVE_CONST_DATA, "0")
DEF_ENV_PARAM(DEBUG_SKIP_COMPILE_XMODEL, "0")
DEF_ENV_PARAM(DEBUG_SKIP_EXPORT_TO_XIR, "0")
DEF_ENV_PARAM(XLNX_ONNX_EP_VERBOSE, "0")
DEF_ENV_PARAM(XLNX_MINIMUM_NUM_OF_CONV, "-2")

#define MY_LOG(n) LOG_IF(INFO, ENV_PARAM(DEBUG_LEVEL1_SHELL_1) >= n)
#define LOG_VERBOSE(n)                                                         \
  LOG_IF(INFO, ENV_PARAM(XLNX_ONNX_EP_VERBOSE) >= n)                           \
      << "[XLNX_ONNX_EP_VERBOSE] "
namespace {
using namespace vaip_core;
[[maybe_unused]] static std::string
vector_floats_to_string(const std::vector<float>& values) {
  std::ostringstream str;
  auto sep = std::string("");
  for (auto value : values) {
    str << sep << value;
    sep = " ";
  }
  return str.str();
}

[[maybe_unused]] static std::string
vector_int_to_string(const std::vector<int>& values) {
  std::ostringstream str;
  auto sep = std::string("");
  for (auto value : values) {
    str << sep << value;
    sep = " ";
  }
  return str.str();
}
struct Level1DynamicDispatch {
  Level1DynamicDispatch(IPass& self)
      : self_{self}, log_dir_{self.get_log_path()} {}
  void process_run_passes(Graph& cloned_graph) {
    (void)self_.get_context()
        ->xclbin_path_to_cache_files(
            self_.get_pass_proto().pass_dpu_param().xclbin())
        .string();
    auto& pass_proto = self_.get_pass_proto();
    all_passes_ = IPass::create_passes(self_.get_context(),
                                       pass_proto.pass_dpu_param().sub_pass());
    IPass::run_passes(all_passes_, cloned_graph);
  }
  void print_fused_stats(Graph& fused_graph) {
    auto nodes_in_fusedgraph = graph_get_node_in_topoligical_order(fused_graph);
    std::map<std::string, int> op_count;
    std::cout << "|" << std::setw(20) << std::left << "Op Name"
              << " | " << std::setw(10) << std::right << "Count"
              << " | " << std::endl;
    std::cout << "|"
              << "--------------------"
              << "-|-"
              << "-----------|" << std::endl;
    for (auto& node_idx : nodes_in_fusedgraph) {
      auto node = VAIP_ORT_API(graph_get_node)(fused_graph, node_idx);
      auto& op_type = VAIP_ORT_API(node_op_type)(*node);
      // if (op_count.find(op_type) == op_count.end()) {
      //   op_count[op_type] = 0;
      // }
      op_count[op_type]++;
    }
    for (const auto& m : op_count)
      std::cout << "|" << std::setw(20) << std::left << m.first << " | "
                << std::setw(10) << std::right << m.second << " | "
                << std::endl;
    std::cout << "|"
              << "--------------------"
              << "-|-"
              << "-----------|" << std::endl;
    std::cout << "|" << std::setw(20) << std::left << "Total"
              << " | " << std::setw(10) << std::right
              << nodes_in_fusedgraph.size() << " | " << std::endl;
  }
  void process(IPass& self, Graph& graph) {
    const auto& session_option = self.get_config_proto().provider_options();
    const bool write_debug_files =
        session_option.contains("dd_write_debug_files") &&
                session_option.at("dd_write_debug_files") == "true"
            ? true
            : false;

    auto log_dir = self.get_log_path();
    // Save original graph
    auto file = log_dir / "onnx.onnx";
    auto dat_file = "onnx.dat";
    if (write_debug_files) {
      CHECK(!log_dir.empty()) << "log dir is empty, call saving onnx.onnx";
      VAIP_ORT_API(graph_save)
      (graph, file.u8string(), dat_file, std::numeric_limits<size_t>::max());
    }
    // Clone graph
    const auto& model = graph_get_model(graph);
    auto cloned_model = model_clone(
        model, VAIP_PROVIDER_OPTION(*self.get_context(),
                                    XLNX_model_clone_external_data_threshold));
    auto& cloned_graph = VAIP_ORT_API(model_main_graph)(*cloned_model);

    // use std::ignore would cause it to destruct immediately
    auto graph_saver = WithCurrentGraph(&graph, &self);
    (void)graph_saver;
    try {
      // Process all fusion passes
      process_run_passes(cloned_graph);
      // save fused graph
      auto fused_file = log_dir / "fused_cxx.onnx";
      auto fused_dat_file = "fused_cxx.dat";
      if (write_debug_files) {
        VAIP_ORT_API(graph_save)
        (cloned_graph, fused_file.u8string(), fused_dat_file,
         std::numeric_limits<size_t>::max());
      }

      if (session_option.find("log_level") != session_option.end()) {
        const auto& log_level = session_option.find("log_level")->second;
        if (log_level == "info") {
          print_fused_stats(cloned_graph);
        }
      }

      // return; // For Python passes
    } catch (const std::exception& e) {
      LOG(WARNING) << "Unexcepted exception: "
                   << "(Exception type: " << typeid(e).name() << ") "
                   << e.what();
      return;
    }
    auto local_context = self_.get_context();
    auto partitioner_ints = local_context->measure("onnx_graph_partitioner");
    // invoke the partitioned  here.
    auto [node_subgraph_labels, subgraph_node_cluster, target_label,
          idx_node_map] = dd::partition_onnx_model(cloned_graph);

    // LOG(DEBUG) << "Subgraph Node Cluster ";
    for (auto x : subgraph_node_cluster) {
      std::ostringstream str;
      str << " SubGraph[" << x.first << "]";
      for (auto j : x.second) {
        str << " " << j;
      }
      // LOG(DEBUG) << str.str();
    }
    // LOG(DEBUG) << "Target label ";
    for (auto x : target_label) {
      // LOG(DEBUG) << " label[" << x.first << "] = " << x.second;
    }
    // LOG(DEBUG) << "Idx node map ";
    for (auto [x, y] : idx_node_map) {
      // LOG(DEBUG) << x << " " << y;
    }
    std::string model_category = "";
    if (session_option.find("model_name") != session_option.end()) {
      const auto& model_name = session_option.find("model_name")->second;
      if (session_option.find("model_category") != session_option.end()) {
        model_category = session_option.find("model_category")->second;
        LOG(INFO) << "model_name = " << model_name
                  << ", model_category = " << model_category;
      }
    }
    partitioner_ints = nullptr;
    auto json_creation = local_context->measure("dd_json_creation");
    // Now for each subgraph, we need to
    //      1: Update the context.json
    //      2: Generate dod.json
    // Expected Flow:
    //  1. Figure out input/output tensors
    //  2. Use it to extract subgraph metadef proto using try_fuse on original
    //  graph
    //  3. Use level_1_fuse to update context.json
    //  4. Use (1) to extract subgraph metadef proto from fused_graph
    //  5. Use level_2_fuse to extract true subgraph from fused_graph
    //  6. Call DD FrontEnd to generate dod.json from subgraph in (5)
    auto model_dir = log_dir;
    nlohmann::json subgraph_metadefs;
    for (const auto& x : subgraph_node_cluster) {
      const auto& subgraph_label = x.first;
      const auto& subgraph_nodes = x.second;
      // auto pre_fuse =
      // local_context->measure(std::to_string(subgraph_label)+"_pre_fuse");
      if (!subgraph_nodes.empty() && target_label[subgraph_nodes[0]] == "CPU") {
        // LOG(DEBUG) << "ignore " << target_label[subgraph_nodes[0]]
        //           << " subgraph_label=" << subgraph_label;
        continue;
      }
      auto model_filename = std::string("fused.onnx");
      auto meta_json_name = model_filename + ".subgraph_"s +
                            std::to_string(subgraph_label) + ".json"s;
      auto meta_json_path = std::filesystem::path(model_dir) / meta_json_name;

      const auto subgraph_path = meta_json_path.string();
      const auto subgraph_name = meta_json_path.filename().string();
      const auto meta_state_path =
          fs::path{meta_json_path}.replace_extension("state");
      const auto meta_state_name = meta_state_path.filename();
      // LOG(DEBUG) << "meta_json_path " << meta_json_path;

      auto model = std::string("fused.onnx");
      auto model_ = std::string("fused.onnx");

      auto [subg_inputs, subg_outputs] = dd::get_subgraph_input_outputs(
          cloned_graph, subgraph_nodes, idx_node_map, meta_json_path,
          subgraph_label);

      auto constant_initializers = std::vector<std::string>{};
      // TODO: it might be important to have a unique name.
      std::string name =
          std::string("subgraph_") + std::to_string(subgraph_label);
      // op_type is not so useful, ORT requires it however.

      auto fuse_inst =
          local_context->measure(std::to_string(subgraph_label) + "_dd_fuse");
      std::string op_type = "dd_subgraph";
      auto [meta_def_proto, error] = self_.try_fuse(
          graph, name, subg_inputs, subg_outputs, constant_initializers, "DOD");
      CHECK(meta_def_proto != nullptr)
          << "cannot fuse subgraph:" << error.comments;

      std::string qos_priority = "";
      if (session_option.find("qos_priority") != session_option.end())
        qos_priority = session_option.find("qos_priority")->second;

      dd::prepare_metadef_context_json_from_subgraph3(
          cloned_graph, meta_def_proto.get(), model_category, qos_priority);

      auto& generic_param = *meta_def_proto->mutable_generic_param();
      generic_param["meta_json"] = meta_json_name;
      generic_param["meta_state"] = meta_state_name.string();

      // DD JSON CREATION
      auto [fused_meta_def_proto, err] =
          self_.try_fuse(cloned_graph, name, subg_inputs, subg_outputs,
                         constant_initializers, "DOD");
      // LOG(DEBUG) << "fused_node = " << node_as_string(fused_node);
      auto subgraph = vaip_cxx ::GraphConstRef(cloned_graph)
                          .virtual_fuse(*fused_meta_def_proto);

      /*if (write_debug_files) {
        VAIP_ORT_API(graph_save)
        (subgraph, subgraph_path + ".onnx", subgraph_name + ".dat", 128u);
      }*/
      // auto nodes_in_subgraph = graph_get_node_in_topoligical_order(subgraph);
      // LOG(DEBUG) << "# Nodes in subgraph : " << nodes_in_subgraph.size();

      auto [op_list, new_tensors, new_tensors_map, const_db] =
          dd::graph_prepare_metadata(subgraph, model_dir);
      auto json_str =
          dd::save_tensors_to_json(op_list, new_tensors, new_tensors_map);
      std::vector<char> vec(json_str.begin(), json_str.end());
      local_context->write_file(meta_json_path.filename().string(), vec);
      // Fusion Runtime Compilation and Saving Metastate
      { // this bracket reduces peak memory
        OpsFusion::FusionRuntime fusion_rt;
        auto meta = OpsFusion::load_meta_string(json_str);
        auto xclbin_path =
            self_.get_context()
                ->xclbin_path_to_cache_files(
                    self_.get_pass_proto().pass_dpu_param().xclbin())
                .string();
        auto dd_cache_dir = self_.get_context()->get_log_dir();
        OpsFusion::DDConfig cfg = {};
        auto read_xclbin = self_.get_context()->read_xclbin(xclbin_path);
        auto xclbin = std::vector<char>(read_xclbin.value().begin(),
                                        read_xclbin.value().end());
        cfg.cache_dir = dd_cache_dir.string();
        cfg.xclbin_content = &xclbin;
        cfg.model_name = model_category;
        // TODO : Model name missing in cfg. will add it while rebasing
        std::string dod_txn_root = "dynamic_dispatch_vaiep";
        LOG(INFO) << "BEFORE FUSION_RT";
        fusion_rt.compile(meta, dod_txn_root, cfg, std::move(const_db));
        LOG(INFO) << "COMPILE DONE";
        save_function func = nullptr;
        func = [&local_context](const std::string& path, FILE* file) {
          auto filename = std::filesystem::path(path).filename().string();
          auto writer = local_context->open_file_for_write(filename);
          fseek64(file, 0, SEEK_END);
          auto size = ftell64(file);
          fseek64(file, 0, SEEK_SET);
          size_t buffer_size = 4096;
          auto buffer = std::vector<char>(buffer_size);
          size_t read_count = 0;
          while ((read_count =
                      std::fread(buffer.data(), 1, buffer_size, file)) > 0) {
            writer->fwrite(buffer.data(), read_count);
          }
          fclose(file);
        };
        fusion_rt.save_state(meta_state_name.string(), func);
        LOG(INFO) << "SAVE_STATE DONE";
      }
      // graph must live longer than cloned_graph, we must not change graph
      // before this point.
      auto& dummy_fused_node2 = self_.fuse(graph, std::move(*meta_def_proto));
      auto& dummy_subgraph2 =
          VAIP_ORT_API(node_get_function_body)(dummy_fused_node2);
      if (write_debug_files) {
        VAIP_ORT_API(graph_save)
        (dummy_subgraph2, subgraph_path + ".orig.onnx",
         subgraph_name + ".orig.dat", 128u);
      }
      auto dummy_nodes_in_subgraph2 =
          graph_get_node_in_topoligical_order(dummy_subgraph2);
      // LOG(DEBUG) << "# Nodes in Original subgraph : "
      //            << dummy_nodes_in_subgraph2.size();
    }
  }

private:
  static void write_to_context_dod_json(const std::filesystem::path& model_dir,
                                        const nlohmann::json& meta_def_dod) {
    auto context_dod_json_path = model_dir / "context_dod.json";
    std::ofstream context_dod_json_stream(context_dod_json_path);
    auto meta_def_dod_content = meta_def_dod.dump(2);
    CHECK(context_dod_json_stream
              .write(&meta_def_dod_content[0], meta_def_dod_content.size())
              .good())
        << "failed to write  " << context_dod_json_path;
    context_dod_json_stream.close();
  }

  IPass& self_;

  const std::filesystem::path& log_dir_;
  std::vector<std::shared_ptr<IPass>> all_passes_;
};
} // namespace

DEFINE_VAIP_PASS(Level1DynamicDispatch, vaip_pass_level1_dd_cxx)
