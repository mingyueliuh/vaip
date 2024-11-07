/*
 *      The Xilinx Vitis AI Vaip in this distribution are provided under the
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

#include <cstdint>
#include <glog/logging.h>

#include "./cache_dir.hpp"
#include "./config.hpp"
#include "./file_lock.hpp"
#include "./pass_imp.hpp"
#include "./stat.hpp"
#include "3rd-party/hash-library/md5.h"
#include "node.hpp"
#include "profile_utils.hpp"
#include "vaip/config_reader.hpp"
#include "vaip/custom_op_imp.hpp"
#include "vaip/graph.hpp"
#include "vaip/model.hpp"
#include "vaip/util.hpp"
#include "vaip/vaip.hpp"
#include "vaip/vaip_plugin.hpp"
#include "vitis/ai/env_config.hpp"
#include "vitis/ai/profiling.hpp"
#include "vitis/ai/weak.hpp"
#include <codecvt>
#include <google/protobuf/util/json_util.h>
#include <ios>
#include <limits>
#include <locale>
#include <memory>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <stdlib.h>
#include <string>
#include <xir/graph/graph.hpp>

DEF_ENV_PARAM_2(XLNX_ONNX_EP_REPORT_FILE, "", std::string)
DEF_ENV_PARAM(XLNX_ENABLE_CACHE, "1")
DEF_ENV_PARAM(ENABLE_TAR_CACHE, "0")
DEF_ENV_PARAM(XLNX_ENABLE_SKIP_FATAL, "1")
DEF_ENV_PARAM(XLNX_ONNX_EP_VERBOSE, "0")
DEF_ENV_PARAM(XLNX_ENABLE_FILE_BASED_CACHE_KEY, "0")
DEF_ENV_PARAM_2(DEBUG_MD5_SIG, "", std::string)
DEF_ENV_PARAM(DEBUG_VITIS_AI_EP, "1")
DEF_ENV_PARAM(DEBUG_FILE_LOCK, "0")
DEF_ENV_PARAM(DEBUG_EP_CONTEXT, "0")
DEF_ENV_PARAM(XLNX_EP_CONTEXT_ENABLE_COMPRESSION, "0")
DEF_ENV_PARAM_2(XLNX_model_clone_external_data_threshold, "128", int64_t)
#define MY_LOG(n) LOG_IF(INFO, ENV_PARAM(DEBUG_VITIS_AI_EP) >= n)

DEF_ENV_PARAM_2(XLNX_MD5_SIG_SKIP_OPS, "QuantizeLinear,DequantizeLinear",
                std::vector<std::string>)

#define LOG_VERBOSE(n)                                                         \
  LOG_IF(INFO, ENV_PARAM(XLNX_ONNX_EP_VERBOSE) >= n)                           \
      << "[XLNX_ONNX_EP_VERBOSE] "
#ifdef ENABLE_PYTHON
#  include <pybind11/pybind11.h>
namespace py = pybind11;
#endif
using namespace onnxruntime;

namespace google {
int GetStackTrace(void** result, int max_depth, int skip_count);
bool Symbolize(void* /*pc*/, char* /*out*/, size_t /*out_size*/);
} // namespace google

namespace vaip_core {

static void save_protobuf_message(const fs::path& filename,
                                  const google::protobuf::Message& msg) {
  try {
    google::protobuf::util::JsonPrintOptions options;
    options.add_whitespace = true;
    auto json_str = std::string();
    auto status =
        google::protobuf::util::MessageToJsonString(msg, &json_str, options);
    CHECK(status.ok()) << "cannot write json string:" << msg.DebugString();
    CHECK(std::ofstream(filename).write(&json_str[0], json_str.size()).good())
        << "failed to write " << filename;
  } catch (const std::exception& e) {
    std::cerr << "exception occurs : " << e.what() << "\n";
  }
}

static void load_protobuf_message(const fs::path& filename,
                                  google::protobuf::Message& msg) {
  auto json_str = slurp(filename);
  auto status = google::protobuf::util::JsonStringToMessage(json_str, &msg);
  CHECK(status.ok()) << "cannot parse json string:" << json_str;
}

static void load_protobuf_message_2(const fs::path& filename,
                                    google::protobuf::Message& msg) {
  auto json_str = slurp_if_exists(filename);
  if (!json_str.empty()) {
    auto status = google::protobuf::util::JsonStringToMessage(json_str, &msg);
    CHECK(status.ok()) << "cannot parse json string:" << json_str;
  }
}

static inline void remove_encryption(ConfigProto& proto) {
  proto.clear_encryption_key();
}

static void save_config_json(PassContextImp& context) {
  ConfigProto proto;
  proto.CopyFrom(context.context_proto.config());
  remove_encryption(proto);
  save_protobuf_message(get_cache_file_name(context, "config.json"), proto);
}

static void print_device_subgraph(const PassContextImp& context) {
  LOG_VERBOSE(2) << "dpu subgraph: " << context.context_proto.meta_def_size();
}

static void print_version_verbose(const char* prefix,
                                  const ConfigProto& config) {
  for (auto version_info : config.version().version_infos()) {
    LOG_VERBOSE(1) << prefix << version_info.version()
                   << " :" + version_info.commit();
  }
  LOG_VERBOSE(1) << prefix << "cache_dir: " << config.cache_dir();
  LOG_VERBOSE(1) << prefix << "cache_key: " << config.cache_key();
  for (auto kv : config.provider_options()) {
    LOG_VERBOSE(1) << "provider_options: " << kv.first << " = " << kv.second;
  }
  for (auto kv : config.session_configs()) {
    LOG_VERBOSE(1) << "session_config: " << kv.first << " = " << kv.second;
  }
}

static void pass_context_update_context_json(PassContextImp& context,
                                             gsl::span<char> json_str) {
  auto session_options_saved = context.context_proto.config().session_configs();
  context.context_proto.Clear();
  auto status = google::protobuf::util::JsonStringToMessage(
      &json_str[0], &context.context_proto);
  context.context_proto.mutable_config()->mutable_session_configs()->swap(
      session_options_saved);
  CHECK(status.ok()) << "cannot parse json string:" << &json_str[0];
  print_version_verbose("CACHE VERSION: ", context.get_config_proto());
}
static void update_pass_context_from_context_json_in_cache(
    std::shared_ptr<PassContextImp> context) {
  auto log_dir = context->get_log_dir();
  auto context_json_path = log_dir / "context.json";
  auto context_context_json = context->read_file_c8("context.json");
  auto context_context_json_text = dos2unix(*context_context_json);
  pass_context_update_context_json(*context, context_context_json_text);
}

static ContextProto load_context_json_2(PassContextImp& context) {
  ContextProto ctxProto;
  load_protobuf_message_2(get_cache_file_name(context, "context_dod.json"),
                          ctxProto);
  return ctxProto;
}

static void
collect_stat_and_dump(const PassContextImp& context,
                      const onnxruntime::Graph& onnx_graph) noexcept {
  try {
    auto filename = ENV_PARAM(XLNX_ONNX_EP_REPORT_FILE);
    collect_stat(onnx_graph, context.context_proto);
    if (!filename.empty()) {
      save_protobuf_message(get_cache_file_name(context, filename),
                            get_stat_proto());
    }
    clean_stat();
  } catch (const std::exception& e) {
    std::cerr << "exception occurs : " << e.what() << "\n";
  }
}

static void update_primary_context(std::shared_ptr<PassContextImp> context) {
  auto second_proto = load_context_json_2(*context);
  for (const auto& meta_def : second_proto.meta_def()) {
    context->context_proto.mutable_meta_def()->Add()->CopyFrom(meta_def);
  }
}

static void update_cache(std::shared_ptr<PassContextImp> context,
                         onnxruntime::Graph& graph) {
  auto deferred_write = std::shared_ptr<void>(
      nullptr, [context](void* p) { context->save_context_json(); });
  auto measure_update_cache = context->measure("update_cache");
  auto passes =
      IPass::create_passes(context, context->get_config_proto().passes());
  IPass::run_passes(passes, graph);
  // ##########
  // Workaround for Shell model compiler
  // Need to load meta_def from secondary json and merge with context.json
  // ##########
  update_primary_context(context);
}

static void read_cache(std::shared_ptr<PassContextImp> context) {
  auto measure = context->measure("read_cache");
  update_pass_context_from_context_json_in_cache(context);
}

static std::string get_commit(const AllVersionInfoProto& proto,
                              const std::string& name) {
  for (const auto& iter : proto.version_infos()) {
    if (iter.package_name() == name) {
      return iter.commit();
    }
  }
  return ""; // if a commit is absent in both version info, we assume it matched
}

static bool cache_valid(const PassContextImp& context) {
  ContextProto proto;
  load_protobuf_message(get_cache_file_name(context, "context.json"), proto);
  auto& code_versions = context.get_config_proto().version();
  auto& cache_versions = proto.config().version();
  std::vector<std::string> package_name = {"xcompiler", "vaip"};
  for (const auto& name : package_name) {
    auto code_package_version = get_commit(code_versions, name);
    auto cache_package_version = get_commit(cache_versions, name);
    if (code_package_version != cache_package_version) {
      LOG(WARNING) << name << "'s versions mistached: " << code_package_version
                   << " at code and " << cache_package_version << " at cache";
      return true;
    }
  }
  return true;
}

static bool check_cache_exist(const PassContextImp& context) {
  fs::path cache_file;
  if (ENV_PARAM(ENABLE_TAR_CACHE)) {
    // TODO: support tar.gz also
    cache_file = context.log_dir;
    cache_file += ".tar ";
  } else {
    cache_file = get_cache_file_name(context, "context.json");
  }
  return file_exists(cache_file);
}

static bool check_cache_hit(PassContextImp& context) {
  auto measure_check_cache_hit = context.measure("check_cache_hit");
  if (ENV_PARAM(XLNX_ONNX_EP_DL_ANALYZER_PROFILING) ||
      ENV_PARAM(XLNX_ONNX_EP_DL_ANALYZER_VISUALIZATION))
    return false;
  if (ENV_PARAM(XLNX_ENABLE_CACHE)) {
    return check_cache_exist(context) && cache_valid(context);
  }
  return false;
}

void compile_onnx_model_2(std::shared_ptr<PassContextImp> context,
                          onnxruntime::Graph& graph, const Graph& onnx_graph) {
  bool cache_hit = check_cache_hit(*context);
  if (!cache_hit) {
    update_cache(context, graph);
  } else {
    MY_LOG(1) << "==== cache hit ====";
  }
  auto encryption_key = context->context_proto.config().encryption_key();
  auto session_configs = context->context_proto.config().session_configs();
  read_cache(context);
  context->context_proto.mutable_config()->set_encryption_key(encryption_key);
  auto session_configs_in_cache =
      context->context_proto.mutable_config()->mutable_session_configs();
  session_configs_in_cache->swap(session_configs);
}

static std::string get_dump_md5_file(const std::string& suffix) {
  auto ret = ENV_PARAM(DEBUG_MD5_SIG);
  if (!ret.empty()) {
    ret = ret + suffix;
  }
  return ret;
}
struct MD5Sig {
public:
  MD5Sig(const std::string& suffix)
      : dump_md5_file{get_dump_md5_file(suffix)} {}
  void add(const void* data, size_t numBytes) {
    md5.add(data, numBytes);
    if (str) {
      CHECK(str->write((const char*)data, numBytes).good())
          << "failed to write to dump_md5_file " << dump_md5_file;
    }
  }
  std::string getHash() {
    if (str) {
      str->close();
    }
    return md5.getHash();
  }

public:
  const std::string dump_md5_file;
  MD5 md5 = MD5();
  std::unique_ptr<std::ofstream> str =
      (dump_md5_file.empty() ? nullptr
                             : std::make_unique<std::ofstream>(dump_md5_file));
};

static std::string
get_model_signature_with_graph_inputs_and_outputs(const Graph& onnx_graph) {
  auto md5 = MD5Sig("_with_io.data");
  auto inputs = graph_get_inputs(onnx_graph);
  for (auto& input : inputs) {
    auto input_name = node_arg_get_name(*input);
    md5.add(input_name.data(), input_name.size());

    auto shape = node_arg_get_shape_i64(*input);
    if (shape && !shape->empty()) {
      md5.add(shape->data(), shape->size() * sizeof(shape->at(0)));
    }
  }
  auto outputs = graph_get_outputs(onnx_graph);
  for (auto& output : outputs) {
    auto output_name = node_arg_get_name(*output);
    md5.add(output_name.data(), output_name.size());

    auto shape = node_arg_get_shape_i64(*output);
    if (shape && !shape->empty()) {
      md5.add(shape->data(), shape->size() * sizeof(shape->at(0)));
    }
  }
  return md5.getHash();
}

static std::string get_model_signature(const Graph& onnx_graph) {
  auto md5 = MD5Sig(".data");
  for (auto node_idx : graph_get_node_in_topoligical_order(onnx_graph)) {
    auto node = VAIP_ORT_API(graph_get_node)(onnx_graph, node_idx);
    auto op_type = node_op_type(*node);
    const auto& skip_op = ENV_PARAM(XLNX_MD5_SIG_SKIP_OPS);
    if (std::find(skip_op.begin(), skip_op.end(), op_type) != skip_op.end()) {
      continue;
    }
    CHECK(node != nullptr) << "node_idx " << node_idx << " ";
    auto output = node_get_output_node_args(*node);
    for (auto& node_arg : output) {
      if (node_arg == nullptr) {
        continue;
      }
      if (!node_arg_exists(*node_arg)) {
        continue;
      }
      auto node_arg_name = node_arg_get_name(*node_arg);
      md5.add(node_arg_name.data(), node_arg_name.size());

      auto shape = node_arg_get_shape_i64(*node_arg);
      if (shape && !shape->empty()) {
        md5.add(shape->data(), shape->size() * sizeof(shape->at(0)));
      }
    }
  }
  return md5.getHash();
}

static std::pair<const std::string, const MepConfigTable*>
find_signature_in_meptabel(const ConfigProto& proto,
                           const std::string md5_file_base,
                           const std::string md5_in_memory_a,
                           const std::string md5_in_memory_b,
                           int32_t node_count) {
  for (auto& mep : proto.mep_table()) {
    if (md5_in_memory_a == mep.md5sum_in_memory()) {
      MY_LOG(1) << "find signature in meptable : "             //
                << "model_name :  " << mep.model_name() << " " //
                << "md5sum_in_memory : " << mep.md5sum_in_memory();
      return std::make_pair(md5_in_memory_a, &mep);
    } else if (md5_in_memory_b == mep.md5sum_in_memory_with_io()) {
      MY_LOG(1) << "find signature in meptable : "             //
                << "model_name :  " << mep.model_name() << " " //
                << "md5sum_in_memory_with_io : "
                << mep.md5sum_in_memory_with_io()
                << " model node_count : " << node_count
                << " mep node_count : " << mep.node_count();
      // Also match node count if it's specified in vaip_config.json
      if ((!mep.has_node_count()) || (node_count == mep.node_count())) {
        return std::make_pair(md5_in_memory_b, &mep);
      }
    } else if (!md5_file_base.empty() &&
               md5_file_base == mep.md5sum_on_disk()) {
      MY_LOG(1) << "find signature in meptable : "             //
                << "model_name :  " << mep.model_name() << " " //
                << "md5sum_on_disk : " << mep.md5sum_on_disk();
      return std::make_pair(md5_file_base, &mep);
    }
  }
  MY_LOG(1) << "Can not find signature in meptable , use in memory signature "
            << md5_in_memory_a;
  return std::make_pair(md5_in_memory_a, nullptr);
}
static std::pair<const std::string, const MepConfigTable*>
get_signature_with_meptable(const std::string& model_path,
                            const Graph& onnx_graph, const ConfigProto& proto) {
  auto md5_file_base =
      model_path.empty() ? "" : xir::get_md5_of_file(model_path);
  auto md5_in_memory_a = get_model_signature(onnx_graph);
  auto md5_in_memory_b =
      get_model_signature_with_graph_inputs_and_outputs(onnx_graph);

  const auto& node_indices = graph_get_node_in_topoligical_order(onnx_graph);
  int32_t node_count = (int32_t)node_indices.size();

  MY_LOG(1) << "File base signature : " << md5_file_base;
  MY_LOG(1) << "Algorithm-A: based on topologically ordered signature : "
            << md5_in_memory_a;
  MY_LOG(1) << "Algorithm-B: based on graph inputs/outputs signature : "
            << md5_in_memory_b;
  MY_LOG(1) << "Algorithm-B: node count: " << node_count;
  return find_signature_in_meptabel(proto, md5_file_base, md5_in_memory_a,
                                    md5_in_memory_b, node_count);
}

// NOTE: this function should not read any file in the cache directory, because
// when ENV_PARAM(ENABLE_TAR_CACHE) is on, all files in the cache directory are
// inside the tar file instead, we should not assume that any files exists
// inside the cache directory.
std::shared_ptr<PassContextImp>
initialize_context(const std::string& model_path, const Graph& onnx_graph,
                   const std::vector<vaip_cxx::NodeConstRef>& ep_context_nodes,
                   const char* json_config) {
  std::shared_ptr<PassContextImp> context = std::make_shared<PassContextImp>();
  context->model_path = model_path;
  auto config_proto = ConfigProto();
  if (json_config != nullptr && !std::string(json_config).empty()) {
    Config::merge_config_proto(config_proto, json_config);
  }
  context->cache_dir_set = (config_proto.cache_dir().size() > 0);
  context->is_ep_context_model = !ep_context_nodes.empty();
  // DL Analyzer dumping partition.json and fused.viz.json, or not.
  bool analyzer_visualization_enabled =
      config_proto.has_ai_analyzer_visualization() &&
      config_proto.ai_analyzer_visualization();
  ENV_PARAM(XLNX_ONNX_EP_DL_ANALYZER_VISUALIZATION) =
      analyzer_visualization_enabled;
  // DL Analyzer creating dpu_timestamp_info.json, or not.
  bool analyzer_profiling_enabled = config_proto.has_ai_analyzer_profiling() &&
                                    config_proto.ai_analyzer_profiling();
  ENV_PARAM(XLNX_ONNX_EP_DL_ANALYZER_PROFILING) = analyzer_profiling_enabled;

  Config::add_version_info(config_proto);
  *context->context_proto.mutable_config() = std::move(config_proto);
  auto& model = graph_get_model(onnx_graph);

  auto [md5, mep_table] = get_signature_with_meptable(
      model_path, onnx_graph, context->context_proto.config());

  if (!context->context_proto.config().cache_key().empty()) {
    MY_LOG(1) << "use cache key specified by user "
              << context->context_proto.config().cache_key();
  } else if (VAIP_ORT_API(model_has_meta_data)(model, "vaip_model_md5sum")) {
    auto new_cache_key =
        *VAIP_ORT_API(model_get_meta_data)(model, "vaip_model_md5sum");
    MY_LOG(1) << "use cache key in meta-data " << new_cache_key;
    *context->context_proto.mutable_config()->mutable_cache_key() =
        new_cache_key;
  } else if (ENV_PARAM(XLNX_ENABLE_FILE_BASED_CACHE_KEY) &&
             (!model_path.empty())) {
    auto new_cache_key = xir::get_md5_of_file(model_path);
    MY_LOG(1) << "use cache key on-disk " << new_cache_key;
    *context->context_proto.mutable_config()->mutable_cache_key() =
        new_cache_key;
  } else {
    auto new_cache_key = md5;
    LOG_VERBOSE(1) << "use cache key in memory signature " << new_cache_key;
    *context->context_proto.mutable_config()->mutable_cache_key() =
        new_cache_key;
  }
  // Algorithm-A : based on names of node-args tensor0-names of
  // topologically ordered model-graph Algorithm-B : based on
  // input/output-tensor names overall auto-mapping mechanism will be to use
  // Algorithm-A first, if that fails then use Algorithm-B to identify the
  // model/target
  if (mep_table) {
    context->context_proto.mutable_config()->mutable_provider_options()->insert(
        {"model_name", mep_table->model_name()});
    std::string model_category = "";
    if (mep_table->has_model_category()) {
      model_category = mep_table->model_category();
    }
    context->context_proto.mutable_config()->mutable_provider_options()->insert(
        {"model_category", model_category});

    std::string model_variant = "";
    if (mep_table->has_model_variant()) {
      model_variant = mep_table->model_variant();
    }
    context->context_proto.mutable_config()->mutable_provider_options()->insert(
        {"model_variant", model_variant});

    std::string qos_priority = "";
    if (mep_table->has_qos_priority()) {
      qos_priority = mep_table->qos_priority();

      context->context_proto.mutable_config()
          ->mutable_provider_options()
          ->insert({"qos_priority", qos_priority});
    }
  }
  vaip_core::update_config_by_target(*context->context_proto.mutable_config(),
                                     mep_table);

  auto onnx_path = model_path.empty() ? std::string("N/A") : model_path;
  *context->context_proto.mutable_config()->mutable_onnx_path() = onnx_path;

  if (VAIP_ORT_API(model_has_meta_data)(model, "suffix_counter")) {
    context->suffix_counter =
        std::stoi(*VAIP_ORT_API(model_get_meta_data)(model, "suffix_counter"));
  }
  update_cache_dir(*context);
  // DANGER!
  Model& mutable_model = const_cast<Model&>(model);
  model_set_meta_data(mutable_model, "vaip_log_dir",
                      context->get_log_dir().u8string());

  // log version of binary
  print_version_verbose("EXEC VERISON: ", context->context_proto.config());
  return context;
}
static std::string get_provider_option(const PassContextImp& context,
                                       const std::string& key,
                                       const std::string& default_value) {
  auto& options = context.context_proto.config().provider_options();
  auto it = options.find(key);
  auto ret = std::string();
  if (it == options.end()) {
    ret = default_value;
  } else {
    ret = it->second;
  }
  return ret;
}

static std::string get_ep_cache_context_embed_mode(PassContextImp& context) {
  auto measure_get_ep_cache_context_embed_mode =
      context.measure("get_ep_cache_context_embed_mode");
  auto bytes = context.cache_files_to_tar_mem();
  if (ENV_PARAM(XLNX_EP_CONTEXT_ENABLE_COMPRESSION)) {
    auto measure_compression = context.measure("vaip_core::compress");
    LOG_IF(INFO, ENV_PARAM(DEBUG_EP_CONTEXT))
        << " start compressing ep context " << bytes.size() << " bytes";
    bytes = vaip_core::compress(bytes);
    LOG_IF(INFO, ENV_PARAM(DEBUG_EP_CONTEXT))
        << " ep context is " << bytes.size() << " bytes after compression";
  }
  LOG_IF(INFO, ENV_PARAM(DEBUG_EP_CONTEXT))
      << "embed mode = 1, load cache directory  to tar memory " << bytes.size()
      << " bytes";
  return std::string(bytes.begin(), bytes.end());
}

static std::string get_ep_cache_context_nonembed_mode(PassContextImp& context) {
  auto measure_get_ep_cache_context_embed_mode =
      context.measure("get_ep_cache_context_nonembed_mode");
  auto model_path = context.model_path;
  auto OrtSessionOptionEpContextFilePath = std::filesystem::path(
      get_provider_option(context, "ep_context_file_path", ""));
  if (OrtSessionOptionEpContextFilePath.empty()) {
    OrtSessionOptionEpContextFilePath = model_path;
    OrtSessionOptionEpContextFilePath += "_ctx.onnx";
  }
  LOG_IF(INFO, ENV_PARAM(DEBUG_EP_CONTEXT))
      << "embed mode = 0, save ep context file to "
      << OrtSessionOptionEpContextFilePath.filename();
  auto OrtSessionOptionEpContextFilePath_binay =
      OrtSessionOptionEpContextFilePath.replace_extension(".bin");
  LOG_IF(INFO, ENV_PARAM(DEBUG_EP_CONTEXT))
      << "embed mode = 0, save cache directory to tar file "
      << OrtSessionOptionEpContextFilePath_binay.filename();
  auto bytes = context.cache_files_to_tar_mem();
  if (ENV_PARAM(XLNX_EP_CONTEXT_ENABLE_COMPRESSION)) {
    auto measure_compression = context.measure("vaip_core::compress");
    LOG_IF(INFO, ENV_PARAM(DEBUG_EP_CONTEXT))
        << " start compressing ep context " << bytes.size() << " bytes";
    bytes = vaip_core::compress(bytes);
    LOG_IF(INFO, ENV_PARAM(DEBUG_EP_CONTEXT))
        << " ep context is " << bytes.size() << " bytes after compression";
  }
  vaip_core::dump_binary(OrtSessionOptionEpContextFilePath_binay, bytes);
  return OrtSessionOptionEpContextFilePath.filename().u8string();
}

static std::string get_ep_cache_context(PassContextImp& context,
                                        bool embed_mode) {
  auto ret = std::string();
  if (embed_mode) {
    ret = get_ep_cache_context_embed_mode(context);
  } else {
    ret = get_ep_cache_context_nonembed_mode(context);
  }
  return ret;
}

#if VAIP_ORT_API_MAJOR < 6
static std::string escape_json(const std::string& s) {
  std::ostringstream o;
  for (auto c = s.cbegin(); c != s.cend(); c++) {
    if (*c == '"' || *c == '\\' || ('\x00' <= *c && *c <= '\x1f')) {
      o << "\\u" << std::hex << std::setw(4) << std::setfill('0')
        << static_cast<int>(*c);
    } else {
      o << *c;
    }
  }
  return o.str();
}
static std::string get_nodes(PassContextImp& context) {
  std::ostringstream o;
  auto log_dir = context.get_log_dir();
  auto cache_dir = log_dir.parent_path().u8string();
  auto cache_key = log_dir.filename().u8string();
  o << "{"
       "\"backend_cache_dir\":"                  //
    << "\"" << escape_json(cache_dir) << "\",\n" //
    << "\"backend_cache_key\":"                  //
    << "\"" << escape_json(cache_key) << "\"\n"  //
    << "}";
  return o.str();
}
#endif
static onnxruntime::Node*
create_ep_context_node(vaip_core::ExecutionProviderConcrete* ep) {
  CHECK(ep != nullptr);
  CHECK(ep->get_fused_node() != nullptr);
  auto p_context = dynamic_cast<PassContextImp*>(ep->get_context().get());
  CHECK(p_context != nullptr);
  auto& context = *p_context;

  if (!context.ep_context_model_) {
    context.ep_context_model_ =
        vaip_cxx::Model::create(context.model_path, {{"ai.onnx", 21}});
  }
  auto ep_context_graph = context.ep_context_model_->main_graph();
  auto fused_node = vaip_cxx::NodeConstRef::from_node(ep_context_graph,
                                                      *ep->get_fused_node());
  auto op_type = "EPContext";
  auto op_domain = "com.microsoft";
  auto name = fused_node.name();
  auto description = "description";
  auto input_args = fused_node.inputs();
  auto output_args = fused_node.outputs();
  auto attrs = NodeAttributesBuilder();
  auto index = node_get_attr_int(fused_node, "index");
  attrs.add("index", index);
  int64_t main_context = index == 0 ? 1 : 0;
  attrs.add("main_context", main_context);
  int64_t embed_mode =
      get_provider_option(context, "ep_context_embed_mode", "1") == "1" ? 1 : 0;
  attrs.add("embed_mode", embed_mode);
  attrs.add("source", std::string("VitisAIExecutionProvider"));
  attrs.add("log_dir", context.log_dir.u8string());
  attrs.add("onnx_model_filename", context.model_path.u8string());
  attrs.add("partition_name", name);
  attrs.add("enable_compression",
            (int64_t)ENV_PARAM(XLNX_EP_CONTEXT_ENABLE_COMPRESSION));
  auto& version_infos = context.get_config_proto().version();
  for (const auto& version_info : version_infos.version_infos()) {
    auto lib_name = "version_of_" + version_info.package_name();
    attrs.add(lib_name, version_info.version());
    lib_name = "version_id_of_" + version_info.package_name();
    attrs.add(lib_name, version_info.commit());
  }
#if VAIP_ORT_API_MAJOR < 6
  auto notes = get_nodes(context);
  attrs.add("notes", notes);
#endif
  auto ep_cache_context = std::string();
  if (main_context) {
    ep_cache_context = get_ep_cache_context(context, embed_mode != 0);
  }
  attrs.add("ep_cache_context", ep_cache_context);
  auto ret = vaip_cxx::GraphRef(ep_context_graph)
                 .add_node(name, op_domain, op_type, description, input_args,
                           output_args, attrs.build());
  LOG_IF(INFO, ENV_PARAM(DEBUG_EP_CONTEXT)) << "add ep node:" << ret;
  return ret.ptr();
}

extern "C" VAIP_DLL_SPEC int create_ep_context_nodes_c(
#if VAIP_ORT_API_MAJOR < 6
    onnxruntime::Graph& /*ep_context_graph unused to deleted*/,
#endif
    const std::vector<std::unique_ptr<vaip_core::ExecutionProvider>>& eps,
    vaip_core::DllSafe<std::vector<Node*>>* ret_value) {
  std::vector<Node*> ret;
  if (eps.empty()) {
    *ret_value =
        vaip_core::DllSafe<std::vector<Node*>>(new std::vector<Node*>());
    return 1;
  }
  auto ep =
      dynamic_cast<vaip_core::ExecutionProviderConcrete*>(eps.front().get());
  if (ep->get_context()->get_is_ep_context_model()) {
    *ret_value =
        vaip_core::DllSafe<std::vector<Node*>>(new std::vector<Node*>());
    return 1;
  }
  auto p_context = dynamic_cast<PassContextImp*>(ep->get_context().get());
  CHECK(p_context != nullptr);
  auto& context = *p_context;
  auto deferred_write = std::shared_ptr<void>(
      nullptr, [&context](void* p) { context.save_context_json(); });
  auto measure_create_ep_context_nodes =
      context.measure("create_ep_context_nodes");
  ret.reserve(eps.size());
  for (auto& ep : eps) {
    ret.push_back(create_ep_context_node(
        dynamic_cast<vaip_core::ExecutionProviderConcrete*>(ep.get())));
  }
  *ret_value = vaip_core::DllSafe<std::vector<Node*>>(
      new std::vector<Node*>(std::move(ret)));
  return 1;
}

static std::vector<vaip_cxx::NodeConstRef>
get_ep_context_nodes(vaip_cxx::GraphConstRef onnx_graph) {
  auto ret = std::vector<vaip_cxx::NodeConstRef>();
  auto nodes = onnx_graph.nodes();
  for (auto node : nodes) {
    if (node.op_type() == "EPContext" && node.op_domain() == "com.microsoft") {
      if (node.has_attr("source") &&
          node.get_attr_string("source") == "VitisAIExecutionProvider") {
        ret.push_back(node);
      }
    }
  }
  return ret;
}

static void update_meta_def_from_ep_node(vaip_cxx::NodeConstRef node,
                                         MetaDefProto& meta_def) {
  meta_def.mutable_inputs()->Clear();
  for (auto input : node.inputs()) {
    if (input.has_value()) {
      meta_def.add_inputs(input->name());
    }
  }
  meta_def.mutable_outputs()->Clear();
  auto output_name = std::string();
  for (auto output : node.outputs()) {
    if (output.has_value()) {
      if (output_name.empty()) {
        // use the first node arg name.
        output_name = output->name();
      }
      meta_def.add_outputs(output->name());
    }
  }
  meta_def.mutable_nodes()->Clear();
  CHECK(!output_name.empty())
      << "EPContext node must have at least one output.";
  meta_def.add_nodes(output_name);
  meta_def.mutable_constant_initializers()->Clear();
  return;
}
static std::optional<vaip_cxx::NodeConstRef> get_main_ep_context_node(
    std::vector<vaip_cxx::NodeConstRef>& ep_context_nodes) {
  std::optional<vaip_cxx::NodeConstRef> ret = std::nullopt;
  auto count_main_context = 0;
  for (auto node : ep_context_nodes) {
    if (node.has_attr("main_context") && node.get_attr_int("main_context")) {
      count_main_context++;
      ret = node;
    }
  }
  CHECK_EQ(count_main_context, 1)
      << "There must be exactly one main EPContext node. The EP context model "
         "have "
      << count_main_context << " main EPContext nodes.";
  return ret;
}
static void
store_cache_directory_from_main_node(PassContextImp& context,
                                     vaip_cxx::NodeConstRef main_node) {

  CHECK(main_node.has_attr("ep_cache_context"))
      << " main EPContext has not ep_cache_context attr";
  auto ep_cache_context = main_node.get_attr_string("ep_cache_context");
  int64_t enable_compression =
      main_node.has_attr("enable_compression")
          ? main_node.get_attr_int("enable_compression")
          : 0;
  int64_t ep_embed_mode = 1; // default embed_mode = 1
  if (main_node.has_attr("embed_mode")) {
    ep_embed_mode = main_node.get_attr_int("embed_mode");
  }
  if (ep_embed_mode) {
    LOG_IF(INFO, ENV_PARAM(DEBUG_EP_CONTEXT))
        << "embed mode = 1, load ep context " << ep_cache_context.size()
        << " bytes";
    auto tar_mem = std::vector<char>();
    auto tar_mem_span = gsl::span<const char>();
    if (enable_compression) {
      LOG_IF(INFO, ENV_PARAM(DEBUG_EP_CONTEXT))
          << " uncompressed ep context, " << ep_cache_context.size()
          << " bytes";
      tar_mem = vaip_core::uncompress(gsl::span<const char>(
          ep_cache_context.data(), ep_cache_context.size()));
      tar_mem_span = tar_mem;
      LOG_IF(INFO, ENV_PARAM(DEBUG_EP_CONTEXT))
          << " ep context is " << tar_mem.size()
          << " bytes after uncompression";
    } else {
      tar_mem_span = gsl::span<const char>(ep_cache_context.data(),
                                           ep_cache_context.size());
    }
    tar_mem = vaip_core::uncompress(
        gsl::span<const char>(tar_mem_span.data(), tar_mem_span.size()));
    context.tar_mem_to_cache_files(tar_mem.data(), tar_mem.size());
  } else {
    auto ep_context_binary_file = std::filesystem::path();
    if (context.model_path.empty()) {
      ep_context_binary_file = ep_cache_context;
    } else {
      ep_context_binary_file =
          context.model_path.parent_path() / ep_cache_context;
    }
    auto tar_mem = vaip_core::slurp_binary_c8(ep_context_binary_file);
    LOG_IF(INFO, ENV_PARAM(DEBUG_EP_CONTEXT))
        << " read ep context, " << tar_mem.size() << " bytes from "
        << ep_context_binary_file;
    if (enable_compression) {
      LOG_IF(INFO, ENV_PARAM(DEBUG_EP_CONTEXT))
          << " start to uncompress, " << tar_mem.size() << " bytes ";
      tar_mem = vaip_core::uncompress(tar_mem);
      LOG_IF(INFO, ENV_PARAM(DEBUG_EP_CONTEXT))
          << " ep context is " << tar_mem.size()
          << " bytes after uncompression";
    }
    context.tar_mem_to_cache_files(tar_mem.data(), tar_mem.size());
  }
  LOG_IF(INFO, ENV_PARAM(DEBUG_EP_CONTEXT))
      << " extract memory cache files to  " << context.get_log_dir();
}

static int64_t get_ep_context_index(const vaip_cxx::NodeConstRef& node) {
  CHECK(node.has_attr("index"))
      << "EPContext Node has no index attr, EPContext node : " << node;
  return node.get_attr_int("index");
}

static std::vector<std::unique_ptr<ExecutionProvider>>
create_execution_providers_from_ep_context_nodes(
    std::shared_ptr<PassContextImp> context,
    std::vector<vaip_cxx::NodeConstRef> ep_context_nodes) {
  CHECK_EQ(ep_context_nodes.size(), context->context_proto.meta_def_size());
  auto size = ep_context_nodes.size();
  auto ret = std::vector<std::unique_ptr<ExecutionProvider>>();
  ret.reserve(size);

  std::sort(
      ep_context_nodes.begin(), ep_context_nodes.end(),
      [](const vaip_cxx::NodeConstRef& a, const vaip_cxx::NodeConstRef& b) {
        return get_ep_context_index(a) < get_ep_context_index(b);
      });

  for (auto idx = 0u; idx < size; ++idx) {
    auto node = ep_context_nodes[idx];
    auto index = get_ep_context_index(node);
    CHECK_EQ(index, idx) << "EPContext Node index mismatch, EPContext node : "
                         << node;
    auto meta_def_index = idx;
    auto& meta_def = *context->context_proto.mutable_meta_def(meta_def_index);
    update_meta_def_from_ep_node(node, meta_def);
    auto device = meta_def.device();
    auto plugin_name = std::string("vaip_custom_op_") + device;
    ret.emplace_back(ExecutionProviderConcrete::create(
        plugin_name, g_static_plugin_func_set_ptr, context, meta_def));
  }
  return ret;
}
static std::vector<std::unique_ptr<ExecutionProvider>>
restore_execution_providers_from_ep_context_model(
    vaip_cxx::GraphConstRef onnx_graph, std::shared_ptr<PassContextImp> context,
    std::vector<vaip_cxx::NodeConstRef> ep_context_nodes) {
  auto measture =
      context->measure("restore_execution_providers_from_ep_context_model");
  LOG_IF(INFO, ENV_PARAM(DEBUG_EP_CONTEXT))
      << "Running EP context onnx model , restore ExecutionProviders from EP "
         "context model";
  // untar the cache directory.
  auto main_node = get_main_ep_context_node(ep_context_nodes);
  CHECK(main_node) << " no main EPContext node";
  store_cache_directory_from_main_node(*context, main_node.value());
  update_pass_context_from_context_json_in_cache(context);
  return create_execution_providers_from_ep_context_nodes(context,
                                                          ep_context_nodes);
}

static std::vector<std::unique_ptr<ExecutionProvider>>
compile_onnx_model_internal(
    const Graph& onnx_graph,
    const std::vector<vaip_cxx::NodeConstRef>& ep_context_nodes,
    std::shared_ptr<PassContextImp> context) {
  auto measure_compile_onnx_model_internal =
      context->measure("compile_onnx_model_internal");
  auto ret = std::vector<std::unique_ptr<ExecutionProvider>>();
  auto is_ep_context_model = !ep_context_nodes.empty();
  auto measure_after_compile_onnx_model_2 = std::unique_ptr<PassContextTimer>();
  if (is_ep_context_model) {
    ret = restore_execution_providers_from_ep_context_model(onnx_graph, context,
                                                            ep_context_nodes);
  } else {
    auto measure_before_compile_onnx_model_2 =
        context->measure("before_compile_onnx_model_internal");
    auto& model = graph_get_model(onnx_graph);
    auto cloned_model = model_clone(
        model, VAIP_PROVIDER_OPTION(*context,
                                    XLNX_model_clone_external_data_threshold));
    auto& cloned_graph = VAIP_ORT_API(model_main_graph)(*cloned_model);
    auto deferred_collect =
        std::shared_ptr<void>(nullptr, [context, &onnx_graph](void* p) {
          collect_stat_and_dump(*context, onnx_graph);
        });
    measure_before_compile_onnx_model_2 = nullptr;
    compile_onnx_model_2(context, cloned_graph, onnx_graph);
    measure_after_compile_onnx_model_2 =
        context->measure("after_compile_onnx_model_internal");
    ret.reserve(context->context_proto.meta_def_size());
    for (auto& meta_def : context->context_proto.meta_def()) {
      auto& device = meta_def.device();
      auto plugin_name = std::string("vaip_custom_op_") + device;
      ret.emplace_back(ExecutionProviderConcrete::create(
          plugin_name, g_static_plugin_func_set_ptr, context, meta_def));
    }
  }
  return ret;
}

static std::vector<std::string> GetStackTrace() {
  std::vector<std::string> stack_strings;

  void* stack[32];
  // +2 to exclude this function and compile_fatal_func.
  const int depth =
      google::GetStackTrace(stack, sizeof(stack) / sizeof(stack[0]), 2);
  for (auto i = 0; i < depth; ++i) {
    auto pc = stack[i];
    const char* symbol = "(unknown)";
    char symbolized[1024]; // Big enough for a sane symbol.
    // Symbolizes the previous address of pc because pc may be in the
    // next function.
    if (google::Symbolize(reinterpret_cast<char*>(pc) - 1, symbolized,
                          sizeof(symbolized))) {
      symbol = symbolized;
    }
    stack_strings.push_back(std::string(symbol));
  }
  return stack_strings;
}

struct GlogFatalException : public std::exception {
public:
  virtual const char* what() const throw() { return m.c_str(); }
  std::string m;
  std::vector<std::string> stacks;
};

static void compile_fatal_func() {
  GlogFatalException e;
  e.stacks = GetStackTrace();
  for (auto&& t : e.stacks) {
    e.m += std::string(t) + "\n";
  }
  throw e;
}

static std::ostream& operator<<(std::ostream& s,
                                const std::vector<int64_t>& v) {
  s << "[";
  for (auto c = 0u; c < v.size(); ++c) {
    if (c != 0) {
      s << "x";
    }
    s << v[c];
  }
  s << "]";
  return s;
}
static bool is_cpu_only_inference(const PassContextImp& context) {
  auto ret = false;
  if (context.context_proto.meta_def_size() == 0) {
    ret = true;
  }
  return ret;
}
std::vector<std::unique_ptr<ExecutionProvider>>
compile_onnx_model_3(const std::string& model_path, const Graph& onnx_graph,
                     const char* json_config) {
  if (ENV_PARAM(XLNX_ENABLE_SKIP_FATAL)) {
    // clang warning: cannot initialize a parameter of type
    // 'google::logging_fail_func_t' (aka 'void (*)()
    // __attribute__((noreturn))') with an rvalue of type 'void
    // (*)()'
    google::InstallFailureFunction(
        (google::logging_fail_func_t)&compile_fatal_func);
  }
  auto graph_inputs = graph_get_inputs(onnx_graph);
  auto graph_outputs = graph_get_outputs(onnx_graph);

  LOG(INFO) << "Vitis AI EP Load ONNX Model Success";
  LOG(INFO) << "Graph Input Node Name/Shape (" << graph_inputs.size() << ")";
  for (auto& input : graph_inputs) {
    auto shape = node_arg_get_shape_i64(*input);
    if (shape != nullptr) {
      LOG(INFO) << "\t " << node_arg_get_name(*input) << " : "
                << *(shape.get());
    } else {
      LOG(INFO) << "\t " << node_arg_get_name(*input) << " : []";
    }
  }
  LOG(INFO) << "Graph Output Node Name/Shape (" << graph_outputs.size() << ")";
  for (auto& output : graph_outputs) {
    auto shape = node_arg_get_shape_i64(*output);
    if (shape != nullptr) {
      LOG(INFO) << "\t " << node_arg_get_name(*output) << " : "
                << *(shape.get());
    } else {
      LOG(INFO) << "\t " << node_arg_get_name(*output) << " : []";
    }
  }

  static std::mutex mtx;
  std::lock_guard<std::mutex> t_lock(mtx);
  auto ep_context_nodes = get_ep_context_nodes(onnx_graph);
  auto context =
      initialize_context(model_path, onnx_graph, ep_context_nodes, json_config);
  auto deferred_write = std::shared_ptr<void>(
      nullptr, [context](void* p) { context->save_context_json(); });
  auto measture_compile_onnx_model_3 = context->measure("compile_onnx_model_3");
  // we cannot use get_cache_filename because cache might be a tar file in
  // memory instead of a physical directory.
  bool in_mem = context->cache_in_mem();
  std::unique_ptr<WithFileLock> lock;
  if (!in_mem) {
    lock = std::make_unique<WithFileLock>(
        (context->log_dir / ".lock").u8string().c_str());
  }
  (void)lock;
  auto p_cpu_usage = CreateICPUUsage();
  std::vector<std::unique_ptr<ExecutionProvider>> ret{};
  try {
    ret = compile_onnx_model_internal(onnx_graph, ep_context_nodes, context);
  } catch (const GlogFatalException& e) {
    for (auto&& s : e.stacks) {
      context->context_proto.add_stacks(s);
    }
    LOG(INFO) << "Catch fatal exception, skip this subgraph. Set "
                 "XLNX_ENABLE_SKIP_FATAL=0 to stop skip.\n"
              << e.what();
  }
#ifdef ENABLE_PYTHON
  catch (py::error_already_set& e) {
    (void)e; // suppress unused variable
    if (ENV_PARAM(XLNX_ENABLE_SKIP_FATAL)) {
      LOG(INFO) << " catch pybind11 exception, skip this subgraph:  maybe not "
                   "found vaip python module";
    } else {
      LOG(INFO) << " catch pybind11 exception, maybe not found vaip python "
                   "module , please throw detail message for "
                   "developer";
      abort();
    }
  }
#endif
  catch (const std::exception& e) {
    if (ENV_PARAM(XLNX_ENABLE_SKIP_FATAL)) {
      LOG(INFO) << " catch other exception, skip this subgraph: " << e.what();
    } else {
      LOG(INFO) << " catch exception : " << e.what();
      abort();
    }
  } catch (...) {
    if (ENV_PARAM(XLNX_ENABLE_SKIP_FATAL)) {
      LOG(INFO) << " unknow exception";
    } else {
      LOG(INFO) << " unknow exception";
      abort();
    }
  }

  {
    context->context_proto.clear_cpu_usage();
    auto usage = context->context_proto.add_cpu_usage();
    auto avg_cpu_usage = p_cpu_usage->GetUsage();
    auto peak_working_set_size =
        (float)GetPeakWorkingSetSize() / 1024 / 1024; // MB
    usage->set_avg_cpu_util(avg_cpu_usage);
    usage->set_mem_peak_working_set_size(peak_working_set_size);
    LOG(INFO) << "AVG CPU Usage " << avg_cpu_usage << "%";
    LOG(INFO) << "Peak Working Set size " << peak_working_set_size << " MB";
  }

  print_device_subgraph(*context);
  auto disable_cpu_only =
      context->get_provider_option("vaip_disable_cpu_only_inference", "0");
  if (disable_cpu_only == "1") {
    if (is_cpu_only_inference(*context)) {
      LOG(ERROR) << "[Vitis AI EP][DISABLE CPU ONLY] The model's NPU "
                    "offload is 0";
      abort();
    }
  }
  return ret;
}

static std::shared_ptr<PassContextImp>
initialize_context_for_graph_optimizer(const std::string& model_path,
                                       const Graph& onnx_graph,
                                       const char* json_config) {
  std::shared_ptr<PassContextImp> context = std::make_shared<PassContextImp>();
  auto config_proto = ConfigProto();

  if (json_config != nullptr && !std::string(json_config).empty()) {
    Config::merge_config_proto(config_proto, json_config);
  }
  *context->context_proto.mutable_config() = std::move(config_proto);
  auto& model = graph_get_model(onnx_graph);

  if (!context->context_proto.config().cache_key().empty()) {
    LOG(INFO) << "use cache key "
              << context->context_proto.config().cache_key();
  } else if (VAIP_ORT_API(model_has_meta_data)(model, "vaip_model_md5sum")) {
    *context->context_proto.mutable_config()->mutable_cache_key() =
        *VAIP_ORT_API(model_get_meta_data)(model, "vaip_model_md5sum");
  } else if (!model_path.empty()) {
    *context->context_proto.mutable_config()->mutable_cache_key() =
        xir::get_md5_of_file(model_path);
  }
  // update cache key
  auto& cache_key = context->context_proto.config().cache_key();
  if (cache_key.empty()) {
    LOG(WARNING) << "the onnx model have no valid module path and "
                    "json_config.cache_key is not set.";
    return {};
  }
  *context->context_proto.mutable_config()->mutable_cache_key() =
      context->context_proto.config().cache_key() + ".opt";
  // update cache dir
  update_cache_dir(*context);
  save_config_json(*context);
  return context;
}

int optimize_onnx_model(const std::filesystem::path& model_path_in,
                        const std::filesystem::path& model_path_out,
                        const char* json_config) {
  auto model_in = model_load(model_path_in.u8string());
  auto& graph = VAIP_ORT_API(model_main_graph)(*model_in);
  graph_resolve(graph);
  model_set_meta_data(*model_in, "vaip_model_md5sum",
                      xir::get_md5_of_file(model_path_in.u8string()));
  auto context = initialize_context_for_graph_optimizer(
      model_path_in.u8string(), graph, json_config);

  if (!check_cache_hit(*context)) {
    context->add_context_resource(
        "__current_graph", std::shared_ptr<void>((void*)&graph, [](void*) {}));
    update_cache(context, graph);

    auto new_graph =
        (Graph*)context->get_context_resource("__current_graph").get();
    auto& new_model =
        const_cast<onnxruntime::Model&>(graph_get_model(*new_graph));
    model_set_meta_data(new_model, "suffix_counter",
                        std::to_string(context->suffix_counter));

    auto model_data = model_path_out;
    model_data.replace_extension(".dat");
    VAIP_ORT_API(graph_save)
    (*new_graph, model_path_out.u8string(), model_data.u8string(),
     std::numeric_limits<size_t>::max());
  }
  return 0;
}

void initialize_graph_optimizer(const std::string& json_path) {
  LOG(FATAL) << "initialize_graph_optimizer todo";
}

constexpr const uint8_t kXCompiler = 1;
// constexpr const uint8_t kDD = 2;
constexpr const uint8_t kVAIML = 4;

[[maybe_unused]] static std::string
extract_xmodel_name(const std::string& context_cache_str) {
  nlohmann::json j_obj = nlohmann::json::parse(context_cache_str);
  if (!j_obj.contains("metaDef")) {
    return "";
  }
  // FIXME: duplicate hashing.
  const nlohmann::json& meta_def_arr = j_obj["metaDef"];
  if (meta_def_arr.empty()) {
    return "";
  }
  // TODO: more cases.
  const nlohmann::json& meta_def_0 = meta_def_arr.at(0);
  if (meta_def_0.contains("dpuParam")) {
    return meta_def_0["dpuParam"]["compiledXmodel"];
  } else {
    return "";
  }
}

[[maybe_unused]] static bool single_file_matches(const fs::path& file_fs_path,
                                                 const std::string& prefix,
                                                 const std::string& file_ext) {
  const fs::path& leaf_name = file_fs_path.filename();
  if (!prefix.empty()) {
    // TODO: platform (Linux, Windows) compatibility.
    if (leaf_name.string().compare(0, prefix.length(), prefix) != 0) {
      return false;
    }
  }
  if (!file_ext.empty()) {
    // TODO: platform (Linux, Windows) compatibility.
    if (leaf_name.extension().compare(file_ext) != 0) {
      return false;
    }
  }
  return true;
}

[[maybe_unused]] static void walk_directory_for_matched_files(
    const fs::path& dir_fs_path, const std::string& prefix,
    const std::string& file_ext, std::vector<fs::path>& result_fs_paths) {
  if (!fs::is_directory(dir_fs_path)) {
    if (single_file_matches(dir_fs_path, prefix, file_ext)) {
      // FIXME:
      // Windows may have multiple logical disks.
      // On a Windows node, the system temp path is "C:\temp\...",
      // on another, the system temp path is "D:\temp\...".
      result_fs_paths.push_back(fs::absolute(dir_fs_path));
    }
    return;
  }
  for (const auto& dir_entry : fs::recursive_directory_iterator(dir_fs_path)) {
    if (dir_entry.is_directory()) {
      continue;
    }
    const auto& curr_path = dir_entry.path();
    if (single_file_matches(curr_path, prefix, file_ext)) {
      result_fs_paths.push_back(fs::absolute(curr_path));
    }
  }
}

[[maybe_unused]] static const IPass* find_pass_instance_by_name(
    const std::shared_ptr<PassContextImp>& p_pass_context,
    const std::string& pass_name) {
  for (auto it = p_pass_context->current_pass_stack.cbegin();
       it != p_pass_context->current_pass_stack.cend(); ++it) {
    if ((*it)->name() == pass_name) {
      return *it;
    }
  }
  return nullptr;
}

[[maybe_unused]] static const PassProto*
find_pass_config_by_name(const std::shared_ptr<PassContextImp>& p_pass_context,
                         const std::string& pass_name) {
  const auto& config_proto = p_pass_context->get_config_proto();
  for (int i = 0; i < config_proto.passes_size(); ++i) {
    const auto& pass_proto = config_proto.passes(i);
    if (pass_proto.name() == pass_name) {
      return &pass_proto;
    }
  }
  return nullptr;
}

[[maybe_unused]] static std::string
get_vaiml_model_path(const std::shared_ptr<PassContextImp>& p_pass_context) {
  std::string vaiml_model_path{""};
  const auto* p_pass_proto =
      find_pass_config_by_name(p_pass_context, "vaiml_partition");
  if (p_pass_proto != nullptr) {
    const auto& vaiml_proto = p_pass_proto->vaiml_config();
    if (vaiml_proto.has_vaiml_model_path()) {
      vaiml_model_path = vaiml_proto.vaiml_model_path();
    }
  }
  if (vaiml_model_path.empty()) {
    for (auto& meta_def : p_pass_context->context_proto.meta_def()) {
      if (meta_def.has_vaiml_param()) {
        vaiml_model_path = meta_def.vaiml_param().vaiml_model_path();
        break;
      }
    }
  }
  if (vaiml_model_path.empty()) {
#if 0
#  ifdef _WIN32
    return "C:\\amd\\voe\\binary-modules\\ResNet.flexml";
#  else
    return "./vaiml_partition.flexml";
#  endif
#endif
    // https://gitenterprise.xilinx.com/VitisAI/vaip/blob/a96f9e3d07527ee27de69f2b56cc5161cff50486/vaip_custom_op_vaiml/src/custom_op.hpp#L126
    vaiml_model_path = "./vaiml_par_0/";
  }
  return vaiml_model_path;
}

#if 0
[[maybe_unused]] static std::string get_vaiml_unarchive_path(IPass* p_pass) {
  const auto& vaiml_proto = p_pass->get_pass_proto().vaiml_config();
  const auto& config_proto = p_pass->get_context()->get_config_proto();
  const auto& onnx_model_path = config_proto.onnx_path();
  std::string onnx_model_name = fs::path(onnx_model_path).stem().string();
  if (vaiml_proto.has_vaiml_unarchive_path()) {
    return vaiml_proto.vaiml_unarchive_path() + '/' + onnx_model_name;
  } else {
    bool is_single_archive =
        vaiml_proto.has_single_archive() && vaiml_proto.single_archive();
    if (is_single_archive) {
      return "./" + onnx_model_name;
    }
    std::string temp_path = fs::temp_directory_path().string() + '/';
#  ifdef _WIN32
    temp_path += std::string(getenv("USERNAME"));
#  else
    temp_path += std::string(getlogin());
#  endif
    return temp_path + '/' + onnx_model_name;
  }
}

[[maybe_unused]] static std::string get_vaiml_unarchive_path(
    const std::shared_ptr<PassContextImp>& p_pass_context) {
  const auto* p_pass_proto =
      find_pass_config_by_name(p_pass_context, "vaiml_partition");
  if (p_pass_proto == nullptr) {
    return "";
  }
  const auto& vaiml_proto = p_pass_proto->vaiml_config();
  const auto& config_proto = p_pass_context->get_config_proto();
  const auto& onnx_model_path = config_proto.onnx_path();
  std::string onnx_model_name = fs::path(onnx_model_path).stem().string();
  if (vaiml_proto.has_vaiml_unarchive_path()) {
    return vaiml_proto.vaiml_unarchive_path() + '/' + onnx_model_name;
  } else {
    bool is_single_archive =
        vaiml_proto.has_single_archive() && vaiml_proto.single_archive();
    if (is_single_archive) {
      return "./" + onnx_model_name;
    }
#  if 0
    std::string temp_path = fs::temp_directory_path().string() + '/';
#    ifdef _WIN32
    temp_path += std::string(getenv("USERNAME"));
#    else
    temp_path += std::string(getlogin());
#    endif
#  endif
    return "./" + onnx_model_name;
  }
}
#endif

void get_compilation_cache(const std::string& model_path, const Graph& graph,
                           const char* json_config, uint8_t compiler_codes,
                           std::string& cache_dir, std::string& cache_key,
                           std::string& cache_data) {}

void restore_compilation_cache(const std::string& cache_dir,
                               const std::string& cache_key,
                               const std::string& cache_data,
                               const std::string& model_path) {}
thread_local const void* g_state = nullptr;
thread_local vaip_core::DllSafe<std::string> (*g_get_config_entry)(
    const void* state, const char* entry_name) = nullptr;

int vitisai_ep_on_run_start(
    const std::vector<std::unique_ptr<vaip_core::ExecutionProvider>>& eps,
    const void* state,
    vaip_core::DllSafe<std::string> (*get_config_entry)(
        const void* state, const char* entry_name)) {
  if (eps.empty()) {
    return 1;
  }
  auto ep =
      dynamic_cast<vaip_core::ExecutionProviderConcrete*>(eps.front().get());
  auto p_context =
      dynamic_cast<vaip_core::PassContextImp*>(ep->get_context().get());
  CHECK(p_context != nullptr);
  g_state = state;
  g_get_config_entry = get_config_entry;
  return 0;
}

int vitisai_ep_set_ep_dynamic_options(
    const std::vector<std::unique_ptr<vaip_core::ExecutionProvider>>& eps,
    const char* const* keys, const char* const* values, size_t kv_len) {
  if (eps.empty()) {
    return 1;
  }
  auto ep =
      dynamic_cast<vaip_core::ExecutionProviderConcrete*>(eps.front().get());
  auto p_context =
      dynamic_cast<vaip_core::PassContextImp*>(ep->get_context().get());
  CHECK(p_context != nullptr);
  std::lock_guard<std::mutex> lock(p_context->ep_dynamic_options_lock);
  for (auto i = 0; i < kv_len; i++) {
    auto key = std::string(keys[i]);
    auto value = std::string(values[i]);
    if (key == "ep.dynamic.workload_type")
      p_context->update_all_qos(value);
  }
  return 0;
}
} // namespace vaip_core

extern "C" VAIP_DLL_SPEC int vitisai_ep_on_run_start_c(
    const std::vector<std::unique_ptr<vaip_core::ExecutionProvider>>& eps,
    const void* state,
    vaip_core::DllSafe<std::string> (*get_config_entry)(
        const void* state, const char* entry_name)) {
  return vaip_core::vitisai_ep_on_run_start(eps, state, get_config_entry);
}

extern "C" VAIP_DLL_SPEC int vitisai_ep_set_ep_dynamic_options_c(
    const std::vector<std::unique_ptr<vaip_core::ExecutionProvider>>& eps,
    const char* const* keys, const char* const* values, size_t kv_len) {
  return vaip_core::vitisai_ep_set_ep_dynamic_options(eps, keys, values,
                                                      kv_len);
}
