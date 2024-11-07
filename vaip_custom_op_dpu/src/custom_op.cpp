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
#include "onnxruntime_api.hpp"

#include <glog/logging.h>
#include <sstream>
#include <fstream>
#include <vaip/vaip.hpp>
#include "custom_op.hpp"
#include "ort_tensor_buffer.hpp"
#include "schedule.hpp"

#include "./graph_holder.hpp"
#include "vitis/ai/env_config.hpp"
#include "vitis/ai/profiling.hpp"
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <thread>
#include <unordered_map>
#include <vitis/ai/dim_calc.hpp>
#include <vitis/ai/weak.hpp>
#include <xir/graph/graph.hpp>
#include <xir/attrs/attrs.hpp>
#include "dlanalyzer.hpp"
#include "../../vaip/src/qos_updater.hpp"

#ifdef VART_IN_BUILD_TREE
#include <vitis/ai/trace.hpp>
#else
#include <vart/trace/trace.hpp>
#endif
// clang-format on
#ifdef _WIN32
// graph-engine should not include <xrt.h> in public header files.
// suppress warning, macro redefinition NOMINMAX
#  pragma warning(push)
#  pragma warning(disable : 4005)
#  pragma warning(pop)
#endif
#ifdef ENABLE_XRT_SHARED_CONTEXT
#  include "../../xrt_shared_context/xrt_shared_context.hpp"
DEF_ENV_PARAM(USE_GRAPH_ENGINE, "1");
#else
DEF_ENV_PARAM(USE_GRAPH_ENGINE, "0");
#endif
DEF_ENV_PARAM(DEBUG_VITIS_AI_EP, "0");
DEF_ENV_PARAM(DEBUG_VITIS_AI_EP_DUMMY_RUNNER, "0");
DEF_ENV_PARAM(XLNX_ENABLE_DUMP, "0");
DEF_ENV_PARAM(NUM_OF_DPU_RUNNERS, "1");
DEF_ENV_PARAM(NUM_OF_PAD_THREADS, "1");
DEF_ENV_PARAM(DEBUG_USE_NEW_SCHEDULE, "1")
DEF_ENV_PARAM(USE_CPU_RUNNER, "0");
DEF_ENV_PARAM(XLINX_VART_DUMP_OUTPUT, "0");
DEF_ENV_PARAM(DEBUG_DPU_CUSTOM_OP, "0");
DEF_ENV_PARAM(XLNX_ENABLE_GRAPH_ENGINE_PAD, "1")

DEF_ENV_PARAM(XLNX_ENABLE_STAT_LOG, "0")
DEF_ENV_PARAM(XLNX_ENABLE_BATCH, "0")

DEF_ENV_PARAM(GET_WORKLOADONARCH_BY_EGOPS, "0");

#define MY_LOG(n) LOG_IF(INFO, ENV_PARAM(DEBUG_DPU_CUSTOM_OP) >= n)

namespace vaip_dpu_custom_op {

static void real_compute(const MyCustomOp* custom_op, const OrtApi* api,
                         OrtKernelContext* context, vart::RunnerExt* runner);
void fill_inputs(
    const MyCustomOp* custom_op, Ort::KernelContext& context,
    const std::vector<vart::TensorBuffer*>& vart_input_tensor_buffers);

void copy_outputs(
    const MyCustomOp* custom_op, Ort::KernelContext& context,
    const std::vector<vart::TensorBuffer*>& vart_output_tensor_buffers);

xir::Subgraph* get_dpu_subgraph_by_name() { return nullptr; }

static std::shared_ptr<GraphHolder>
create_graph_holder(std::shared_ptr<const PassContext> context,
                    const std::string& filename) {
  auto ret = std::shared_ptr<GraphHolder>();
  auto log_dir = context->get_log_dir();
  auto path = log_dir / filename;
  auto full_filename = path.u8string();

  std::string decryption_key = context->get_config_proto().encryption_key();
  ret = vitis::ai::WeakStore<std::string, GraphHolder>::create(
      full_filename, *context, filename, decryption_key);
  return ret;
}

static const xir::Subgraph*
find_dpu_subgraph(const std::shared_ptr<GraphHolder>& graph_holder,
                  const std::string& subgrah_name) {
  for (auto& subgraph : graph_holder->get_subgraphs()) {

    if (subgraph->has_attr("device") &&
        subgraph->get_attr<std::string>("device") == "DPU" &&
        subgraph->get_name() == subgrah_name) {
      return subgraph;
    }
  }
  return nullptr;
}
#ifdef ENABLE_XRT_SHARED_CONTEXT
// This function originates from GE, as referenced in the following link.
// Currently, it ignores the XLNX_ENABLE_DEBUG_MODE environment parameter,
// but it may need to consider it in the future.
// https://gitenterprise.xilinx.com/VitisAI/graph-engine/blob/dev/src/graph-engine/graph_runner.cpp#L117
static bool is_pdi_enabled(const xir::Subgraph* subgraph) {
  bool en_pdi = false;
  if (subgraph->has_attr("enable_pdi"))
    en_pdi = subgraph->get_attr<bool>("enable_pdi");

  if ((subgraph->has_attr("enable_fast_pm")) &&
      (subgraph->get_attr<bool>("enable_fast_pm"))) {
    en_pdi = false;
  }
  return en_pdi;
}
#endif

static std::unordered_map<std::string, std::uint64_t>
get_qos_from_subgraph(const xir::Subgraph* Subgraph) {
  std::unordered_map<std::string, std::uint64_t> qos_in_subg;
  std::uint64_t workload = 0;
  std::uint64_t workload_on_arch = 0;

  std::uint64_t xmodel_fingerprint = 0;
  if (Subgraph->has_attr("dpu_fingerprint")) {
    xmodel_fingerprint = Subgraph->get_attr<std::uint64_t>("dpu_fingerprint");
  }

  if (Subgraph->has_attr("workload_on_arch") &&
      (xmodel_fingerprint !=
       576460752305570371)) { // workload_on_arch is not valid for .122 xmodel
    workload_on_arch = Subgraph->get_attr<std::uint64_t>("workload_on_arch");
  }
  if (Subgraph->has_attr("workload")) {
    workload = Subgraph->get_attr<std::uint64_t>("workload");
  }
  if ((workload_on_arch & workload) &&
      (workload_on_arch < workload)) { // both exists but workload_on_arch is
                                       // smaller, then not valid
    workload_on_arch = 0;
  }
  qos_in_subg["workload"] = workload;
  qos_in_subg["workload_on_arch"] = workload_on_arch;

  if (Subgraph->has_attr("const_data_load_size")) {
    qos_in_subg["const_data_load_size"] =
        Subgraph->get_attr<std::uint64_t>("const_data_load_size");
  }
  if (Subgraph->has_attr("output_feature_save_size")) {
    qos_in_subg["output_feature_save_size"] =
        Subgraph->get_attr<std::uint64_t>("output_feature_save_size");
  }
  if (Subgraph->has_attr("input_feature_load_size")) {
    qos_in_subg["input_feature_load_size"] =
        Subgraph->get_attr<std::uint64_t>("input_feature_load_size");
  }

  return qos_in_subg;
}

MyCustomOp::MyCustomOp(std::shared_ptr<const PassContext> context,
                       const std::shared_ptr<MetaDefProto>& meta_def,
                       onnxruntime::Model* model)
    : CustomOpImp(context, meta_def, model), //
      graph_holder_{create_graph_holder(
          context, meta_def->dpu_param().compiled_xmodel())},
      subgraph_{find_dpu_subgraph(graph_holder_,
                                  meta_def->dpu_param().subgraph_name())},
      input_schedules_(meta_def->dpu_param().input_schedule()),
      output_schedules_(meta_def->dpu_param().output_schedule()) {
  // LOG(INFO) << " Vitis AI EP running " << meta_def->nodes_size() << " Nodes";

  CHECK(subgraph_ != nullptr);

#ifdef ENABLE_XRT_SHARED_CONTEXT
#else
  std::shared_ptr<xir::Attrs> shared_attrs = xir::Attrs::create();
#endif

  auto cfg_sess_opts = context->get_config_proto().provider_options();
  int num_of_runners = 1;
  auto runners_num = cfg_sess_opts.find("num_of_dpu_runners");
  if (runners_num != cfg_sess_opts.end()) {
    std::string num_string = runners_num->second;
    try {
      std::uint32_t num_int = std::atoi(num_string.c_str());
      if (num_int <= 0 || num_int > 8) {
        LOG(FATAL) << "num_of_dpu_runners should > 0 and <= 8, but now is: "
                   << num_int;
        return;
      }
      num_of_runners = num_int;
    } catch (std::exception& e) {
      LOG(WARNING) << "Failed to convert num_of_runners param"
                   << " from string to int : " << e.what();
    }
  }

  std::vector<std::unique_ptr<RunnerHolder>> runners;
  if (ENV_PARAM(DEBUG_VITIS_AI_EP_DUMMY_RUNNER)) {
    num_of_runners = 1;
  }
  if (ENV_PARAM(USE_CPU_RUNNER)) {
    num_of_runners = 1;
  }

  auto dl_analyzer_enabled = ENV_PARAM(XLNX_ONNX_EP_DL_ANALYZER_PROFILING) ||
                             ENV_PARAM(XLNX_ONNX_EP_DL_ANALYZER_VISUALIZATION);

  if (dl_analyzer_enabled) {
    try {
      auto _g = graph_holder_->get_graph();
      auto dpu_timestamp_info = gen_dpu_timestamp_info(_g);
      auto fused_viz = gen_fused_viz(_g);
      std::ofstream json_file;

      if (ENV_PARAM(XLNX_ONNX_EP_DL_ANALYZER_PROFILING)) {
        json_file.open("dpu_timestamp_info.json");
        if (json_file.is_open()) {
          json_file << dpu_timestamp_info;
          json_file.close();
        }
      }

      if (ENV_PARAM(XLNX_ONNX_EP_DL_ANALYZER_VISUALIZATION)) {
        json_file.open("fused_viz.json");
        if (json_file.is_open()) {
          json_file << fused_viz;
          json_file.close();
        }
      }

#ifdef _WIN32
      if (_putenv("Debug.ml_timeline=true") != 0)
        LOG(WARNING)
            << "failed to set environment variable to enable ml timeline";
#else
      char env_var[] = "Debug.ml_timeline=true";
      if (putenv(env_var) != 0)
        LOG(WARNING)
            << "failed to set environment variable to enable ml timeline";
#endif

      namespace fs = std::filesystem;
      fs::path report_path = context->get_log_dir() / "vitisai_ep_report.json";
      fs::path dest_path = fs::current_path() / "vitisai_ep_report.json";
      fs::copy_file(report_path, dest_path,
                    fs::copy_options::overwrite_existing);

      /*
       * The number of runners is set to 1,
       * because currently XRT only supports profiling one hw_context
       */
      num_of_runners = 1;
    } catch (std::exception& e) {
      LOG(WARNING) << "-- Error: Failed to generate files for dlanalyzer: "
                   << e.what();
    }
  }

  // Backward compatibility.
  // here we can get xclbin from MetaDef's DpuParam, during the compilation
  // phase ,will setting xclbin into MetaDef
  // here xclbin is a fullpath
  auto xclbin_file = context
                         ->xclbin_path_to_cache_files(std::filesystem::path(
                             meta_def->dpu_param().xclbin()))
                         .string();
  CHECK(!xclbin_file.empty()) << "no setting xclbin";

  share_context_ = false;
#ifdef ENABLE_XRT_SHARED_CONTEXT
  if (cfg_sess_opts.contains(vaip::Context::CTX_SHARE_OPTION_KEY)) {
    try {
      share_context_ =
          std::stoi(cfg_sess_opts.at(vaip::Context::CTX_SHARE_OPTION_KEY));
    } catch (...) {
      MY_LOG(1) << "failed to convert provider option \""
                << vaip::Context::CTX_SHARE_OPTION_KEY << "\" value \""
                << cfg_sess_opts.at(vaip::Context::CTX_SHARE_OPTION_KEY)
                << "\" to int, disable context sharing.";
    }
  }
#endif
  for (auto i = 0; i < num_of_runners; ++i) {
    auto runner_holder = std::make_unique<RunnerHolder>();

    std::map<std::string, std::uint32_t> qos_map;
    // https://github.com/Xilinx/XRT/blob/72f7994e24e21e3f16dd0db23c93c19929f44f72/src/runtime_src/core/include/xrt/xrt_hw_context.h#L36-L52
    // FIXME: "tops" on users' side? "gops" handled by GE opaquely?
    for (auto param : {"tops", "fps", "dma_bandwidth", "latency",
                       "latency_in_us", "frame_execution_time", "priority"}) {
      auto it = cfg_sess_opts.find(param);
      if (it != cfg_sess_opts.end()) {
        std::string qos_string = it->second;
        try {
          std::uint32_t qos_int = std::atoi(qos_string.c_str());
          // When priority is set, set perf_pref to 1 to enable efficient mode.
          // qos_map will be used either by share_context or GE.
          if (param == "priority") {
            qos_map["perf_pref"] = 1;
          } else {
            qos_map[param] = qos_int;
          }
        } catch (std::exception& e) {
          LOG(WARNING) << "Failed to convert qos param :" << param
                       << " from string to int : " << e.what();
        }
      }
    }

#ifdef ENABLE_XRT_SHARED_CONTEXT
    if (share_context_) {
      // get QoS info from subgraph
      // This function logic basically originates from GE, as referenced in the
      // following link.
      // https://gitenterprise.xilinx.com/VitisAI/graph-engine/blob/c4e1132b0c9d05a47dab3175d9dc9d6ed878522b/src/graph-engine/graph_runner.cpp#L615-L630
      std::map<std::string, std::uint32_t> qos_map_share_ctx = qos_map;
      if (!ENV_PARAM(GET_WORKLOADONARCH_BY_EGOPS)) {
        if (subgraph_->has_attr("workload")) { // change to workload_on_arch
                                               // when subgraph has this attr
          auto workload = subgraph_->get_attr<std::uint64_t>("workload");
          qos_map_share_ctx["gops"] =
              static_cast<uint32_t>(std::ceil(workload / 1000000000.0));
        }
      } else {
        std::unordered_map<std::string, std::uint64_t> all_qos =
            get_qos_from_subgraph(subgraph_);
        qos_map_share_ctx["gops"] = static_cast<uint32_t>(
            std::ceil(all_qos["workload"] / 1000000000.0));
        qos_map_share_ctx["egops"] = static_cast<uint32_t>(
            std::ceil(all_qos["workload_on_arch"] / 1000000000.0));
      }

      MY_LOG(1) << "DPU op using shared context";
      auto device_id = 0;
      auto context_id = i % vaip::Context::MAX_NUM_OF_CONTEXTS;
      runner_holder->context_ = vaip::Context::create_shared_context(
          *context, device_id, context_id, xclbin_file, qos_map_share_ctx);
      runner_holder->context_->update_qos(qos_map_share_ctx);
      // use XRTUpdateQosImpl object to update efficient mode directly through
      // xrt::hw_context
      auto qos_updater = std::make_shared<vaip_core::XRTUpdateQosImpl>(
          &(runner_holder->context_->xrt_hw_context()));
      context->add_QosUpdater(qos_updater);
    } else {
      // no share, only the attr is used. For backward compatibility,
      // DO share the attr between multiple dpu op instances
      MY_LOG(1) << "DPU op NOT using shared context";
      auto context_id = i % vaip::Context::MAX_NUM_OF_CONTEXTS;
      auto key = std::to_string(context_id) + xclbin_file;
      runner_holder->context_ =
          vitis::ai::WeakStore<std::string, vaip::Context>::create(key);
    }

    // Attributes
    auto attrs = runner_holder->context_->get_attrs();
    // Let GE know the location of xclbin when
    // the env var XLNX_VART_FIRMWARE is not used.
    if (context->cache_in_mem()) {
      auto read_xclbin = context->read_xclbin(xclbin_file);
      if (read_xclbin.has_value()) {
        MY_LOG(1) << "passing xclbin data to GE";
        auto xclbin_data = std::vector<char>(read_xclbin.value().begin(),
                                             read_xclbin.value().end());
        attrs->set_attr("xclbin_raw_data", xclbin_data);
      }
    } else {
      MY_LOG(1) << "passing xclbin path to GE";
      attrs->set_attr("xclbin_file", xclbin_file);
    }

    if (ENV_PARAM(USE_CPU_RUNNER)) {
      // do nothing
    } else if (!share_context_) {
      // not use xrt_shared_context
      MY_LOG(1) << "setting default_initialize to true";
      attrs->set_attr<bool>("default_initialize", true);
    } else {
      MY_LOG(1) << "Using XRT device from vaip";
      bool en_pdi = is_pdi_enabled(subgraph_);
      if (en_pdi) {
        // Create PDI kernels
        auto pdi_subgraphs = subgraph_->children_topological_sort();
        std::for_each(
            pdi_subgraphs.cbegin(), pdi_subgraphs.cend(),
            [&](const xir::Subgraph* pdi_subgraph) {
              if ((pdi_subgraph->get_attr<std::string>("type") == "PDI")) {
                auto ker_name = pdi_subgraph->get_attr<std::string>("name");
                auto attr_name = pdi_subgraph->get_name() + '_' + ker_name;
                runner_holder->context_->create_kernel(attr_name, ker_name);
              }
            });
      } else {
        // Create kernel
        auto kernel_name = runner_holder->context_->get_xclbin_kernelName();
        runner_holder->context_->create_kernel(std::string("xrt_kernel"),
                                               kernel_name);
      }
      // Set default init to false
      attrs->set_attr<bool>("default_initialize", false);
      vitis::ai::trace::add_info(
          "dpu-controller", "cu_device_id", 0, "cu_core_id", 0, "cu_batch", 1,
          TRACE_VAR("DPU"), TRACE_VAR("IPU_DPU"), "cu_fingerprint", 1);
    }
#else
    runner_holder->attrs_ = shared_attrs;
    auto attrs = shared_attrs.get();
    // Let GE know the location of xclbin when
    // the env var XLNX_VART_FIRMWARE is not used.
    attrs->set_attr("xclbin_file", xclbin_file);
#endif

    if (ENV_PARAM(USE_GRAPH_ENGINE)) {
      // XLNX_ENABLE_GRAPH_ENGINE_PAD == 0 means data has been paded
      attrs->set_attr("bypass_pad",
                      ENV_PARAM(XLNX_ENABLE_GRAPH_ENGINE_PAD) == 0);
      attrs->set_attr("lib", std::map<std::string, std::string>{
                                 {"DPU", "libgraph-engine.so"}});
    }

    if (ENV_PARAM(DEBUG_VITIS_AI_EP_DUMMY_RUNNER)) {
      attrs->set_attr("lib", std::map<std::string, std::string>{
                                 {"DPU", "libvart-dummy-runner.so"}});
    } else if (ENV_PARAM(USE_CPU_RUNNER)) {
      std::string mode = "ref";
      attrs->set_attr("mode", mode);
      attrs->set_attr("lib", std::map<std::string, std::string>{
                                 {"DPU", "libvart-cpu-runner.so"}});
      if (ENV_PARAM(XLINX_VART_DUMP_OUTPUT)) {
        attrs->set_attr("dump_op_output", true);
      }
    }

    if (!qos_map.empty()) {
      attrs->set_attr<std::map<std::string, std::uint32_t>>("qos_params",
                                                            qos_map);
    }
    std::string ge_ctx_id = cfg_sess_opts["ctx_idx"];
    if (!ge_ctx_id.empty()) {
      attrs->set_attr<int>("ctx_idx", std::atoi(ge_ctx_id.c_str()));
    }

    try {
      auto vart_runner = vart::RunnerExt::create_runner(subgraph_, attrs);
      if (!share_context_) {
        auto ge_update_qos_impl =
            std::make_shared<vaip_core::GEUpdateQosImpl>(vart_runner.get());
        context->add_QosUpdater(ge_update_qos_impl);
      }

      runner_holder->runner_ = std::move(vart_runner);
    } catch (std::exception& e) {
      LOG(FATAL) << "-- Error: Failed to create GE handle: " << e.what();
    }
    runners.push_back(std::move(runner_holder));
  }
  runnerRequestsQueue_ =
      std::unique_ptr<RunnerRequestsQueue>(new RunnerRequestsQueue(runners));
}

MyCustomOp::~MyCustomOp() {}

void MyCustomOp::Compute(const OrtApi* api, OrtKernelContext* context) const {
  // there are two version of the global variable, be careful, because
  // ORT is remove all public/external variables for linking.

  if (Ort::Global<void>::api_ == nullptr) {
    Ort::Global<void>::api_ = api;
  }
  __TIC__(GET_RUNNER);
  auto runner_request = runnerRequestsQueue_->getIdleRequest();
  __TOC__(GET_RUNNER);
  auto runner = runner_request->runner_.get();

  __TIC__(COMPUTE);
  MY_LOG(1) << "dpu kernel " << subgraph_->get_name() << "\n";
  auto __customop_start_time = std::chrono::steady_clock::now();
  real_compute(this, api, context, runner);
  auto __customop_end_time = std::chrono::steady_clock::now();
  auto dpu_time = std::chrono::duration_cast<std::chrono::microseconds>(
                      __customop_end_time - __customop_start_time)
                      .count();
  if (ENV_PARAM(XLNX_ENABLE_STAT_LOG)) {
    std::cout << "[Vitis AI EP] IPU subgraph : " << subgraph_->get_name()
              << " \t No. of Ops " << meta_def_->nodes_size() << " \tLatency "
              << dpu_time << " us" << std::endl;
  }
  __TOC__(COMPUTE);

  __TIC__(PUT_RUNNER);
  runnerRequestsQueue_->putIdleRequest(runner_request);
  __TOC__(PUT_RUNNER);
}

static void real_compute(const MyCustomOp* custom_op, const OrtApi* api,
                         OrtKernelContext* context, vart::RunnerExt* runner) {

  __TIC__(COPY_INPUT_PREPARE);
  Ort::KernelContext ctx(context);
  auto num_inputs = ctx.GetInputCount();
  auto num_outputs = ctx.GetOutputCount();
  auto vart_input_tensor_buffers = runner->get_inputs();
  auto vart_output_tensor_buffers = runner->get_outputs();
  MY_LOG(1) << "num_inputs " << num_inputs << " "                        //
            << "num_outputs " << num_outputs << " "                      //
            << "\tnum_vart_inputs: " << vart_input_tensor_buffers.size() //
            << "\tnum_vart_outputs: " << vart_output_tensor_buffers.size();

  __TOC__(COPY_INPUT_PREPARE);

  // layout transform and fill dpu input from onnx OrtValue
  __TIC__(COPY_INPUT);
  fill_inputs(custom_op, ctx, vart_input_tensor_buffers);
  __TOC__(COPY_INPUT);

  // sync input tensor buffers
  __TIC__(SYNC_INPUT);
  for (auto& input : vart_input_tensor_buffers) {
    auto batch = input->get_tensor()->get_shape()[0];
    if (!ENV_PARAM(XLNX_ENABLE_BATCH)) {
      // ignore HW batch， the first dim is not batch
      batch = 1;
    }
    input->sync_for_write(0, input->get_tensor()->get_data_size() / batch);
  }
  __TOC__(SYNC_INPUT);

  // run dpu runner
  __TIC__(RUN);
  vitis::ai::trace::add_trace("user-task", vitis::ai::trace::func_start,
                              "graph_engine::dpu_kernel_run", "");
  auto v = runner->execute_async(vart_input_tensor_buffers,
                                 vart_output_tensor_buffers);
  auto status = runner->wait((int)v.first, -1);
  CHECK_EQ(status, 0) << "failed to run the graph";
  vitis::ai::trace::add_trace("user-task", vitis::ai::trace::func_end,
                              "graph_engine::dpu_kernel_run", "");
  __TOC__(RUN);

  // sync output tensor buffers
  __TIC__(SYNC_OUTPUT);
  for (auto output : vart_output_tensor_buffers) {
    auto batch = output->get_tensor()->get_shape()[0];
    if (!ENV_PARAM(XLNX_ENABLE_BATCH)) {
      // ignore HW batch， the first dim is not batch
      batch = 1;
    }
    output->sync_for_read(0, output->get_tensor()->get_data_size() / batch);
  }
  __TOC__(SYNC_OUTPUT);
  // layout tranform and copy dpu output to onnx OrtValue
  __TIC__(COPY_OUTPUT);
  copy_outputs(custom_op, ctx, vart_output_tensor_buffers);
  __TOC__(COPY_OUTPUT);
}
static int64_t get_onnx_batch(Ort::KernelContext& ctx) {
  auto onnx_tensor = ctx.GetInput(0);
  auto tensor_info = onnx_tensor.GetTensorTypeAndShapeInfo();
  auto tensor_shape = tensor_info.GetShape();
  auto batch = tensor_shape[0];
  if (!ENV_PARAM(XLNX_ENABLE_BATCH)) {
    // ignore onnx batch , the first dim is not batch
    batch = 1;
  }
  return batch;
}
std::shared_ptr<vart::TensorBuffer> create_tensor_buffer(
    Ort::KernelContext& context,
    const std::vector<vart::TensorBuffer*>& vart_input_tensor_buffers,
    const TensorBufferParam& param) {
  auto tb_type = param.tb_type();
  if (tb_type == vaip_core::ONNX_INPUT) {
    return create_onnx_input_tensor_buffer(param.tensor_name(), context,
                                           (int)param.onnx_index());
  } else if (tb_type == vaip_core::ONNX_OUTPUT) {
    auto onnx_shape = std::vector<int64_t>();
    onnx_shape.reserve(param.onnx_shape_size());
    for (auto i = 0; i < param.onnx_shape_size(); ++i) {
      onnx_shape.push_back((int64_t)param.onnx_shape(i));
    }
    if (ENV_PARAM(XLNX_ENABLE_BATCH)) {
      int64_t onnx_batch = get_onnx_batch(context);
      onnx_shape[0] = onnx_batch;
    } else {
      if (onnx_shape[0] == -1) {
        onnx_shape[0] = 1;
      }
    }
    return create_onnx_output_tensor_buffer(
        param.tensor_name(), context, (int)param.onnx_index(), onnx_shape);
  } else if (tb_type == vaip_core::XIR_INPUT) {
    return create_xir_tensor_buffer(param.tensor_name(),
                                    vart_input_tensor_buffers);
  } else if (tb_type == vaip_core::XIR_OUTPUT) {
    return create_xir_tensor_buffer(param.tensor_name(),
                                    vart_input_tensor_buffers);
  } else {
    LOG(FATAL) << "not support tensor buffer type " << param.tb_type();
  }
  return nullptr;
}

void trans_data(
    Ort::KernelContext& context,
    const std::vector<vart::TensorBuffer*>& vart_input_tensor_buffers,
    const MetaSchedule& schedule) {

  auto from_tensor_buffer = create_tensor_buffer(
      context, vart_input_tensor_buffers, schedule.from_tb_param());
  auto to_tensor_buffer = create_tensor_buffer(
      context, vart_input_tensor_buffers, schedule.to_tb_param());
  trans_data(from_tensor_buffer, to_tensor_buffer, schedule.op(),
             get_onnx_batch(context));
}

void fill_inputs(
    const MyCustomOp* custom_op, Ort::KernelContext& context,
    const std::vector<vart::TensorBuffer*>& vart_input_tensor_buffers) {
  auto& input_schedules = custom_op->get_input_schdules();
  for (auto& schedule : input_schedules) {
    trans_data(context, vart_input_tensor_buffers, schedule);
  }
}

void copy_outputs(
    const MyCustomOp* custom_op, Ort::KernelContext& context,
    const std::vector<vart::TensorBuffer*>& vart_output_tensor_buffers) {
  auto& output_schedules = custom_op->get_output_schdules();
  for (auto& schedule : output_schedules) {
    trans_data(context, vart_output_tensor_buffers, schedule);
  }
}

} // namespace vaip_dpu_custom_op
