/*
 *  Copyright (C) 2022 Xilinx, Inc. All rights reserved.
 *  Copyright (C) 2023 – 2024 Advanced Micro Devices, Inc. All rights reserved.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 **/
// #include "symbols.hpp"

// typedef void* void_ptr_t;
// #define DECLARE_SYMBOL(sym) extern "C" void_ptr_t sym;
// SYMBOLS(DECLARE_SYMBOL)
// #if defined(_WIN32)
// SYMBOLS_WIN(DECLARE_SYMBOL)
// #endif

// #define DEFINE_SYMBOL(sym) sym,

// static void_ptr_t reserved_symbols[] = {SYMBOLS(DEFINE_SYMBOL)};
#include "onnxruntime_vitisai_ep/onnxruntime_vitisai_ep.hpp"
#include "vaip/config_reader.hpp"
#include "vaip/op_def.hpp"
#include "vaip/vaip.hpp"
#include "vaip/vaip_plugin.hpp"
#include <fstream>
#include <glog/logging.h>

// for fix graph_engine not define hook when use BUILD_SHARED_LIBS
#if GRAPH_ENGINE_USE_DLL == 1
extern "C" {
void* graph_engine__hook = nullptr;
}
#endif
#ifdef FOUND_GRAPH_ENGINE
extern "C" void* graph_engine__hook;
void* graph_engine_reserved_symbols[] = {graph_engine__hook};
#endif
#ifdef FOUND_CPU_RUNNER
extern void* vart_cpu_runner_reg_hooks[];
void** vart_cpu_runner_reg_hooks_ptr = vart_cpu_runner_reg_hooks;
extern "C" void* vart_cpu_runner_hook;
void* vart_cpu_runner_reserved_symbols[] = {vart_cpu_runner_hook};
#endif
#ifdef FOUND_XCOMPILER
extern void* xcompiler_hooks[];
void** xcompiler_hooks_ptr = xcompiler_hooks;
#endif

extern "C" {
ONNXRUNTIME_VITISAI_EP_DLL_SPEC
uint32_t vaip_get_version_c() { return 0; }
// The interface exported below is used by onnxruntime_providers_vitisai.so
ONNXRUNTIME_VITISAI_EP_DLL_SPEC
void initialize_onnxruntime_vitisai_ep_c(
    vaip_core::OrtApiForVaip* api,
    std::vector<OrtCustomOpDomain*>& ret_domain) {
  vaip_core::initialize_onnxruntime_vitisai_ep(api, ret_domain);
  static std::vector<OrtCustomOpDomain*> contrib_domains;
  std::vector<std::string> op_defs{
#include "op_def.cpp.inc"
  };
  for (auto& op_def : op_defs) {
    auto plugin_holder = vaip_core::Plugin::get(op_def);
    auto op_def_info =
        plugin_holder->invoke<vaip_core::OpDefInfo*>("vaip_op_def_info");
    std::vector<Ort::CustomOpDomain> domains;
    op_def_info->get_domains(domains);
    for (auto& domain : domains) {
      // Memory leak, passing data across dlls
      contrib_domains.push_back(domain.release());
      ret_domain.push_back(*contrib_domains.rbegin());
      CHECK_LE(ret_domain.size(), 100)
          << "ret_domain applied for 100 in onnxruntime";
    }
  }
}

ONNXRUNTIME_VITISAI_EP_DLL_SPEC
std::vector<std::unique_ptr<vaip_core::ExecutionProvider>>*
compile_onnx_model_with_options_c(const std::string& model_path,
                                  const onnxruntime::Graph& graph,
                                  const onnxruntime::ProviderOptions& options) {
  auto json_config = vaip_core::get_config_json_str(options);
  return new std::vector<std::unique_ptr<vaip_core::ExecutionProvider>>(
      vaip_core::compile_onnx_model_3(model_path, graph, json_config.c_str()));
}
}