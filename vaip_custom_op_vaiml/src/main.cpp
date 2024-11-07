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

#include <glog/logging.h>

#include "GT/GT_1_2/custom_op_gt_1_2.hpp"
#include "GT/GT_1_3/custom_op_gt_1_3.hpp"
#include "HT_1_2/custom_op_ht_1_2.hpp"
#include "vaip/vaip.hpp"

static vaip_core::ExecutionProvider* create_execution_provider_imp(
    std::shared_ptr<const vaip_core::PassContext> context,
    const vaip_core::MetaDefProto& meta_def) {

  auto& session_option = context->get_config_proto().provider_options();
  auto model_version_ = session_option.at("model_name");
  if (model_version_ == "GT_v1.2") {
    return new vaip_core::ExecutionProviderImp<
        vaip_vaiml_custom_op::MyCustomOpGT1_2>(context, meta_def);
  } else if (model_version_ == "HT_v1.2") {
    return new vaip_core::ExecutionProviderImp<
        vaip_vaiml_custom_op::MyCustomOpHT1_2>(context, meta_def);
  } else if (model_version_ == "GT_v1.3") {
    return new vaip_core::ExecutionProviderImp<
        vaip_vaiml_custom_op::MyCustomOpGT1_3>(context, meta_def);
  } else {
    throw std::invalid_argument("Unknown model");
  }
}

#if VAIP_CUSTOM_OP_VAIML_USE_DLL == 0
#  include <vitis/ai/plugin.hpp>
namespace {
static vitis::ai::StaticPluginRegister
    __register("vaip_custom_op_VAIML", "create_execution_provider",
               (void*)&create_execution_provider_imp);
} // namespace
extern "C" {
void* vaip_custom_op_vaiml__hook = &__register;
}
#else

extern "C" VAIP_PASS_ENTRY vaip_core::ExecutionProvider*
create_execution_provider(std::shared_ptr<const vaip_core::PassContext> context,
                          const onnxruntime::Graph* graph,
                          const vaip_core::MetaDefProto& meta_def) {
  return create_execution_provider_imp(context, graph, meta_def);
}
extern "C" {
void* vaip_custom_op_vaiml__hook = nullptr;
}
#endif
