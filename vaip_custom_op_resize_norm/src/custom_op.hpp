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
#pragma once

#include "vaip/vaip.hpp"
#include "vart/runner_ext.hpp"

#include <algorithm>
#include <future>
#include <mutex>
class ResizeDown;
class Normalize;
// XRT includes
#include "../../xrt_shared_context/xrt_shared_context.hpp"
// FD Post kernel

namespace vaip_resize_norm_custom_op {
using namespace vaip_core;
class MyCustomOp : public CustomOpImp {
public:
  MyCustomOp(std::shared_ptr<const PassContext> context,
             const std::shared_ptr<MetaDefProto>& meta_def,
             onnxruntime::Model* model);

  virtual ~MyCustomOp();

private:
  std::shared_ptr<vaip::Context> context_;
  std::unique_ptr<ResizeDown> kernel_resize_;
  std::unique_ptr<Normalize> kernel_norm_;
  // std::uint32_t instr_buffer_norm[INSTR_BUFFER_LENGTH_MAX];
  xrt::bo instr_bo_resize_;
  xrt::bo instr_bo_norm_;
  std::string kernel_name_resize_;
  std::string kernel_name_norm_;
  std::vector<int64_t> input_shape_;
  std::vector<int64_t> output_shape_;
  std::vector<int64_t> target_shape_;
  std::vector<int> fl_bits_;
  std::vector<float> mean_;
  std::vector<float> stddev_;
  bool en_transpose_{false};

  virtual void Compute(const OrtApi* api,
                       OrtKernelContext* context) const override final;
};

} // namespace vaip_resize_norm_custom_op