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

#include "./custom_op.hpp"
#include "fstream"
#include "iostream"
#include "onnxruntime_api.hpp"
#include <cstdint>
#include <glog/logging.h>
#include <sstream>
#include <vector>

namespace vaip_dqcastgather_custom_op {

MyCustomOp::MyCustomOp(std::shared_ptr<const PassContext> context,
                       const std::shared_ptr<MetaDefProto>& meta_def,
                       onnxruntime::Model* model)
    : CustomOpImp(context, meta_def, model) {
  scale = stof(meta_def->generic_param().at("in_scale"));
  ifm_dim_0 = stoi(meta_def->generic_param().at("ifm_dim_0"));
  ifm_dim_1 = stoi(meta_def->generic_param().at("ifm_dim_1"));
  indeces_shape = stoi(meta_def->generic_param().at("indeces_shape"));
  std::string zp_file = meta_def->generic_param().at("zp_file");
  std::string data_file = meta_def->generic_param().at("data_file");
  auto data_file_opt = context->read_file_c8(
      std::filesystem::path(data_file).filename().string());
  auto zp_file_opt =
      context->read_file_c8(std::filesystem::path(zp_file).filename().string());
  if (!data_file_opt.has_value()) {
    std::cerr << "Error reading file: " << data_file << std::endl;
  }
  if (!zp_file_opt.has_value()) {
    std::cerr << "Error reading file: " << zp_file << std::endl;
  }
  auto file = data_file_opt.value();
  in_data.resize(file.size() / sizeof(int8_t));
  memcpy(in_data.data(), file.data(), file.size());

  auto file_zp = zp_file_opt.value();
  zp = *reinterpret_cast<const int8_t*>(file_zp.data());

  size_t input_size = ifm_dim_0 * ifm_dim_1;
  in_dq.resize(input_size);
  for (size_t i = 0; i < input_size; ++i) {
    in_dq[i] = (float)((float)in_data[i] - (float)zp) * scale;
  }
}

MyCustomOp::~MyCustomOp() {}

void MyCustomOp::Compute(const OrtApi* api, OrtKernelContext* context) const {
  if (Ort::Global<void>::api_ == nullptr) {
    Ort::Global<void>::api_ = api;
  }

  Ort::KernelContext ctx(context);
  auto num_inputs = ctx.GetInputCount();
  auto num_outputs = ctx.GetOutputCount();
  auto input_tensor = ctx.GetInput(0);
  auto tensor_info = input_tensor.GetTensorTypeAndShapeInfo();
  auto indeces = input_tensor.GetTensorData<float>();
  auto output_tensor = ctx.GetOutput(0, {1, indeces_shape, ifm_dim_1});
  auto out_data = output_tensor.GetTensorMutableData<int8_t>();
  size_t out_size = indeces_shape * ifm_dim_1;
  // std::vector<float> dq_out(out_size);

  float maxval = 127.0f;
  float minval = -128.0f;

  for (size_t i = 0; i < indeces_shape; i++) {
    int idx_v = (int)indeces[i];
    for (int j = 0; j < ifm_dim_1; j++) {
      float temp =
          in_dq[idx_v * ifm_dim_1 + j] / scale + static_cast<float>(zp);
      temp = std::min(maxval, std::max(minval, temp));
      out_data[i * ifm_dim_1 + j] = static_cast<int8_t>(temp);
    }
  }
}
} // namespace vaip_dqcastgather_custom_op
