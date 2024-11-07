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
#include "custom_op_gqo.hpp"

#if defined(_WIN32)
#  include <intrin.h>
#else
#  include "ryzenai/dynamic_dispatch/utils/instruction_cache.hpp"
#  include "ryzenai/dynamic_dispatch/utils/instruction_registry.hpp"
#  include <ryzenai/dynamic_dispatch/xrt_context/xrt_context.hpp>
#  include <x86intrin.h>
#endif
#include <immintrin.h>
#include <mmintrin.h>
#include <xmmintrin.h>

#include <cmath>
#include <filesystem>
#include <thread>
#include <utility>

#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

#include <glog/logging.h>

#include "matmulnbits_util.hpp"
#include "reporter.hpp"
#include "vitis/ai/profiling.hpp"

namespace fs = std::filesystem;
std::vector<int> grp_sizes_;
std::vector<std::vector<int>> n_sizes_;

DEF_ENV_PARAM(DEBUG_MHA_CUSTOM_OP, "0")
DEF_ENV_PARAM(USE_AIE_GQO, "1")
DEF_ENV_PARAM(USE_AIE_RoPE, "1")
DEF_ENV_PARAM(USE_AIE_TOKEN, "0")
DEF_ENV_PARAM(MHA_PARALLEL_BATCH, "1")
DEF_ENV_PARAM_2(MLADF_VERSION, "v1", std::string)
#define MY_LOG(n) LOG_IF(INFO, ENV_PARAM(DEBUG_MHA_CUSTOM_OP) >= n)
DEF_ENV_PARAM(DRY_RUN, "0")

namespace ort_gqo_custom_op {
// Custom Op Domain
bool MyCustomOpKernel::is_const_cache = true;
std::shared_ptr<void> MyCustomOpKernel::gemm__ = nullptr;
int MyCustomOpKernel::instances__ = 0;
int MyCustomOpKernel::compute_instances__ = 0;
std::shared_ptr<uint16_t> MyCustomOpKernel::cos_sin_cache__ = nullptr;
std::once_flag MyCustomOpKernel::initFlag;

uint16_t* AttenMaskProvider::get_atten_mask(int32_t S) {
  // if S is in aie support list, try to get LUT first.
  auto is_supported = aie_kernel_info_->is_seq_aie_supported(S);
  if (is_supported && attn_mask_lut_->hasLut(S)) {
    assert(attn_mask_lut_ != nullptr);
    MY_LOG(2) << "Use atten_mask LUT for current S: " << S;
    return attn_mask_lut_->getLut(S);
  } else {
    MY_LOG(2) << "Construct atten_mask on the fly for current S: " << S;
    // otherwise construct the LUT on the fly.
    // Todo(ltp): consider case when b != 1;
    auto size = 1 * 1 * S * S; // B * 1 * S * S
    auto bf16_attention_mask =
        (uint16_t*)allocator_.Alloc(size * sizeof(uint16_t));
    fill_attn_mask_impl(bf16_attention_mask, S);
    free_list_.push_back(bf16_attention_mask);
    return bf16_attention_mask;
  }
}
void* GQO_Allocator::get_buffer(size_t sz, BufferInfo& buffer) {
  if (buffer.first && buffer.second >= sz) {
    // size already satisfied
    return buffer.first;
  } else if (!buffer.first && buffer.second == 0) {
    // first alloc
    MY_LOG(2) << "mha initial memory allocatation, size " << sz;
    buffer.first = (void*)allocator_.Alloc(sz);
    buffer.second = sz;
  } else {
    // reallocation
    size_t new_sz = sz * growth_factor_;
    if (buffer.first) {
      // allocator_.Free(buffer.first);
      free_list_.push_back(buffer);
    }
    MY_LOG(2) << "mha reallocating memory, new size " << new_sz
              << " original size " << buffer.second;
    buffer.first = (void*)allocator_.Alloc(new_sz);
    buffer.second = new_sz;
  }
  return buffer.first;
}

void MyCustomOpKernel::get_rope_cache(Ort::ConstValue& cos_tensor,
                                      Ort::ConstValue& sin_tensor) {
  const float* cos_data = cos_tensor.GetTensorData<float>();
  auto cos_shape = cos_tensor.GetTensorTypeAndShapeInfo().GetShape();
  int shape_cs_0 = cos_shape[0];
  int shape_cs_1 = cos_shape[1];

  const float* sin_data = sin_tensor.GetTensorData<float>(); // 4096x64

  std::vector<float> cos_embed(shape_cs_0 * shape_cs_1 * 2);
  std::vector<float> sin_embed(shape_cs_0 * shape_cs_1 * 2);
  int newK = 2 * shape_cs_1;
  // duplicate second dimention for cos and sin cache
  for (int i = 0; i < shape_cs_0; ++i) {
    for (int j = 0; j < shape_cs_1; ++j) {

      float cos_val = cos_data[i * shape_cs_1 + j];
      float sin_val = sin_data[i * shape_cs_1 + j];

      // Duplicate the values for second half
      cos_embed[i * newK + j] = cos_val;
      cos_embed[i * newK + j + shape_cs_1] = cos_val;

      sin_embed[i * newK + j] = sin_val;
      sin_embed[i * newK + j + shape_cs_1] = sin_val;
    }
  }
  max_seq_length = shape_cs_0;
  cs_1 = shape_cs_1;
#ifdef _WIN32
  cos_sin_cache__ = std::shared_ptr<uint16_t>(
      (uint16_t*)_aligned_malloc(
          2 * max_seq_length * (2 * shape_cs_1) * sizeof(uint16_t), 64),
      [](void* ptr) { _aligned_free(ptr); });
#else
  cos_sin_cache__ = std::shared_ptr<uint16_t>(
      (uint16_t*)aligned_alloc(64, 2 * max_seq_length * (2 * shape_cs_1) *
                                       sizeof(uint16_t)),
      [](void* ptr) { free(ptr); });
#endif
  size_t offset = 0;
  auto cos_element_num = max_seq_length * shape_cs_1;

  float_to_bfloat16_avx512_unrolled(cos_embed.data() + offset,
                                    cos_sin_cache__.get(), cos_element_num * 2);
  float_to_bfloat16_avx512_unrolled(sin_embed.data() + offset,
                                    cos_sin_cache__.get() + 2 * cos_element_num,
                                    cos_element_num * 2);
}

// transpose [0, 1, 2, 3] to [0, 2, 1, 3]
void MyCustomOpKernel::transpose0213(uint16_t* output_data,
                                     uint16_t* input_data, int D0, int D1,
                                     int D2, int D3,
                                     OrtKernelContext* context) {
  int tensor_size = D0 * D1 * D2 * D3;
  int64_t input_shape[4] = {D0, D1, D2, D3};
  int64_t output_shape[4] = {D0, D2, D1, D3};
  Ort::MemoryInfo info =
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  auto bf16_enum =
      ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_BFLOAT16;
  OrtValue* input = nullptr;
  api_->CreateTensorWithDataAsOrtValue(info, input_data,
                                       tensor_size * sizeof(uint16_t),
                                       input_shape, 4, bf16_enum, &input);
  OrtValue* output = nullptr;
  api_->CreateTensorWithDataAsOrtValue(info, output_data,
                                       tensor_size * sizeof(uint16_t),
                                       output_shape, 4, bf16_enum, &output);
  const OrtValue* inputs[1] = {input};
  OrtValue* outputs[1] = {output};

  transpose0213_built_in.Invoke(context, inputs, 1, outputs, 1);
}

inline bool check_prefill(int seq_len) { return (seq_len != 1); }

// Try pad qkv from [B, N, S, H] to [B, N, S_pad, H] with 0
void try_pad_qkv(uint16_t* dst, uint16_t* src, int64_t N, int64_t S,
                 int64_t S_pad, int64_t H) {
  if (S != S_pad) {
    std::memset(dst, 0, N * S_pad * H * sizeof(uint16_t));
    for (int64_t n = 0; n < N; n++) {
      std::memcpy(dst + n * S_pad * H, src + n * S * H,
                  S * H * sizeof(uint16_t));
    }
  } else {
    std::memcpy(dst, src, N * S * H * sizeof(uint16_t));
  }
}

// Try pad mask from [S, S] to [S_pad, S_pad] with neg_inf_ui16
void try_pad_mask(uint16_t* dst, uint16_t* src, int64_t S, int64_t S_pad) {
  if (S != S_pad) {
    const uint16_t neg_inf_ui16 = float_to_bfloat16(-3.389e38f);
    for (int64_t s = 0; s < S; s++) {
      std::memcpy(dst + s * S_pad, src + s * S, S * sizeof(uint16_t));
      for (int64_t s1 = S; s1 < S_pad; s1++) {
        dst[s * S_pad + s1] = neg_inf_ui16;
      }
    }
    for (int64_t s = S; s < S_pad; s++) {
      for (int64_t s1 = 0; s1 < S_pad; s1++) {
        dst[s * S_pad + s1] = neg_inf_ui16;
      }
    }
  } else {
    std::memcpy(dst, src, S * S * sizeof(uint16_t));
  }
}

/// @brief built in rope
/// @param output_data
/// @param input_data
/// @param B
/// @param N
/// @param S
/// @param H
/// @param context
void MyCustomOpKernel::RoPE(float* output_data, float* input_data,
                            int64_t* pos_ids_data, const OrtValue* cos_cache,
                            const OrtValue* sin_cache, int B, int N, int S,
                            int H, OrtKernelContext* context) {
  int tensor_size = B * N * S * H;
  std::vector<int64_t> tensor_shape{B, N, S, H};
  std::vector<int64_t> pos_ids_shape{B};
  Ort::MemoryInfo info =
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      info, input_data, tensor_size, tensor_shape.data(), tensor_shape.size());
  Ort::Value pos_ids = Ort::Value::CreateTensor<int64_t>(
      info, pos_ids_data, B, pos_ids_shape.data(), pos_ids_shape.size());
  Ort::Value output_tensor = Ort::Value::CreateTensor<float>(
      info, output_data, tensor_size, tensor_shape.data(), tensor_shape.size());
  const OrtValue* inputs[4] = {input_tensor, pos_ids, cos_cache, sin_cache};
  MY_LOG(2) << "RoPE input init done." << std::endl;
  OrtValue* outputs[1] = {output_tensor};
  try {
    if (N == num_heads_) {
      MY_LOG(2) << "Invoke rope_built_in_q." << std::endl;
      rope_built_in_q.Invoke(context, inputs, 4, outputs, 1);
    } else {
      MY_LOG(2) << "Invoke rope_built_in_k." << std::endl;
      rope_built_in_k.Invoke(context, inputs, 4, outputs, 1);
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
  }

  MY_LOG(2) << "RoPE done." << std::endl;
}

///
/// @brief concat past and current, then pad (B = 1)
///
/// @param dst      : shape [B, N_kv, T_pad, H]
/// @param past     : shape [B, N_kv, past_S, H]
/// @param current  : shape [B, N_kv, 1, H]
/// @param share_buffer: past_present_share_buffer flag
///
/// @note
/// 1. concat pask_v with current_v to total_v:
///       [B, N_kv, past_S, H] + [B, N_kv,1, H] -> [B, N_kv, T, H], where T =
///       past_S + 1
/// 2. pad 3rd dim of total_v to multiples of 128:
///       [B, N_kv, T, H] to [B, N_kv, T_pad, H], where T_pad is multiples of
///       128
///
///
void pad_concat_kv(uint16_t* dst, uint16_t* past, uint16_t* current, int N_kv,
                   int T, int T_pad, int H, bool share_buffer) {
  std::memset(dst, 0, N_kv * T_pad * H * sizeof(uint16_t));
  int past_S = T - 1;
  /// if past_presenst_share_buffer is true, S stride will be 4096
  int past_s_stride = share_buffer ? 4096 : past_S;
  for (int n = 0; n < N_kv; n++) {
    /// copy past v past_S times, each time copy H elements
    for (int s = 0; s < past_S; s++) {
      std::memcpy(dst + n * T_pad * H + s * H,
                  past + n * past_s_stride * H + s * H, H * sizeof(uint16_t));
    }
    /// pad v and concat new v with past v
    std::memcpy(dst + n * T_pad * H + past_S * H, current + n * 1 * H,
                H * sizeof(uint16_t));
  }
}

void fill_attn_mask_token(uint16_t* atten_mask, int T, int T_pad) {
  //(B,1,1,T) -> (B,1, T,T) //
  // std::memset(atten_mask, 0, T_pad * T_pad * sizeof(uint16_t));
  std::memset(atten_mask, 0, 128 * T_pad * sizeof(uint16_t));
  const uint16_t neg_inf_ui16 = float_to_bfloat16(-3.389e38f);
#ifdef _WIN32
  __stosw(atten_mask + T, neg_inf_ui16, T_pad - T);

  for (int t = 1; t < 128; t++) {
    __stosw(atten_mask + t * T_pad, neg_inf_ui16, T_pad);
  }
#else
  std::fill(atten_mask + T, atten_mask + T_pad, neg_inf_ui16);

  for (int t = 1; t < 128; t++) {
    std::fill(atten_mask + t * T_pad, atten_mask + (t + 1) * T_pad,
              neg_inf_ui16);
  }
#endif
}

void MyCustomOpKernel::set_params() {
  std::vector<size_t> a_shape_1 = {32, MAX_SEQ_LENGTH, 128};
  std::vector<size_t> w_shape_1 = {32, 128, MAX_SEQ_LENGTH};
  bmm1_->set_params("BMM", a_shape_1, w_shape_1);
  std::vector<size_t> a_shape_2 = {32, MAX_SEQ_LENGTH, MAX_SEQ_LENGTH};
  std::vector<size_t> w_shape_2 = {32, MAX_SEQ_LENGTH, 128};
  bmm2_->set_params("BMM", a_shape_2, w_shape_2);
}

void MyCustomOpKernel::LazyInit() {
  dry_run_ = 0;
  if (ENV_PARAM(DRY_RUN) == 1)
    dry_run_ = 1;

  mladf_version_ = ENV_PARAM(MLADF_VERSION);
  std::map<std::string, std::any> attr = {{"op_version", mladf_version_}};
  MY_LOG(2) << "MLADF_VERSION: " << mladf_version_ << std::endl;
  static ryzenai::bmm<uint16_t, uint16_t, uint16_t> bmm1 =
      ryzenai::bmm<uint16_t, uint16_t, uint16_t>("bfloat16", "bfloat16",
                                                 "bfloat16", true, true, attr);
  std::map<std::string, std::any> attr_softmax = {
      {"skip_create_input", 1},
      {"skip_create_output", 1},
      {"op_version", mladf_version_}};
  static ryzenai::masked_softmax<uint16_t, uint16_t, uint16_t> softmax =
      ryzenai::masked_softmax<uint16_t, uint16_t, uint16_t>("bfloat16", true,
                                                            attr_softmax);
  static ryzenai::bmm<uint16_t, uint16_t, uint16_t> bmm2 =
      ryzenai::bmm<uint16_t, uint16_t, uint16_t>("bfloat16", "bfloat16",
                                                 "bfloat16", true, false, attr);
  std::string transpose_type = "input";
  std::map<std::string, std::any> rope_attr = {{"op_version", mladf_version_},
                                               {"transpose", transpose_type}};
  static ryzenai::mha_rope<uint16_t, uint16_t, uint16_t> mha_rope_q =
      ryzenai::mha_rope<uint16_t, uint16_t, uint16_t>("bfloat16", true,
                                                      rope_attr);
  static ryzenai::mha_rope<uint16_t, uint16_t, uint16_t> mha_rope_k =
      ryzenai::mha_rope<uint16_t, uint16_t, uint16_t>("bfloat16", true,
                                                      rope_attr);
  if (rope_q_ == nullptr) {
    rope_q_ = &mha_rope_q;
  }

  if (rope_k_ == nullptr) {
    rope_k_ = &mha_rope_k;
  }

  if (bmm1_ == nullptr) {
    bmm1_ = &bmm1;
  }

  if (bmm2_ == nullptr) {
    bmm2_ = &bmm2;
  }
  if (softmax_ == nullptr) {
    softmax_ = &softmax;
  }
}

void MyCustomOpKernel::LazyInit_matmul_nbits(std::vector<int8_t> b,
                                             std::vector<int8_t> zeros,
                                             std::vector<float> scales,
                                             std::vector<float> bias) {

  std::string mladf_version_("v1");
  if (instances__ == 0) {
    std::map<std::string, std::any> mm_attrs;
    mm_attrs["op_version"] = mladf_version_;
    gemm__ = std::make_shared<
        ryzenai::mladfmatmulbias<uint16_t, int8_t, uint16_t, uint16_t>>(
        "bfloat16", "int4", "bfloat16", true, mm_attrs);
  }
  auto ptr = (ryzenai::mladfmatmulbias<uint16_t, int8_t, uint16_t, uint16_t>*)
                 gemm__.get();

  std::vector<size_t> b_shape_dd = {static_cast<size_t>(k_k),
                                    static_cast<size_t>(k_n)};

  Tensor weight_tensor = {b.data(), b_shape_dd, "int4"};
  Tensor bias_tensor = {bias.data(), {(size_t)k_block_size, 0}, "float"};
  Tensor scales_tensor = {scales.data(), {(size_t)k_block_size, 0}, "float"};
  Tensor zeros_tensor = {zeros.data(), b_shape_dd, "int4"};
  std::vector<Tensor> constant_tensors = {weight_tensor, bias_tensor,
                                          scales_tensor, zeros_tensor};
  std::map<std::string, std::any> attrs;
  attrs["default_shape"] = 1;
  attrs["op_version"] = mladf_version_;
  attrs["group_size"] = static_cast<int>(k_block_size);
  attrs["max_m"] = MAX_SEQ_LENGTH;
  ptr->initialize_const_params(constant_tensors, attrs);
}

MyCustomOpKernel::MyCustomOpKernel(const OrtKernelInfo* k_info,
                                   const OrtApi& api) {
  api_ = &api;
  std::string node_name;
  // Get constant info for the node
  Ort::ConstKernelInfo info{k_info};

  // Get Logger
  m_logger = info.GetLogger();

  // Get attrs
  do_rotary_ = info.GetAttribute<int64_t>("do_rotary");
  kv_num_heads_ = info.GetAttribute<int64_t>("kv_num_heads");
  num_heads_ = info.GetAttribute<int64_t>("num_heads");
  rotary_interleaved_ = info.GetAttribute<int64_t>("rotary_interleaved");
  scale_ = info.GetAttribute<float>("scale");
  //  Get inputs attrs
  m_node_name = info.GetNodeName();

  int is_cos_const = 0;
  const_cos_ = info.GetTensorConstantInput(5, &is_cos_const);
  int is_sin_const = 0;
  const_sin_ = info.GetTensorConstantInput(6, &is_sin_const);
  MY_LOG(2) << "is_cos_const: " << is_cos_const << std::endl;
  MY_LOG(2) << "is_sin_const: " << is_sin_const << std::endl;
  /// check if cos/sin cache is const tensor
  if (is_cos_const == 0 || is_sin_const == 0) {
    is_const_cache = false;
  }
  /// if cos/sin cache is const tensor, then set rotary_embedding_dim to
  /// cos_shape[1] * 2, we need to pass this param to ort RoPE kernel.
  /// if not, set it to defalut value 0.
  int64_t rotary_embedding_dim = 0;
  if (is_const_cache) {
    auto cos_shape = const_cos_.GetTensorTypeAndShapeInfo().GetShape();
    rotary_embedding_dim = cos_shape[1] * 2;
  }

  // onnx built in transpose op
  MY_LOG(2) << "initialization for onnx transpose builtin op..." << std::endl;

  const char* transpose_type_constraint_names[1] = {"T"};
  ONNXTensorElementDataType transpose_type_constraint_values[1] = {
      ONNX_TENSOR_ELEMENT_DATA_TYPE_BFLOAT16};
  std::vector<int64_t> perm_vec{0, 2, 1, 3};
  auto perm = Ort::OpAttr("perm", perm_vec.data(), perm_vec.size(),
                          OrtOpAttrType::ORT_OP_ATTR_INTS);
  Ort::OpAttr transpose_attrs[1] = {std::move(perm)};
  transpose0213_built_in = Ort::Op::Create(
      info.Copy(), "Transpose", "ai.onnx", 21, transpose_type_constraint_names,
      transpose_type_constraint_values, 1, transpose_attrs, 1, 1,
      1); // 1 attributes, 1 input, 1 output
  MY_LOG(2) << "initialization for onnx transpose builtin op done..."
            << std::endl;

  /// ort built in RotaryEmbedding kernel
  MY_LOG(2) << "initialization for onnx rope builtin op..." << std::endl;
  const char* rope_type_constraint_names[2] = {"T", "M"};
  ONNXTensorElementDataType rope_type_constraint_values[2] = {
      ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64};
  // scale
  int64_t val_scale_rope_q = scale_;
  auto attr_scale_rope_q = Ort::OpAttr("scale", &val_scale_rope_q, 1,
                                       OrtOpAttrType::ORT_OP_ATTR_FLOAT);
  // interleaved
  int64_t val_interleaved_q = rotary_interleaved_;
  auto attr_interleaved_q = Ort::OpAttr("interleaved", &val_interleaved_q, 1,
                                        OrtOpAttrType::ORT_OP_ATTR_INT);
  // num_heads for q
  int64_t val_num_heads_q = num_heads_;
  auto attr_num_heads_q = Ort::OpAttr("num_heads", &val_num_heads_q, 1,
                                      OrtOpAttrType::ORT_OP_ATTR_INT);
  int64_t val_rotary_embedding_dim_q = rotary_embedding_dim;
  auto attr_rotary_embedding_dim_q =
      Ort::OpAttr("rotary_embedding_dim", &val_rotary_embedding_dim_q, 1,
                  OrtOpAttrType::ORT_OP_ATTR_INT);
  Ort::OpAttr rope_attrs_q[4] = {
      std::move(attr_scale_rope_q), std::move(attr_interleaved_q),
      std::move(attr_num_heads_q), std::move(attr_rotary_embedding_dim_q)};
  rope_built_in_q =
      Ort::Op::Create(info.Copy(), "RotaryEmbedding", "com.microsoft", 1,
                      rope_type_constraint_names, rope_type_constraint_values,
                      2, rope_attrs_q, 4, 4,
                      1); // 4 attributes, 4 inputs, 1 outputs
  // scale
  int64_t val_scale_rope_k = scale_;
  auto attr_scale_rope_k = Ort::OpAttr("scale", &val_scale_rope_k, 1,
                                       OrtOpAttrType::ORT_OP_ATTR_FLOAT);
  // interleaved
  int64_t val_interleaved_k = rotary_interleaved_;
  auto attr_interleaved_k = Ort::OpAttr("interleaved", &val_interleaved_k, 1,
                                        OrtOpAttrType::ORT_OP_ATTR_INT);
  // num_heads for k
  int64_t val_num_heads_k = kv_num_heads_;
  auto attr_num_heads_k = Ort::OpAttr("num_heads", &val_num_heads_k, 1,
                                      OrtOpAttrType::ORT_OP_ATTR_INT);
  int64_t val_rotary_embedding_dim_k = rotary_embedding_dim;
  auto attr_rotary_embedding_dim_k =
      Ort::OpAttr("rotary_embedding_dim", &val_rotary_embedding_dim_k, 1,
                  OrtOpAttrType::ORT_OP_ATTR_INT);
  Ort::OpAttr rope_attrs_k[4] = {
      std::move(attr_scale_rope_k), std::move(attr_interleaved_k),
      std::move(attr_num_heads_k), std::move(attr_rotary_embedding_dim_k)};
  rope_built_in_k =
      Ort::Op::Create(info.Copy(), "RotaryEmbedding", "com.microsoft", 1,
                      rope_type_constraint_names, rope_type_constraint_values,
                      2, rope_attrs_k, 4, 4,
                      1); // 4 attributes, 4 inputs, 1 outputs
  MY_LOG(2) << "initialization for onnx rope builtin op done..." << std::endl;

  // onnx built in gqo op
  MY_LOG(2) << "initialization for onnx gqo builtin op..." << std::endl;

  const char* gqo_type_constraint_names[1] = {"T"};
  ONNXTensorElementDataType gqo_type_constraint_values[1] = {
      ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT};

  int64_t val_do_rotary = do_rotary_;
  int64_t val_kv_num_heads = kv_num_heads_;
  int64_t val_num_heads = num_heads_;
  int64_t val_rotary_interleaved = rotary_interleaved_;
  float val_scale = scale_;
  auto attr_do_rotary = Ort::OpAttr("do_rotary", &val_do_rotary, 1,
                                    OrtOpAttrType::ORT_OP_ATTR_INT);
  auto attr_kv_num_heads = Ort::OpAttr("kv_num_heads", &val_kv_num_heads, 1,
                                       OrtOpAttrType::ORT_OP_ATTR_INT);
  auto attr_num_heads = Ort::OpAttr("num_heads", &val_num_heads, 1,
                                    OrtOpAttrType::ORT_OP_ATTR_INT);
  auto attr_rotary_interleave =
      Ort::OpAttr("rotary_interleaved", &val_rotary_interleaved, 1,
                  OrtOpAttrType::ORT_OP_ATTR_INT);
  auto attr_scale =
      Ort::OpAttr("scale", &val_scale, 1, OrtOpAttrType::ORT_OP_ATTR_FLOAT);

  Ort::OpAttr gqo_attrs[5] = {
      std::move(attr_do_rotary), std::move(attr_kv_num_heads),
      std::move(attr_num_heads), std::move(attr_rotary_interleave),
      std::move(attr_scale)};
  gqo_built_in =
      Ort::Op::Create(info.Copy(), "GroupQueryAttention", "com.microsoft", 1,
                      gqo_type_constraint_names, gqo_type_constraint_values, 1,
                      gqo_attrs, 5, 9, 3); // 5 attributes, 9 inputs, 3 output
  MY_LOG(2) << "initialization for onnx gqo builtin op done..." << std::endl;

  int is_constant = 0;
  if (ENV_PARAM(USE_AIE_GQO) == 1) {
    // aie mha op from DD
    MY_LOG(2) << "initialization for mha aie custom-op..." << std::endl;

    LazyInit();

    std::call_once(initFlag, [this]() { this->set_params(); });
    bmm1_->debug(false);
    bmm2_->debug(false);
    softmax_->debug(false);
    rope_q_inputs = rope_q_->get_inputs();
    rope_q_outputs = rope_q_->get_outputs();
    rope_k_inputs = rope_k_->get_inputs();
    rope_k_outputs = rope_k_->get_outputs();
    bmm1_inputs = bmm1_->get_inputs();
    bmm1_outputs = bmm1_->get_outputs();
    bmm2_inputs = bmm2_->get_inputs();
    bmm2_outputs = bmm2_->get_outputs();
    softmax_mask = softmax_->get_inputs()[1];

    // initialize the atten_provider
    atten_mask_provider_ =
        std::make_unique<AttenMaskProvider>(&mha_aie_kernel_info_);

    MY_LOG(2) << "initialization for mha aie custom-op done." << std::endl;

    if (ENV_PARAM(USE_AIE_RoPE) == 1 && is_const_cache) {
      MY_LOG(2) << "Getting cos/sin cache as constant input.\n";
      get_rope_cache(const_cos_, const_sin_);
    }
  }
  MY_LOG(2) << "initialization for mat mul nbits custom-op..." << std::endl;
  // Extracting the attributes for MatMul Nbits
  k_n = info.GetAttribute<int64_t>("o_proj_N");
  k_k = info.GetAttribute<int64_t>("o_proj_K");
  k_bits = info.GetAttribute<int64_t>("o_proj_bits");
  k_block_size = info.GetAttribute<int64_t>("o_proj_block_size");

  m_node_name = info.GetNodeName();

  // Get weights
  is_constant = 0;
  m_weights = info.GetTensorConstantInput(7, &is_constant);
  if (is_constant) {
    const uint8_t* value = m_weights.GetTensorData<uint8_t>();
    std::stringstream message;
    message << "- Node: " << m_node_name << " Weights[0] = " << int(value[0]);
    ORT_CXX_LOG(m_logger, ORT_LOGGING_LEVEL_INFO, message.str().c_str());
  }

  // Get scales
  is_constant = 0;
  m_scales = info.GetTensorConstantInput(8, &is_constant);
  if (is_constant) {
    const float* value = m_scales.GetTensorData<float>();
    std::stringstream message;
    message << "- Node: " << m_node_name
            << " Scales[0] = " << std::to_string(value[0]) << "\n";
    ORT_CXX_LOG(m_logger, ORT_LOGGING_LEVEL_INFO, message.str().c_str());
  }

  // Get zero-points
  is_constant = 0;
  m_zeros = info.GetTensorConstantInput(9, &is_constant);
  if (is_constant) {
    m_asymmetric = true;
    const uint8_t* value = m_zeros.GetTensorData<uint8_t>();
    std::stringstream message;
    message << "- Node: " << m_node_name
            << " Zero-points[0] = " << std::to_string(value[0]) << "\n";
    ORT_CXX_LOG(m_logger, ORT_LOGGING_LEVEL_INFO, message.str().c_str());
  } else {
    m_asymmetric = false;
    ORT_CXX_LOG(m_logger, ORT_LOGGING_LEVEL_INFO, "No zero-point");
  }

  m_biased = false;
  std::vector<float> scales(k_k * k_n / k_block_size);
  std::vector<int8_t> b(k_k * k_n, 0);
  size_t kblks = k_k / k_block_size;
  // fill this with zeros for Symmetric quantization
  size_t zp_shape = (k_n * std::floor((float)((kblks + 1) * k_bits) / 8.0f));
  // std::vector<int8_t> zeros(k_k * k_n / k_block_size, 0);
  std::vector<int8_t> zeros(zp_shape * 2, 0);

  // Original weights are in NxK/2 packed as uint8
  // Convert to KXN uint8
  const uint8_t* wts = m_weights.GetTensorData<uint8_t>();
  for (int64_t i = 0; i < k_k; i += 2) {
    for (int64_t j = 0; j < k_n; j++) {
      auto srcv = wts[j * k_k / 2 + i / 2];
      auto src0 = (srcv & 0xf) - 8;
      auto src1 = ((srcv & 0xf0) >> 4) - 8;
      b[i * k_n + j] = static_cast<int8_t>(src0);
      b[(i + 1) * k_n + j] = static_cast<int8_t>(src1);
    }
  }

  // Original Scales are in Nx(K/BlockSize) shape
  // Convert to (K/BLOCK_SIZE)xN shape
  const float* scl = m_scales.GetTensorData<float>();
  for (int i = 0; i < k_n; i++) {
    for (int j = 0; j < kblks; j++) {
      scales[j * k_n + i] = scl[i * kblks + j];
    }
  }

  // fill this with zeros for Symmetric quantization
  if (m_asymmetric) {
    const uint8_t* zero_pt = m_zeros.GetTensorData<uint8_t>();
    int kblks_pad = 2 * zp_shape / k_n;
    for (int i = 0; i < k_n; i++) {
      for (int j = 0; j < kblks_pad; j = j + 2) {
        // auto zpv = zero_pt[(i * (kblks / 2)) + (j / 2)];
        auto zpv = zero_pt[(i * kblks_pad) / 2 + j / 2];
        zeros[j * k_n + i] = (zpv & 0xf) - 8;
        zeros[(j + 1) * k_n + i] = ((zpv & 0xf0) >> 4) - 8;
      }
    }
  }
  // fill this with zeros for MatMul without bias
  std::vector<float> bias(k_n, 0); // fill with zeros
  if (m_biased) {
    const float* m_bias_ptr = m_bias.GetTensorData<float>();
    memcpy(bias.data(), m_bias_ptr, sizeof(float) * k_n);
  }

  n_sizes_.push_back({static_cast<int>(k_k), static_cast<int>(k_n)});
  grp_sizes_.push_back(k_block_size);

  LazyInit_matmul_nbits(b, zeros, scales, bias);
  cnt = instances__++;
  MY_LOG(2) << "initialization for matmul nbits custom-op Done..." << std::endl;
}

MyCustomOpKernel::~MyCustomOpKernel() {
  if (trig_max_len)
#ifdef _WIN32
    _aligned_free(trig_max_len);
#else
    free(trig_max_len);
  gemm__.reset();
  ryzenai::dynamic_dispatch::xrt_context::destroy_ctx_map();

#endif
}

void MyCustomOpKernel::matmul_nbits_aie_execute1(
    uint16_t* input_data, uint16_t* out, std::vector<int64_t> input_shape,
    std::vector<int> wts_shape, int grp_size, int run_cnt) {
  auto ptr = (ryzenai::mladfmatmulbias<uint16_t, int8_t, uint16_t, uint16_t>*)
                 gemm__.get();
  // Ryzen-AI implementation
  int M = input_shape[0] * input_shape[1];

  std::vector<size_t> a_shape = {static_cast<size_t>(M),
                                 static_cast<size_t>(input_shape[2])};

  std::vector<size_t> c_shape = {static_cast<size_t>(M),
                                 static_cast<size_t>(wts_shape[1])};
  std::vector<size_t> wts_shape_dd = {static_cast<size_t>(wts_shape[0]),
                                      static_cast<size_t>(wts_shape[1])};
  ptr->set_shape(a_shape, wts_shape_dd, grp_size);

  Tensor output_tensor = {out, c_shape, "bfloat16"};
  std::vector<Tensor> output_tensors = {output_tensor};

  if (M > 1) {
    // Exec
    // DD Tensors
    Tensor input_tensor = {(int16_t*)input_data, a_shape, "bfloat16"};
    std::vector<Tensor> input_tensors = {input_tensor};
    TRY_EXECUTE_WITH_LOG(
        ptr->execute_internal(input_tensors, output_tensors, run_cnt), dry_run_,
        ReportInventory::getInstance().addData,
        "mladfmatmulbias_" + std::to_string(M),
        std::to_string(M) + "_" + std::to_string(input_shape[2]));
  } else {
    Tensor input_tensor = {(int16_t*)input_data, a_shape, "bfloat16"};
    std::vector<Tensor> input_tensors = {input_tensor};
    TRY_EXECUTE_WITH_LOG(
        ptr->execute_internal(input_tensors, output_tensors, run_cnt), dry_run_,
        ReportInventory::getInstance().addData,
        "mladfmatmulbias_" + std::to_string(M),
        std::to_string(M) + "_" + std::to_string(input_shape[2]));
  }
}

void MyCustomOpKernel::matmul_nbits_aie_execute(
    std::vector<xrt::bo>& inputs, uint16_t* out,
    std::vector<int64_t> output_shape, std::vector<int> wts_shape, int grp_size,
    int run_cnt) {
  int M = output_shape[0] * output_shape[1];

  std::vector<size_t> a_shape = {static_cast<size_t>(M),
                                 static_cast<size_t>(output_shape[2])};

  std::vector<size_t> wts_shape_dd = {static_cast<size_t>(wts_shape[0]),
                                      static_cast<size_t>(wts_shape[1])};
  std::vector<size_t> c_shape = {static_cast<size_t>(M),
                                 static_cast<size_t>(wts_shape[1])};

  auto ptr = (ryzenai::mladfmatmulbias<uint16_t, int8_t, uint16_t, uint16_t>*)
                 gemm__.get();
  ptr->set_shape(a_shape, wts_shape_dd, grp_size);
  auto o_wts = ptr->get_const();
  mm_outputs = ptr->get_outputs(M);
  std::vector<xrt::bo> o_in = {inputs[0], o_wts[run_cnt]};
  // inputs[0].sync(XCL_BO_SYNC_BO_TO_DEVICE);
  // mm_outputs[0].sync(XCL_BO_SYNC_BO_TO_DEVICE);
  MY_LOG(2) << "aie execute." << std::endl;
  TRY_EXECUTE_WITH_LOG(ptr->execute(o_in, mm_outputs), dry_run_,
                       ReportInventory::getInstance().addData,
                       "mladfmatmulbias_" + std::to_string(M),
                       std::to_string(M) + "_" +
                           std::to_string(output_shape[2]));

  mm_outputs[0].sync(XCL_BO_SYNC_BO_FROM_DEVICE);
  uint16_t* mm_out = mm_outputs[0].map<uint16_t*>();
  MY_LOG(2) << "copy output." << std::endl;
  memcpy(out, mm_out, M * wts_shape[1] * sizeof(uint16_t));
  MY_LOG(2) << "aie mm_nbits done." << std::endl;
}

void MyCustomOpKernel::aie_execute_rope(const uint16_t* input, uint16_t* output,
                                        const int N, const int S, const int H,
                                        const int past_S) {
  std::vector<size_t> in_shape{(size_t)N, (size_t)S, (size_t)H};
  rope_q_->set_params("rope", in_shape);
  std::vector<xrt::bo> rope_inbos = rope_q_->get_inputs();
  std::vector<xrt::bo> rope_outbos = rope_q_->get_outputs();

  MY_LOG(2) << "aie rope begin." << std::endl;
  MY_LOG(2) << "NxSxH: " << N << "x" << S << "x" << H << std::endl;
  MY_LOG(2) << "copy input." << std::endl;
  uint16_t* a_bo_map = rope_inbos[0].map<uint16_t*>();
  memcpy((void*)a_bo_map, (void*)input, N * S * H * sizeof(uint16_t));

  uint16_t* b_bo_map = rope_inbos[1].map<uint16_t*>();
  auto trig_max_len_offset = 2 * max_seq_length * cs_1;
  auto b_bo_map_offset = S * H;

  size_t pos_offset = (S == 1) ? (past_S)*H : 0;
  MY_LOG(2) << "===aie rope offset: " << past_S << std::endl;

  MY_LOG(2) << "copy cos." << std::endl;
  memcpy((void*)b_bo_map, (void*)(cos_sin_cache__.get() + pos_offset),
         S * H * sizeof(uint16_t));
  MY_LOG(2) << "copy sin." << std::endl;
  memcpy((void*)(b_bo_map + b_bo_map_offset),
         (void*)(cos_sin_cache__.get() + pos_offset + trig_max_len_offset),
         S * H * sizeof(uint16_t));

  uint16_t* rope_out = rope_outbos[0].map<uint16_t*>();
  rope_inbos[0].sync(XCL_BO_SYNC_BO_TO_DEVICE);
  rope_inbos[1].sync(XCL_BO_SYNC_BO_TO_DEVICE);
  MY_LOG(2) << "aie execute." << std::endl;
  TRY_EXECUTE_WITH_LOG(
      rope_q_->execute(rope_inbos, rope_outbos), dry_run_,
      ReportInventory::getInstance().addData, "mha_rope_" + std::to_string(S),
      std::to_string(N) + "_" + std::to_string(S) + "_" + std::to_string(H));
  rope_outbos[0].sync(XCL_BO_SYNC_BO_FROM_DEVICE);
  MY_LOG(2) << "copy output." << std::endl;
  memcpy(output, rope_out, N * S * H * sizeof(uint16_t));
  MY_LOG(2) << "aie rope done." << std::endl;
}

void MyCustomOpKernel::aie_execute_mha(uint16_t* output, const int N_q,
                                       const int N_kv, const int S,
                                       const int S_pad, const int H) {
  MY_LOG(2) << "Running AIE MHA alone.";
  /// set kernel shapes
  std::vector<size_t> bmm1_shape_a{(size_t)N_q, (size_t)S_pad, (size_t)H};
  std::vector<size_t> bmm1_shape_w{(size_t)N_kv, (size_t)H, (size_t)S_pad};
  std::vector<size_t> softmax_shape{(size_t)N_q, (size_t)S_pad, (size_t)S_pad};
  std::vector<size_t> bmm2_shape_a{(size_t)N_q, (size_t)S_pad, (size_t)S_pad};
  std::vector<size_t> bmm2_shape_w{(size_t)N_kv, (size_t)S_pad, (size_t)H};
  bmm1_->set_execute_kernel_shape(bmm1_shape_a, bmm1_shape_w);
  bmm2_->set_execute_kernel_shape(bmm2_shape_a, bmm2_shape_w);
  softmax_->set_params("softmax", softmax_shape);

  // Sync data
  bmm1_inputs[0].sync(XCL_BO_SYNC_BO_TO_DEVICE);
  bmm1_inputs[1].sync(XCL_BO_SYNC_BO_TO_DEVICE);
  softmax_mask.sync(XCL_BO_SYNC_BO_TO_DEVICE);
  bmm2_inputs[1].sync(XCL_BO_SYNC_BO_TO_DEVICE);

#ifdef _WIN32
  // Execute QKT MatMul
  TRY_EXECUTE_WITH_LOG(bmm1_->execute(bmm1_inputs, bmm1_outputs, false),
                       dry_run_, ReportInventory::getInstance().addData,
                       "bmm_" + std::to_string(S_pad),
                       std::to_string(N_q) + "_" + std::to_string(S_pad) + "_" +
                           std::to_string(H));
#else
  TRY_EXECUTE_WITH_LOG(bmm1_->execute(bmm1_inputs, bmm1_outputs, true),
                       dry_run_, ReportInventory::getInstance().addData,
                       "bmm_" + std::to_string(S_pad),
                       std::to_string(N_q) + "_" + std::to_string(S_pad) + "_" +
                           std::to_string(H));
#endif
  // Sync

  std::vector<xrt::bo> inputs = {bmm1_outputs[0], softmax_mask};
  std::vector<xrt::bo> outputs = {bmm2_inputs[0]};

// Execute Softmax
#ifdef _WIN32
  TRY_EXECUTE_WITH_LOG(softmax_->execute(inputs, outputs, false), dry_run_,
                       ReportInventory::getInstance().addData,
                       "masked_softmax_" + std::to_string(S_pad),
                       std::to_string(N_q) + "_" + std::to_string(S_pad) + "_" +
                           std::to_string(S_pad));
#else
  TRY_EXECUTE_WITH_LOG(softmax_->execute(inputs, outputs, true), dry_run_,
                       ReportInventory::getInstance().addData,
                       "masked_softmax_" + std::to_string(S_pad),
                       std::to_string(N_q) + "_" + std::to_string(S_pad) + "_" +
                           std::to_string(S_pad));
#endif

  // Execute SMV MatMul
  TRY_EXECUTE_WITH_LOG(bmm2_->execute(bmm2_inputs, bmm2_outputs, true),
                       dry_run_, ReportInventory::getInstance().addData,
                       "bmm_" + std::to_string(S_pad),
                       std::to_string(N_q) + "_" + std::to_string(S_pad) + "_" +
                           std::to_string(S_pad));
  // Sync output
  bmm2_outputs[0].sync(XCL_BO_SYNC_BO_FROM_DEVICE);

  // Copy output from XRT BO to OrtTensor
  if (output != nullptr) {
    uint16_t* output_bo = bmm2_outputs[0].map<uint16_t*>();
    memcpy(output, output_bo, N_q * S * H * sizeof(uint16_t));
  }
}

void MyCustomOpKernel::aie_execute_rope_mha(uint16_t* output, const int N_q,
                                            const int N_kv, const int S,
                                            const int S_pad, const int H) {
  MY_LOG(2) << "Running AIE RoPE MHA with BO sharing.";
  /// set kernel shapes
  std::vector<size_t> rope_q_shape{(size_t)N_q, (size_t)S_pad, (size_t)H};
  std::vector<size_t> rope_k_shape{(size_t)N_kv, (size_t)S_pad, (size_t)H};
  std::vector<size_t> bmm1_shape_a{(size_t)N_q, (size_t)S_pad, (size_t)H};
  std::vector<size_t> bmm1_shape_w{(size_t)N_kv, (size_t)H, (size_t)S_pad};
  std::vector<size_t> softmax_shape{(size_t)N_q, (size_t)S_pad, (size_t)S_pad};
  std::vector<size_t> bmm2_shape_a{(size_t)N_q, (size_t)S_pad, (size_t)S_pad};
  std::vector<size_t> bmm2_shape_w{(size_t)N_kv, (size_t)S_pad, (size_t)H};

  rope_q_->set_params("rope", rope_q_shape);
  rope_k_->set_params("rope", rope_k_shape);
  bmm1_->set_execute_kernel_shape(bmm1_shape_a, bmm1_shape_w);
  bmm2_->set_execute_kernel_shape(bmm2_shape_a, bmm2_shape_w);
  softmax_->set_params("softmax", softmax_shape);

  uint16_t* rope_q_cs_bo = rope_q_inputs[1].map<uint16_t*>();
  uint16_t* rope_k_cs_bo = rope_k_inputs[1].map<uint16_t*>();
  auto trig_max_len_offset = 2 * max_seq_length * cs_1;
  auto cs_bo_offset = S_pad * H;

  memcpy((void*)rope_q_cs_bo, (void*)(cos_sin_cache__.get()),
         S_pad * H * sizeof(uint16_t));
  memcpy((void*)(rope_q_cs_bo + cs_bo_offset),
         (void*)(cos_sin_cache__.get() + trig_max_len_offset),
         S_pad * H * sizeof(uint16_t));

  memcpy((void*)rope_k_cs_bo, (void*)(cos_sin_cache__.get()),
         S_pad * H * sizeof(uint16_t));
  memcpy((void*)(rope_k_cs_bo + cs_bo_offset),
         (void*)(cos_sin_cache__.get() + trig_max_len_offset),
         S_pad * H * sizeof(uint16_t));

  rope_q_inputs[0].sync(XCL_BO_SYNC_BO_TO_DEVICE);
  rope_q_inputs[1].sync(XCL_BO_SYNC_BO_TO_DEVICE);
  rope_k_inputs[0].sync(XCL_BO_SYNC_BO_TO_DEVICE);
  rope_k_inputs[1].sync(XCL_BO_SYNC_BO_TO_DEVICE);
  softmax_mask.sync(XCL_BO_SYNC_BO_TO_DEVICE);
  bmm2_inputs[1].sync(XCL_BO_SYNC_BO_TO_DEVICE);

#ifdef _WIN32
  TRY_EXECUTE_WITH_LOG(rope_q_->execute(rope_q_inputs, rope_q_outputs, false),
                       dry_run_, ReportInventory::getInstance().addData,
                       "mha_rope_" + std::to_string(S_pad),
                       std::to_string(N_q) + "_" + std::to_string(S_pad) + "_" +
                           std::to_string(H));

  TRY_EXECUTE_WITH_LOG(rope_k_->execute(rope_k_inputs, rope_k_outputs, false),
                       dry_run_, ReportInventory::getInstance().addData,
                       "mha_rope_" + std::to_string(S_pad),
                       std::to_string(N_q) + "_" + std::to_string(S_pad) + "_" +
                           std::to_string(H));
#else
  TRY_EXECUTE_WITH_LOG(rope_q_->execute(rope_q_inputs, rope_q_outputs, true),
                       dry_run_, ReportInventory::getInstance().addData,
                       "mha_rope_" + std::to_string(S_pad),
                       std::to_string(N_q) + "_" + std::to_string(S_pad) + "_" +
                           std::to_string(H));

  TRY_EXECUTE_WITH_LOG(rope_k_->execute(rope_k_inputs, rope_k_outputs, true),
                       dry_run_, ReportInventory::getInstance().addData,
                       "mha_rope_" + std::to_string(S_pad),
                       std::to_string(N_q) + "_" + std::to_string(S_pad) + "_" +
                           std::to_string(H));
#endif

  std::vector<xrt::bo> bmm1_shared_inputs = {rope_q_outputs[0],
                                             rope_k_outputs[0]};
#ifdef _WIN32
  // Execute QKT MatMul
  TRY_EXECUTE_WITH_LOG(bmm1_->execute(bmm1_shared_inputs, bmm1_outputs, false),
                       dry_run_, ReportInventory::getInstance().addData,
                       "bmm_" + std::to_string(S_pad),
                       std::to_string(N_kv) + "_" + std::to_string(S_pad) +
                           "_" + std::to_string(H));
#else
  TRY_EXECUTE_WITH_LOG(bmm1_->execute(bmm1_shared_inputs, bmm1_outputs, true),
                       dry_run_, ReportInventory::getInstance().addData,
                       "bmm_" + std::to_string(S_pad),
                       std::to_string(N_kv) + "_" + std::to_string(S_pad) +
                           "_" + std::to_string(H));
#endif

  std::vector<xrt::bo> sm_inputs = {bmm1_outputs[0], softmax_mask};
  std::vector<xrt::bo> sm_outputs = {bmm2_inputs[0]};

#ifdef _WIN32
  // Execute Softmax
  TRY_EXECUTE_WITH_LOG(softmax_->execute(sm_inputs, sm_outputs, false),
                       dry_run_, ReportInventory::getInstance().addData,
                       "masked_softmax_" + std::to_string(S_pad),
                       std::to_string(N_q) + "_" + std::to_string(S_pad) + "_" +
                           std::to_string(S_pad));

#else
  TRY_EXECUTE_WITH_LOG(softmax_->execute(sm_inputs, sm_outputs, true), dry_run_,
                       ReportInventory::getInstance().addData,
                       "masked_softmax_" + std::to_string(S_pad),
                       std::to_string(N_q) + "_" + std::to_string(S_pad) + "_" +
                           std::to_string(S_pad));
#endif
  // Execute SMV MatMul
  TRY_EXECUTE_WITH_LOG(bmm2_->execute(bmm2_inputs, bmm2_outputs, true),
                       dry_run_, ReportInventory::getInstance().addData,
                       "bmm_" + std::to_string(S_pad),
                       std::to_string(N_q) + "_" + std::to_string(S_pad) + "_" +
                           std::to_string(S_pad));

  // Sync output
  bmm2_outputs[0].sync(XCL_BO_SYNC_BO_FROM_DEVICE);

  // Copy output from XRT BO to OrtTensor
  if (output != nullptr) {
    uint16_t* output_bo = bmm2_outputs[0].map<uint16_t*>();
    memcpy(output, output_bo, S * N_q * H * sizeof(uint16_t));
  }
  rope_q_outputs[0].sync(XCL_BO_SYNC_BO_FROM_DEVICE);
  rope_k_outputs[0].sync(XCL_BO_SYNC_BO_FROM_DEVICE);
}

void MyCustomOpKernel::aie_execute_token(uint16_t* output, int N_q, int N_kv,
                                         int S, int T, int H) {

  std::vector<size_t> bmm1_shape_a{(size_t)N_q, (size_t)S, (size_t)H};
  std::vector<size_t> bmm1_shape_w{(size_t)N_kv, (size_t)H, (size_t)T};
  std::vector<size_t> softmax_shape{(size_t)N_q, (size_t)128, (size_t)T};
  std::vector<size_t> bmm2_shape_a{(size_t)N_q, (size_t)S, (size_t)T};
  std::vector<size_t> bmm2_shape_w{(size_t)N_kv, (size_t)T, (size_t)H};

  MY_LOG(2) << "AIE excute token params: \n"
            << "  N_q: " << N_q << " N_kv: " << N_kv << "  S: " << S
            << " H: " << H << " T: " << T;

  // Set kernel shape
  bmm1_->set_execute_kernel_shape(bmm1_shape_a, bmm1_shape_w);
  bmm2_->set_execute_kernel_shape(bmm2_shape_a, bmm2_shape_w);
  softmax_->set_params("softmax", softmax_shape);

  // Execute QKT MatMul
  MY_LOG(2) << "BMM1 execute.";
  __TIC__(AIET_BMM_QKT)
  bmm1_->execute(bmm1_inputs, bmm1_outputs, false);
  __TOC__(AIET_BMM_QKT)

  // Set softmax inputs/outputs
  std::vector<xrt::bo> sm_inputs = {bmm1_outputs[0], softmax_mask};
  std::vector<xrt::bo> sm_outputs = {bmm2_inputs[0]};

  // Execute Softmax
  MY_LOG(2) << "AIE softmax execute.";
  __TIC__(AIET_Softmax)
  softmax_->execute(sm_inputs, sm_outputs, false);
  __TOC__(AIET_Softmax)

  // Execute SMV MatMul
  MY_LOG(2) << "BMM2 execute.";
  __TIC__(AIET_BMM_SfmV)
  bmm2_->execute(bmm2_inputs, bmm2_outputs, true);
  __TOC__(AIET_BMM_SfmV)

  // Sync output
  __TIC__(AIET_SYNC_OUT)
  bmm2_outputs[0].sync(XCL_BO_SYNC_BO_FROM_DEVICE);
  __TOC__(AIET_SYNC_OUT)

  // Copy output from XRT BO to OrtTensor
  if (output != nullptr) {
    uint16_t* out_bo_map = bmm2_outputs[0].map<uint16_t*>();
    memcpy(output, out_bo_map, N_q * 1 * H * sizeof(uint16_t));
  }
}

static void GetInputTensorData(DataPtrWrapper& data_ptr, int data_type,
                               Ort::ConstValue& input) {
  if (data_type == 16) {
    data_ptr.T = (void*)input.GetTensorData<uint16_t>();
    data_ptr.dtag = "bf16";
  } else if (data_type == 1) {
    data_ptr.T = (void*)input.GetTensorData<float>();
    data_ptr.dtag = "float32";
  } else {
    throw std::runtime_error("Not supported data type, tag:" +
                             std::string(std::to_string(data_type)));
  }
}

static void GetOutputTensorMutableData(DataPtrWrapper& data_ptr, int data_type,
                                       Ort::UnownedValue& output) {
  if (data_type == 16) {
    data_ptr.T = (void*)output.GetTensorMutableData<uint16_t>();
    data_ptr.dtag = "bf16";
  } else if (data_type == 1) {
    data_ptr.T = (void*)output.GetTensorMutableData<float>();
    data_ptr.dtag = "float32";
  } else {
    throw std::runtime_error("Not supported data type, tag:" +
                             std::string(std::to_string(data_type)));
  }
}

bool isBf16Model(const DataPtrWrapper& packed_qkv, DataPtrWrapper& present_key,
                 DataPtrWrapper& present_value) {
  return packed_qkv.is_bf16() && present_key.is_bf16() &&
         present_value.is_bf16();
}

/// @brief save present_k/v[B, N, S, H] to shared buffer[B, N, S_buffer, H].
void save_present_kv_to_shared_buffer(uint16_t* dst_k, uint16_t* dst_v,
                                      const uint16_t* src_k,
                                      const uint16_t* src_v,
                                      const int num_heads,
                                      const int buffer_seq_len,
                                      const int seq_len, const int head_size) {
  for (int n = 0; n < num_heads; n++) {
    int offset_dst = n * buffer_seq_len * head_size;
    int offset_src = n * seq_len * head_size;
    int copy_size = seq_len * head_size;
    std::memcpy(dst_k + offset_dst, src_k + offset_src,
                copy_size * sizeof(uint16_t));
    std::memcpy(dst_v + offset_dst, src_v + offset_src,
                copy_size * sizeof(uint16_t));
  }
}

void GetQKVFromPackedQKV(uint16_t* q, uint16_t* k, uint16_t* v,
                         uint16_t* packed_qkv, int N_q, int N_kv, int S,
                         int H) {
  for (int s = 0; s < S; s++) {
    std::memcpy(q, packed_qkv, N_q * H * sizeof(uint16_t));
    std::memcpy(k, packed_qkv + N_q * H, N_kv * H * sizeof(uint16_t));
    std::memcpy(v, packed_qkv + (N_q + N_kv) * H, N_kv * H * sizeof(uint16_t));
    packed_qkv += (N_q + 2 * N_kv) * H;
    q += N_q * H;
    k += N_kv * H;
    v += N_kv * H;
  }
}

struct SplitQKVData {
  /// qkv
  const uint16_t* packed_qkv = {};
  uint16_t* q = {};
  uint16_t* k = {};
  uint16_t* v = {};
  /// params
  const int N_q = 0;
  const int N_kv = 0;
  const int S = 0;
  const int H = 0;
};

void Split_QKV(void* raw_data, size_t n) {
  auto data = reinterpret_cast<SplitQKVData*>(raw_data);
  int N_q = data->N_q;
  int N_kv = data->N_kv;
  int S = data->S;
  int H = data->H;
  auto q_src_offset = n * (N_q + 2 * N_kv) * H;
  auto k_src_offset = q_src_offset + N_q * H;
  auto v_src_offset = k_src_offset + N_kv * H;
  auto q_dst_offset = n * N_q * H;
  auto k_dst_offset = n * N_kv * H;
  auto v_dst_offset = n * N_kv * H;
  std::memcpy(data->q + q_dst_offset, data->packed_qkv + q_src_offset,
              N_q * H * sizeof(uint16_t));
  std::memcpy(data->k + k_dst_offset, data->packed_qkv + k_src_offset,
              N_kv * H * sizeof(uint16_t));
  std::memcpy(data->v + v_dst_offset, data->packed_qkv + v_src_offset,
              N_kv * H * sizeof(uint16_t));
}

void MyCustomOpKernel::Compute(OrtKernelContext* context) {
  MY_LOG(2) << "- AMD GQA compute start ...\n";
  __TIC__(Compute)
  Ort::KernelContext ctx(context);
  auto num_inputs = ctx.GetInputCount();
  auto num_outputs = ctx.GetOutputCount();

  MY_LOG(2) << "num_inputs " << num_inputs << " "
            << "num_outputs " << num_outputs << " \n";

  // Prepare input/output tensors
  // Extracting the input and output information
  MY_LOG(2) << "Getting inputs...\n";
  auto packed_qkv = ctx.GetInput(0);
  auto key = ctx.GetInput(1);
  auto value = ctx.GetInput(2);
  auto past_k = ctx.GetInput(1);
  auto past_v = ctx.GetInput(2);
  auto seqlens_k = ctx.GetInput(3);
  auto total_seqlen = ctx.GetInput(4);
  auto cos_cache = ctx.GetInput(5);
  auto sin_cache = ctx.GetInput(6);
  uint16_t* mm_inp = nullptr;
  uint16_t* output_ptr1 = nullptr;
  uint16_t* output_ptr = nullptr;

  MY_LOG(2) << "Getting inputs shape and data...\n";
  // bool is_packed_qkv = key == nullptr ? true : false;
  bool is_packed_qkv = true;

  Ort::AllocatorWithDefaultOptions allocator;
  auto& GQO_Allocator = GQO_Allocator::get_instance();

  // when RoPE cache is dynamic, we get the cache from input when the first op
  // instance call Compute().
  if (ENV_PARAM(USE_AIE_RoPE) && !is_const_cache && compute_instances__ == 0) {
    MY_LOG(2) << "Getting cos/sin cache dynamicly.\n";
    compute_instances__++;
    get_rope_cache(cos_cache, sin_cache);
  }

  // Query data / shape
  auto qkv_data_type = packed_qkv.GetTensorTypeAndShapeInfo().GetElementType();
  auto qkv_shape = packed_qkv.GetTensorTypeAndShapeInfo().GetShape();
  auto qkv_size = packed_qkv.GetTensorTypeAndShapeInfo().GetElementCount();

  DataPtrWrapper qkv_data;
  GetInputTensorData(qkv_data, qkv_data_type, packed_qkv);
  int batch_size = qkv_shape[0];
  int seq_len = qkv_shape[1];
  int hidden_size = qkv_shape[2];
  int head_size = is_packed_qkv
                      ? (hidden_size / (num_heads_ + 2 * kv_num_heads_))
                      : (hidden_size / num_heads_);

  MY_LOG(2) << "packed qkv shape: " << shape2str(qkv_shape) << std::endl;
  MY_LOG(2) << "packed qkv size: " << qkv_size << std::endl;
  MY_LOG(2) << "Packed qkv data: " << qkv_data.toString();
  DataPtrWrapper k_data;
  DataPtrWrapper v_data;
  uint16_t* q_data_ptr = nullptr;
  uint16_t* k_data_ptr = nullptr;
  uint16_t* v_data_ptr = nullptr;
  size_t q_size = batch_size * num_heads_ * seq_len * head_size;
  size_t kv_size = batch_size * kv_num_heads_ * seq_len * head_size;
  if (is_packed_qkv) {
    __TIC__(SplitQKV)
    q_data_ptr = GQO_Allocator.get_buffer_generic<uint16_t>(
        q_size * sizeof(uint16_t), GQO_Allocator::BufferType::AIE_Q);
    k_data_ptr = GQO_Allocator.get_buffer_generic<uint16_t>(
        kv_size * sizeof(uint16_t), GQO_Allocator::BufferType::AIE_K);
    v_data_ptr = GQO_Allocator.get_buffer_generic<uint16_t>(
        kv_size * sizeof(uint16_t), GQO_Allocator::BufferType::AIE_V);
    SplitQKVData split_qkv_data = {qkv_data.cast<uint16_t>(),
                                   q_data_ptr,
                                   k_data_ptr,
                                   v_data_ptr,
                                   static_cast<int>(num_heads_),
                                   static_cast<int>(kv_num_heads_),
                                   seq_len,
                                   head_size};
    MY_LOG(2) << "ParallelFor Split_QKV.";
    ctx.ParallelFor(Split_QKV, static_cast<size_t>(seq_len), 0,
                    &split_qkv_data);
    // GetQKVFromPackedQKV(q_data_ptr, k_data_ptr, v_data_ptr,
    //                     qkv_data.cast<uint16_t>(), num_heads_, kv_num_heads_,
    //                     seq_len, head_size);
    __TOC__(SplitQKV)
  } else {
    auto k_data_type = key.GetTensorTypeAndShapeInfo().GetElementType();
    auto k_shape = key.GetTensorTypeAndShapeInfo().GetShape();
    auto k_size = key.GetTensorTypeAndShapeInfo().GetElementCount();
    GetInputTensorData(k_data, k_data_type, key);
    auto v_data_type = value.GetTensorTypeAndShapeInfo().GetElementType();
    auto v_shape = value.GetTensorTypeAndShapeInfo().GetShape();
    auto v_size = value.GetTensorTypeAndShapeInfo().GetElementCount();
    GetInputTensorData(v_data, v_data_type, value);
    q_data_ptr = qkv_data.cast<uint16_t>();
    k_data_ptr = k_data.cast<uint16_t>();
    v_data_ptr = v_data.cast<uint16_t>();
  }

  // Past Key data / shape
  auto past_k_shape = past_k.GetTensorTypeAndShapeInfo().GetShape();
  auto past_k_size = past_k.GetTensorTypeAndShapeInfo().GetElementCount();
  auto past_k_type = past_k.GetTensorTypeAndShapeInfo().GetElementType();
  DataPtrWrapper past_k_data;
  GetInputTensorData(past_k_data, past_k_type, past_k);
  MY_LOG(2) << "past k shape: " << shape2str(past_k_shape) << std::endl;
  MY_LOG(2) << "past k size: " << past_k_size << std::endl;

  // Past Value data / shape
  auto past_v_shape = past_v.GetTensorTypeAndShapeInfo().GetShape();
  auto past_v_size = past_v.GetTensorTypeAndShapeInfo().GetElementCount();
  auto past_v_type = past_v.GetTensorTypeAndShapeInfo().GetElementType();
  DataPtrWrapper past_v_data;
  GetInputTensorData(past_v_data, past_v_type, past_v);
  MY_LOG(2) << "past v shape: " << shape2str(past_v_shape) << std::endl;
  MY_LOG(2) << "past v size: " << past_v_size << std::endl;

  const int32_t* seq_len_k = seqlens_k.GetTensorData<int32_t>();
  int total_seq_len = total_seqlen.GetTensorData<int32_t>()[0];
  MY_LOG(2) << "k seq len: " << seq_len_k[0] << std::endl;
  MY_LOG(2) << "total seq len: " << total_seq_len << std::endl;

  __TIC__(CAL_POSID)
  /// pos_ids for RoPE
  std::vector<int64_t> pos_ids(seq_len == 1 ? batch_size : 1);
  if (seq_len == 1) {
    for (int b = 0; b < batch_size; b++) {
      pos_ids[b] = static_cast<int64_t>(seq_len_k[b]);
    }
  } else {
    pos_ids[0] = static_cast<int64_t>(0);
  }
  __TOC__(CAL_POSID)

  __TIC__(ALLOC_OUTPUT)
  MY_LOG(2) << "Getting outputs...\n";
  // Allocate output (primary output shape is same as query shape
  // [batch_size, sequence_length, hidden_size])
  std::vector<int64_t> output_shape(
      {static_cast<int64_t>(batch_size), static_cast<int64_t>(seq_len),
       static_cast<int64_t>(head_size * num_heads_)});
  // Convert to xrt bo
  //  auto output = ctx.GetOutput(0, output_shape);
  //  auto output_data_type =
  //  output.GetTensorTypeAndShapeInfo().GetElementType(); auto output_size =
  //  output.GetTensorTypeAndShapeInfo().GetElementCount(); DataPtrWrapper
  //  output_data; GetOutputTensorMutableData(output_data, output_data_type,
  //  output); MY_LOG(2) << "output shape: " << shape2str(output_shape) <<
  //  std::endl; MY_LOG(2) << "output size: " << output_size << std::endl;
  //  MY_LOG(2) << "output data: " << output_data.toString();

  MY_LOG(2) << "calculate shape for present k/v...\n";
  // calculate shape for present k/v
  int past_sequence_length = static_cast<int>(past_k_shape[2]);
  int total_sequence_length = total_seq_len > past_sequence_length
                                  ? total_seq_len
                                  : past_sequence_length;
  bool past_present_share_buffer =
      past_sequence_length == total_sequence_length;
  std::vector<int64_t> present_k_shape(
      {static_cast<int64_t>(batch_size), static_cast<int64_t>(kv_num_heads_),
       static_cast<int64_t>(total_sequence_length),
       static_cast<int64_t>(head_size)});
  std::vector<int64_t> present_v_shape(
      {static_cast<int64_t>(batch_size), static_cast<int64_t>(kv_num_heads_),
       static_cast<int64_t>(total_sequence_length),
       static_cast<int64_t>(head_size)});
  MY_LOG(2) << "present k shape: " << shape2str(present_k_shape) << std::endl;
  MY_LOG(2) << "present v shape: " << shape2str(present_v_shape) << std::endl;
  MY_LOG(2) << "Getting present_key and present_value...\n";
  auto present_k = ctx.GetOutput(0, present_k_shape);
  auto present_k_size = present_k.GetTensorTypeAndShapeInfo().GetElementCount();
  auto present_k_data_type =
      present_k.GetTensorTypeAndShapeInfo().GetElementType();
  DataPtrWrapper present_k_data;
  GetOutputTensorMutableData(present_k_data, present_k_data_type, present_k);
  MY_LOG(2) << "present_key data: " << present_k_data.toString();

  auto present_v = ctx.GetOutput(1, present_v_shape);
  auto present_v_size = present_v.GetTensorTypeAndShapeInfo().GetElementCount();
  auto present_v_data_type =
      present_v.GetTensorTypeAndShapeInfo().GetElementType();
  DataPtrWrapper present_v_data;
  GetOutputTensorMutableData(present_v_data, present_v_data_type, present_v);
  MY_LOG(2) << "present_value data: " << present_v_data.toString();
  MY_LOG(2) << "present k shape: " << shape2str(present_k_shape) << std::endl;
  MY_LOG(2) << "present k size: " << present_k_size << std::endl;
  MY_LOG(2) << "present v shape: " << shape2str(present_v_shape) << std::endl;
  MY_LOG(2) << "present v size: " << present_v_size << std::endl;
  __TOC__(ALLOC_OUTPUT)

  MY_LOG(2) << "B: " << batch_size << std::endl;
  MY_LOG(2) << "S: " << seq_len << std::endl;
  MY_LOG(2) << "N_q: " << num_heads_ << std::endl;
  MY_LOG(2) << "N_kv: " << kv_num_heads_ << std::endl;
  MY_LOG(2) << "H: " << head_size << std::endl;

  int B = batch_size;         // batch
  int S = seq_len;            // sequence length
  int N_q = num_heads_;       // num_heads
  int N_kv = kv_num_heads_;   // kv num_heads
  int group_num = N_q / N_kv; // group num
  int H = head_size;          // head size
  int past_S = seq_len_k[0];
  int T = total_seq_len;

  size_t k_size = B * N_kv * S * H;

  bool is_prefill = check_prefill(seq_len);
  MY_LOG(2) << "is_prefill: " << is_prefill << std::endl;

  // Note(ltp): Using aie kernel when:
  // - prefill phase
  // - USE_AIE_GQO = 1
  // - S <= aie max S length
  if (is_prefill && ENV_PARAM(USE_AIE_GQO) == 1 &&
      S <= mha_aie_kernel_info_.max_seq_length()) {
    MY_LOG(2) << "running AIE kernel" << std::endl;
    if (isBf16Model(qkv_data, present_k_data, present_v_data)) {
      int64_t S_pad = mha_aie_kernel_info_.try_pad_seq(S);

      uint16_t* mask_bo_map = softmax_mask.map<uint16_t*>();
      uint16_t* q_bo_map = bmm1_inputs[0].map<uint16_t*>();
      uint16_t* k_bo_map = bmm1_inputs[1].map<uint16_t*>();
      uint16_t* v_bo_map = bmm2_inputs[1].map<uint16_t*>();

      uint16_t* bf16_K_BNSH = nullptr;
      /// transpose V from [B, S, N_kv, H] to [B, N_kv, S, H]
      MY_LOG(2) << "Transposing V." << std::endl;
      uint16_t* bf16_V_BNSH = GQO_Allocator.get_buffer_generic<uint16_t>(
          kv_size * sizeof(uint16_t), GQO_Allocator::BufferType::AIE_V_T);
      __TIC__(TransposeV)
      transpose0213(bf16_V_BNSH, v_data_ptr, B, S, N_kv, H, context);
      __TOC__(TransposeV)

      /// Getting attention mask from LUT
      MY_LOG(2) << "Filling attn mask." << std::endl;
      __TIC__(AllocFillAttnMask)
      size_t attention_mask_size = B * 1 * S * S;
      assert(atten_mask_provider_ != nullptr);
      uint16_t* bf16_attention_mask = atten_mask_provider_->get_atten_mask(S);
      __TOC__(AllocFillAttnMask)

      /// Try pad qkv and mask in S dimension to {256, 512, 1024, 2048}
      try_pad_qkv(v_bo_map, bf16_V_BNSH, N_kv, S, S_pad, H);
      try_pad_mask(mask_bo_map, bf16_attention_mask, S, S_pad);

      if (ENV_PARAM(USE_AIE_RoPE) == 1 &&
          mha_aie_kernel_info_.is_seq_aie_supported(S)) {
        uint16_t* rope_q_in_bo = rope_q_inputs[0].map<uint16_t*>();
        uint16_t* rope_k_in_bo = rope_k_inputs[0].map<uint16_t*>();
        try_pad_qkv(rope_q_in_bo, q_data_ptr, N_q, S, S_pad, H);
        try_pad_qkv(rope_k_in_bo, k_data_ptr, N_kv, S, S_pad, H);

        const_cast<MyCustomOpKernel*>(this)->aie_execute_rope_mha(
            nullptr, N_q, N_kv, S, S_pad, H);
        bf16_K_BNSH = rope_k_outputs[0].map<uint16_t*>();
        ;
      } else {
        MY_LOG(2) << "Using ORT RoPE for Query." << std::endl;
        uint16_t* bf16_q_rope = GQO_Allocator.get_buffer_generic<uint16_t>(
            q_size * sizeof(uint16_t), GQO_Allocator::BufferType::AIE_Q_ROPE);
        uint16_t* bf16_k_rope = GQO_Allocator.get_buffer_generic<uint16_t>(
            kv_size * sizeof(uint16_t), GQO_Allocator::BufferType::AIE_K_ROPE);
        __TIC__(TransposeQK)
        uint16_t* bf16_q_data_transposed =
            GQO_Allocator.get_buffer_generic<uint16_t>(
                q_size * sizeof(uint16_t), GQO_Allocator::BufferType::AIE_Q_T);
        uint16_t* bf16_k_data_transposed =
            GQO_Allocator.get_buffer_generic<uint16_t>(
                kv_size * sizeof(uint16_t), GQO_Allocator::BufferType::AIE_K_T);
        transpose0213(bf16_q_data_transposed, q_data_ptr, B, S, N_q, H,
                      context);
        transpose0213(bf16_k_data_transposed, k_data_ptr, B, S, N_kv, H,
                      context);
        __TOC__(TransposeQK)
        __TIC__(QueryBF16toFP32)
        float* fp32_q_data = GQO_Allocator.get_buffer_generic<float>(
            q_size * sizeof(float), GQO_Allocator::BufferType::AIE_F_Q);
        float* fp32_q_rope = GQO_Allocator.get_buffer_generic<float>(
            q_size * sizeof(float), GQO_Allocator::BufferType::AIE_F_Q_ROPE);
        vec_bf16_to_float(fp32_q_data, bf16_q_data_transposed, q_size);
        __TOC__(QueryBF16toFP32)
        __TIC__(RoPEQuery)
        RoPE(fp32_q_rope, fp32_q_data, pos_ids.data(), cos_cache, sin_cache, B,
             N_q, S, H, context);
        __TOC__(RoPEQuery)
        __TIC__(QueryFP32toBF16)
        vec_float32_to_bf16(bf16_q_rope, fp32_q_rope, q_size);
        __TOC__(QueryFP32toBF16)

        __TIC__(KeyBF16toFP32)
        float* fp32_k_data = GQO_Allocator.get_buffer_generic<float>(
            kv_size * sizeof(float), GQO_Allocator::BufferType::AIE_F_K);
        float* fp32_k_rope = GQO_Allocator.get_buffer_generic<float>(
            kv_size * sizeof(float), GQO_Allocator::BufferType::AIE_F_K_ROPE);
        vec_bf16_to_float(fp32_k_data, bf16_k_data_transposed, kv_size);
        __TOC__(KeyBF16toFP32)
        __TIC__(RoPEKey)
        RoPE(fp32_k_rope, fp32_k_data, pos_ids.data(), cos_cache, sin_cache, B,
             N_kv, S, H, context);
        __TOC__(RoPEKey)
        __TIC__(KeyFP32toBF16)
        vec_float32_to_bf16(bf16_k_rope, fp32_k_rope, kv_size);
        __TOC__(KeyFP32toBF16)
        bf16_K_BNSH = bf16_k_rope;
        try_pad_qkv(q_bo_map, bf16_q_rope, N_q, S, S_pad, H);
        try_pad_qkv(k_bo_map, bf16_k_rope, N_kv, S, S_pad, H);
        const_cast<MyCustomOpKernel*>(this)->aie_execute_mha(nullptr, N_q, N_kv,
                                                             S, S_pad, H);
      }

      MY_LOG(2) << "save present k/v." << std::endl;
      /// save present k/v
      __TIC__(SavePresentKV)
      if (past_present_share_buffer) {
        save_present_kv_to_shared_buffer(present_k_data.cast<uint16_t>(),
                                         present_v_data.cast<uint16_t>(),
                                         bf16_K_BNSH, bf16_V_BNSH, N_kv,
                                         past_k_shape[2], seq_len, head_size);
      } else {
        std::memcpy(present_k_data.cast<uint16_t>(), bf16_K_BNSH,
                    kv_size * sizeof(uint16_t));
        std::memcpy(present_v_data.cast<uint16_t>(), bf16_V_BNSH,
                    kv_size * sizeof(uint16_t));
      }
      __TOC__(SavePresentKV)
    } else {
      throw std::runtime_error(
          "Not supported now, only support QKV with bfloat16 as inputs.");
    }
    if (!(mha_aie_kernel_info_.is_seq_aie_supported(S))) {
      output_ptr = (uint16_t*)malloc(N_q * S * H * sizeof(uint16_t));
      uint16_t* output_bo = bmm2_outputs[0].map<uint16_t*>();
      memcpy(output_ptr, output_bo, N_q * S * H * sizeof(uint16_t));
      mm_inp = output_ptr;
    }

  } else if ((!is_prefill) && ENV_PARAM(USE_AIE_GQO) == 1 &&
             ENV_PARAM(USE_AIE_TOKEN) &&
             T < mha_aie_kernel_info_.max_seq_length()) {
    /// AIE Token phase
    MY_LOG(2) << "AIE Token phase begin." << std::endl;
    __TIC__(AIETokenPhase)
    /// RoPE for Q
    MY_LOG(2) << "AIE Token phase RoPE for Q." << std::endl;
    uint16_t* bf16_q_rope = GQO_Allocator.get_buffer_generic<uint16_t>(
        q_size * sizeof(uint16_t), GQO_Allocator::BufferType::AIE_Q_ROPE);
    uint16_t* bf16_k_rope = GQO_Allocator.get_buffer_generic<uint16_t>(
        kv_size * sizeof(uint16_t), GQO_Allocator::BufferType::AIE_K_ROPE);

    if (ENV_PARAM(USE_AIE_RoPE) == 1) {
      MY_LOG(2) << "====Using AIE RoPE for Query and Key." << std::endl;
      __TIC__(AIETRoPEQ)
      aie_execute_rope(q_data_ptr, bf16_q_rope, N_q, S, H, past_S);
      __TOC__(AIETRoPEQ)
      __TIC__(AIETRoPEK)
      aie_execute_rope(k_data_ptr, bf16_k_rope, N_kv, S, H, past_S);
      __TOC__(AIETRoPEK)
    } else {
      MY_LOG(2) << "====Using ORT RoPE for Query and Key." << std::endl;
      uint16_t* bf16_q_data_transposed =
          GQO_Allocator.get_buffer_generic<uint16_t>(
              q_size * sizeof(uint16_t), GQO_Allocator::BufferType::AIE_Q_T);
      uint16_t* bf16_k_data_transposed =
          GQO_Allocator.get_buffer_generic<uint16_t>(
              kv_size * sizeof(uint16_t), GQO_Allocator::BufferType::AIE_K_T);
      __TIC__(TransposeQK)
      transpose0213(bf16_q_data_transposed, q_data_ptr, B, S, N_q, H, context);
      transpose0213(bf16_k_data_transposed, k_data_ptr, B, S, N_kv, H, context);
      __TOC__(TransposeQK)

      float* fp32_q_data = GQO_Allocator.get_buffer_generic<float>(
          q_size * sizeof(float), GQO_Allocator::BufferType::AIE_F_Q);
      float* fp32_q_rope = GQO_Allocator.get_buffer_generic<float>(
          q_size * sizeof(float), GQO_Allocator::BufferType::AIE_F_Q_ROPE);
      __TIC__(QueryBF16toFP32)
      vec_bf16_to_float(fp32_q_data, bf16_q_data_transposed, q_size);
      __TOC__(QueryBF16toFP32)

      /// ORT RoPE for Query
      __TIC__(RoPEQuery)
      RoPE(fp32_q_rope, fp32_q_data, pos_ids.data(), cos_cache, sin_cache, B,
           N_q, S, H, context);
      __TOC__(RoPEQuery)

      __TIC__(QueryFP32toBF16)
      vec_float32_to_bf16(bf16_q_rope, fp32_q_rope, q_size);
      __TOC__(QueryFP32toBF16)

      __TIC__(KeyBF16toFP32)
      float* fp32_k_data = GQO_Allocator.get_buffer_generic<float>(
          kv_size * sizeof(float), GQO_Allocator::BufferType::AIE_F_K);
      float* fp32_k_rope = GQO_Allocator.get_buffer_generic<float>(
          kv_size * sizeof(float), GQO_Allocator::BufferType::AIE_F_K_ROPE);
      vec_bf16_to_float(fp32_k_data, bf16_k_data_transposed, kv_size);
      __TOC__(KeyBF16toFP32)

      /// ORT RoPE for Key
      __TIC__(RoPEKey)
      RoPE(fp32_k_rope, fp32_k_data, pos_ids.data(), cos_cache, sin_cache, B,
           N_kv, S, H, context);
      __TOC__(RoPEKey)

      __TIC__(KeyFP32toBF16)
      vec_float32_to_bf16(bf16_k_rope, fp32_k_rope, kv_size);
      __TOC__(KeyFP32toBF16)
    }

    MY_LOG(2) << "AIE Token phase Pad and Concat K/V." << std::endl;
    int64_t T_pad = mha_aie_kernel_info_.try_pad_total_seq(T);

    uint16_t* total_k_map = bmm1_inputs[1].map<uint16_t*>();
    auto func_pad_concat_k = [&]() {
      /// Pad and concat k
      /// 1. concat pask_k with current_k to total_k.
      /// 2. pad total_k from [B, N_kv, T, H] to [B, N_kv, T_pad, H], where
      /// T_pad is multiples of 128
      __TIC__(AIET_PadConcatK)
      pad_concat_kv(total_k_map, past_k_data.cast<uint16_t>(), bf16_k_rope,
                    N_kv, T, T_pad, H, past_present_share_buffer);
      bmm1_inputs[1].sync(XCL_BO_SYNC_BO_TO_DEVICE);
      __TOC__(AIET_PadConcatK)
    };

    uint16_t* total_v_map = bmm2_inputs[1].map<uint16_t*>();
    auto func_pad_concat_v = [&]() {
      /// Pad and concat v
      /// 1. concat pask_v with current_v to total_v.
      /// 2. pad total_v from [B, N_kv, T, H] to [B, N_kv, T_pad, H], where
      /// T_pad is multiples of 128
      __TIC__(AIET_PadConcatV)
      pad_concat_kv(total_v_map, past_v_data.cast<uint16_t>(), v_data_ptr, N_kv,
                    T, T_pad, H, past_present_share_buffer);
      bmm2_inputs[1].sync(XCL_BO_SYNC_BO_TO_DEVICE);
      __TOC__(AIET_PadConcatV)
    };
    auto rst_pad_concat_k = std::async(std::launch::async, func_pad_concat_k);
    auto rst_pad_concat_v = std::async(std::launch::async, func_pad_concat_v);

    /// save present k/v
    /// if past_present_share_buffer, only new k/v should be saved to the share
    /// buffer
    auto func_savekv = [&]() {
      __TIC__(AIET_SaveCurrentKV)
      if (past_present_share_buffer) {
        for (int n = 0; n < N_kv; n++) {
          int past_S = T - 1;
          std::memcpy(present_k_data.cast<uint16_t>() + n * 4096 * H +
                          past_S * H,
                      bf16_k_rope + n * H, H * sizeof(uint16_t));
          /// no need to transpose v here
          /// since the transpose is [B, 1, N, H] to [B, N, 1, H]
          std::memcpy(present_v_data.cast<uint16_t>() + n * 4096 * H +
                          past_S * H,
                      v_data_ptr + n * H, H * sizeof(uint16_t));
        }
      } else {
        for (int n = 0; n < N_kv; n++) {
          int offset_dst = n * T * H;
          int offset_src = n * T_pad * H;
          std::memcpy(present_k_data.cast<uint16_t>() + offset_dst,
                      total_k_map + offset_src, T * H * sizeof(uint16_t));
          std::memcpy(present_v_data.cast<uint16_t>() + offset_dst,
                      total_v_map + offset_src, T * H * sizeof(uint16_t));
        }
      }
      __TOC__(AIET_SaveCurrentKV)
    };
    /// AIE MHA
    /// pad q to [B, N_q, 128, H]
    __TIC__(AIET_PadQKV)
    uint16_t* q_bo_map = bmm1_inputs[0].map<uint16_t*>();
    try_pad_qkv(q_bo_map, bf16_q_rope, N_q, S, 128, H);
    bmm1_inputs[0].sync(XCL_BO_SYNC_BO_TO_DEVICE);
    __TOC__(AIET_PadQKV)
    /// fill attention mask [B, 1, 128, T_pad]
    __TIC__(AIET_FillAttnMaskToken)
    uint16_t* mask_bo_map = softmax_mask.map<uint16_t*>();
    fill_attn_mask_token(mask_bo_map, T, T_pad);
    softmax_mask.sync(XCL_BO_SYNC_BO_TO_DEVICE);
    __TOC__(AIET_FillAttnMaskToken)
    /// alloc output padded for bmm2
    size_t output_padded_size = B * N_q * 128 * H;
    uint16_t* bf16_output_padded = GQO_Allocator.get_buffer_generic<uint16_t>(
        output_padded_size * sizeof(uint16_t),
        GQO_Allocator::BufferType::AIE_OUTPUT_P);
    MY_LOG(2) << "AIE Token phase MHA execute." << std::endl;

    // make sure the K,V is ready before aie_execute_token
    rst_pad_concat_k.wait();
    rst_pad_concat_v.wait();
    // then we could start teh task to save preset KV
    MY_LOG(2) << "AIE Token phase save present K/V." << std::endl;
    auto rst_savekv = std::async(std::launch::async, func_savekv);

    __TIC__(AIET_AieExecToken)
    aie_execute_token(nullptr, N_q, N_kv, 128, T_pad, H);
    __TOC__(AIET_AieExecToken)
    MY_LOG(2) << "AIE Token phase done." << std::endl;
    __TOC__(AIETokenPhase)

    // sync present_kv before exit
    rst_savekv.wait();
    if (!(mha_aie_kernel_info_.is_seq_aie_supported(S))) {
      output_ptr = (uint16_t*)malloc(N_q * S * H * sizeof(uint16_t));
      uint16_t* output_bo = bmm2_outputs[0].map<uint16_t*>();
      memcpy(output_ptr, output_bo, N_q * S * H * sizeof(uint16_t));
      mm_inp = output_ptr;
    }
  } else {
    MY_LOG(2) << "running ORT kernel" << std::endl;
    if (isBf16Model(qkv_data, present_k_data, present_v_data)) {
      __TIC__(ORTCompute)
      __TIC__(ORTKernelAllocInput)
      // inputs: q, k, v, past_key and past_value, relative_postion_bias
      float* float_q_data_conveter = GQO_Allocator.get_buffer_generic<float>(
          q_size * sizeof(float), GQO_Allocator::BufferType::ORT_Q);
      float* float_k_data_conveter = GQO_Allocator.get_buffer_generic<float>(
          kv_size * sizeof(float), GQO_Allocator::BufferType::ORT_K);
      float* float_v_data_conveter = GQO_Allocator.get_buffer_generic<float>(
          kv_size * sizeof(float), GQO_Allocator::BufferType::ORT_V);
      float* float_pask_k_data_conveter =
          GQO_Allocator.get_buffer_generic<float>(
              past_k_size * sizeof(float),
              GQO_Allocator::BufferType::ORT_PAST_K);
      float* float_past_v_data_conveter =
          GQO_Allocator.get_buffer_generic<float>(
              past_v_size * sizeof(float),
              GQO_Allocator::BufferType::ORT_PAST_V);
      __TOC__(ORTKernelAllocInput)

      __TIC__(ORTKernelAllocOutput)
      // outputs
      int output_size = N_q * S * H;
      DataPtrWrapper output_data;
      float* float_output_data_conveter =
          GQO_Allocator.get_buffer_generic<float>(
              output_size * sizeof(float),
              GQO_Allocator::BufferType::ORT_OUTPUT);
      float* float_present_k_data_converter =
          GQO_Allocator.get_buffer_generic<float>(
              present_k_size * sizeof(float),
              GQO_Allocator::BufferType::ORT_PRESENT_K);
      float* float_present_v_data_converter =
          GQO_Allocator.get_buffer_generic<float>(
              present_v_size * sizeof(float),
              GQO_Allocator::BufferType::ORT_PRESENT_V);
      __TOC__(ORTKernelAllocOutput)
      Ort::MemoryInfo info =
          Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
      __TIC__(ORTKernelInputBf16ToFloat32)
      vec_bf16_to_float(float_q_data_conveter, q_data_ptr, q_size);
      vec_bf16_to_float(float_k_data_conveter, k_data_ptr, kv_size);
      vec_bf16_to_float(float_v_data_conveter, v_data_ptr, kv_size);
      if (past_present_share_buffer) {
        for (int n = 0; n < N_kv; n++) {
          int offset = n * past_sequence_length * head_size;
          int convert_size = seq_len_k[0] * head_size;
          vec_bf16_to_float(float_pask_k_data_conveter + offset,
                            past_k_data.cast<uint16_t>() + offset,
                            convert_size);
          vec_bf16_to_float(float_past_v_data_conveter + offset,
                            past_v_data.cast<uint16_t>() + offset,
                            convert_size);
        }
      } else {
        vec_bf16_to_float(float_pask_k_data_conveter,
                          past_k_data.cast<uint16_t>(), past_k_size);
        vec_bf16_to_float(float_past_v_data_conveter,
                          past_v_data.cast<uint16_t>(), past_v_size);
      }
      __TOC__(ORTKernelInputBf16ToFloat32)

      // Create single Ort tensors
      __TIC__(ORTKernelCreateInputOrtTensorValue)
      auto t = ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT;
      std::vector<int64_t> q_shape{B, S, N_q * H};
      std::vector<int64_t> k_shape{B, S, N_kv * H};
      std::vector<int64_t> v_shape{B, S, N_kv * H};
      Ort::Value fp_q = Ort::Value::CreateTensor<float>(
          info, float_q_data_conveter, q_size, q_shape.data(), q_shape.size());
      Ort::Value fp_k = Ort::Value::CreateTensor<float>(
          info, float_k_data_conveter, kv_size, k_shape.data(), k_shape.size());
      Ort::Value fp_v = Ort::Value::CreateTensor<float>(
          info, float_v_data_conveter, kv_size, v_shape.data(), v_shape.size());
      Ort::Value fp_p_k = Ort::Value::CreateTensor<float>(
          info, float_pask_k_data_conveter, past_k_size, past_k_shape.data(),
          past_k_shape.size());
      Ort::Value fp_p_v = Ort::Value::CreateTensor<float>(
          info, float_past_v_data_conveter, past_v_size, past_v_shape.data(),
          past_v_shape.size());
      __TOC__(ORTKernelCreateInputOrtTensorValue)
      const OrtValue* inputs[9] = {fp_q,         fp_k,      fp_v,
                                   fp_p_k,       fp_p_v,    seqlens_k,
                                   total_seqlen, cos_cache, sin_cache};

      __TIC__(ORTKernelCreateOutputOrtTensorValue)
      Ort::Value fp_out = Ort::Value::CreateTensor<float>(
          info, float_output_data_conveter, output_size, output_shape.data(),
          output_shape.size());
      Ort::Value fp_present_key = Ort::Value::CreateTensor<float>(
          info, float_present_k_data_converter, present_k_size,
          present_k_shape.data(), present_k_shape.size());
      Ort::Value fp_present_value = Ort::Value::CreateTensor<float>(
          info, float_present_v_data_converter, present_v_size,
          present_v_shape.data(), present_v_shape.size());
      __TOC__(ORTKernelCreateOutputOrtTensorValue)

      OrtValue* outputs[3] = {fp_out, fp_present_key, fp_present_value};

      __TIC__(ORTBuiltInKernelCompute)
      gqo_built_in.Invoke(context, inputs, 9, outputs, 3);
      __TOC__(ORTBuiltInKernelCompute)

      __TIC__(ORTOutputFloat32ToBf16)
/// convert float32 output to bfloat16
#ifdef _WIN32
      output_ptr1 =
          (uint16_t*)_aligned_malloc(output_size * sizeof(uint16_t), 64);
#else
      output_ptr1 =
          (uint16_t*)aligned_alloc(64, output_size * sizeof(uint16_t));
#endif

      vec_float32_to_bf16(output_ptr1, float_output_data_conveter, output_size);
      mm_inp = output_ptr1;
      /*
            if (ENV_PARAM(USE_AIE_GQO) == 1) {
              int past_seq_len = past_k_shape[2];
              int present_seq_len = present_k_shape[2];
              int num_batch = get_best_parallel_batch(S);
              KVCacheData kv_cache = {past_k_data.cast<uint16_t>(),
                                      float_present_k_data_converter,
                                      present_k_data.cast<uint16_t>(),
                                      past_v_data.cast<uint16_t>(),
                                      float_present_v_data_converter,
                                      present_v_data.cast<uint16_t>(),
                                      past_seq_len * head_size,
                                      present_seq_len * head_size,
                                      head_size};
              ctx.ParallelFor(KV_cache_copy, static_cast<size_t>(kv_num_heads_),
                              num_batch, &kv_cache);
            } else {
      */
      if (past_present_share_buffer) {
        for (int n = 0; n < N_kv; n++) {
          int offset = n * past_sequence_length * head_size;
          int convert_size = total_seq_len * head_size;
          vec_float32_to_bf16(present_k_data.cast<uint16_t>() + offset,
                              float_present_k_data_converter + offset,
                              convert_size);
          vec_float32_to_bf16(present_v_data.cast<uint16_t>() + offset,
                              float_present_v_data_converter + offset,
                              convert_size);
        }
      } else {
        vec_float32_to_bf16(present_k_data.cast<uint16_t>(),
                            float_present_k_data_converter, present_k_size);
        vec_float32_to_bf16(present_v_data.cast<uint16_t>(),
                            float_present_v_data_converter, present_v_size);
      }

      //      }
      __TOC__(ORTOutputFloat32ToBf16)

      __TIC__(ORTMemoryFree)
      __TIC__(ORTMemoryFreeInput)
      // place holder
      __TOC__(ORTMemoryFreeInput)
      __TOC__(ORTMemoryFree)

      __TOC__(ORTCompute)
      MY_LOG(2) << "running ORT kernel^^^^^^^^^^^^^^^^^^^^^" << std::endl;
    } else {
      throw std::runtime_error(
          "Not supported now, only support QKV with bfloat16 as inputs.");
    }
  }

  __TOC__(Compute)
  MY_LOG(2) << "- AMD GQA compute done ...\n";
  MY_LOG(2) << "- AMD Matmul nbits compute start ...\n";
  // MatmulNbits inputs
  auto input_w = ctx.GetInput(7);
  auto input_s = ctx.GetInput(8);
  auto dimension_w = input_w.GetTensorTypeAndShapeInfo().GetShape();
  auto dimension_s = input_s.GetTensorTypeAndShapeInfo().GetShape();
  // Matmul Nbits

  // auto dimensions_out = qkv_shape;
  // dimensions_out[2] = k_n;
  // auto output = ctx.GetOutput(0, dimensions_out);
  // const uint8_t* weights = m_weights.GetTensorData<uint8_t>();
  // const float* scales = m_scales.GetTensorData<float>();

  std::vector<int64_t> out_shape;
  for (unsigned i = 0; i < (qkv_shape.size() - 1); i++)
    out_shape.push_back(qkv_shape[i]);
  out_shape.push_back(n_sizes_[cnt][1]);
  auto output_tensor = ctx.GetOutput(2, {out_shape.begin(), out_shape.end()});
  auto out = output_tensor.GetTensorMutableData<uint16_t>();
  auto output_shape1 = output_tensor.GetTensorTypeAndShapeInfo().GetShape();
  if (mha_aie_kernel_info_.is_seq_aie_supported(S) &&
      ENV_PARAM(USE_AIE_GQO) == 1) {
    matmul_nbits_aie_execute(bmm2_outputs, out, output_shape1, n_sizes_[cnt],
                             grp_sizes_[cnt], cnt);
  } else {
    matmul_nbits_aie_execute1(mm_inp, out, output_shape1, n_sizes_[cnt],
                              grp_sizes_[cnt], cnt);
  }
  MY_LOG(2) << "- AMD GQO compute done ...\n";

  if (output_ptr) {
    free(output_ptr);
    output_ptr = nullptr;
  }
#ifdef _WIN32
  if (output_ptr1) {
    _aligned_free(output_ptr1);
    output_ptr1 = nullptr;
  }
#else
  if (output_ptr1) {
    free(output_ptr1);
    output_ptr1 = nullptr;
  }
#endif
}

} // namespace ort_gqo_custom_op