/*
 *     The Xilinx Vitis AI Vaip in this distribution are provided under the
 * following free and permissive binary-only license, but are not provided in
 * source code form.  While the following free and permissive license is similar
 * to the BSD open source license, it is NOT the BSD open source license nor
 * other OSI-approved open source license.
 *
 *      Copyright (C) 2022 Xilinx, Inc.
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

/*

**/
#pragma once

#include "vaip/vaip.hpp"
#include "vart/runner_ext.hpp"
#include <algorithm>
#include <future>
#include <mutex>
#include <xrt/xrt_bo.h>

#include "../../xrt_shared_context/xrt_shared_context.hpp"
#include "onnxruntime_api.hpp"
#include <ops/op_interface.hpp>

#include <chrono>
// #define PROFILE_DOD

#ifdef PROFILE_DOD
#  define WRAP(msg) msg
#else
#  define WRAP(msg)
#endif

using dd_timer_t = float;
#define GET_TIMESTAMP() std::chrono::steady_clock::now()
#define GET_INTERVAL(t1, t2)                                                   \
  std::chrono::duration<dd_timer_t, std::micro>(t2 - t1).count()

struct DDTimer {
  dd_timer_t compute_time{0};
  dd_timer_t dod_time{0};
  dd_timer_t pre_dod_time{0};
  dd_timer_t post_dod_time{0};
  std::vector<dd_timer_t> pad_time;
  std::vector<dd_timer_t> depad_time;
  std::vector<dd_timer_t> nchw2nhwc_time;
  std::vector<dd_timer_t> nhwc2nchw_time;
  std::vector<dd_timer_t> c4hw2hwc4_time;
  std::vector<dd_timer_t> data_conv_time;
  std::vector<dd_timer_t> iconv_prep_time;

  void reset() {
    compute_time = 0;
    dod_time = 0;
    pre_dod_time = 0;
    post_dod_time = 0;
    pad_time.clear();
    depad_time.clear();
    nchw2nhwc_time.clear();
    nhwc2nchw_time.clear();
    c4hw2hwc4_time.clear();
    data_conv_time.clear();
    iconv_prep_time.clear();
  }
};

namespace vaip_dod_custom_op {

static enum DTypeConvert { TO_BF16 = 1, FROM_BF16 = 2, AS_IS = 3 };

using namespace vaip_core;
class MyCustomOp : public CustomOpImp {
public:
  MyCustomOp(std::shared_ptr<const PassContext> context,
             const std::shared_ptr<MetaDefProto>& meta_def,
             onnxruntime::Model* model);

  virtual ~MyCustomOp();

  template <typename DType>
  std::vector<DType> string_to_values(std::string str_values);

  template <typename SrcDType, typename DstDType>
  void pad(const SrcDType* src, std::vector<int64_t>& src_shape, DstDType* dst,
           std::vector<size_t>& dst_shape, int dim, DTypeConvert flag,
           float scale, float zp) const;

  template <typename SrcDType, typename DstDType>
  void depad(SrcDType* src, std::vector<size_t>& src_shape, DstDType* dst,
             std::vector<int64_t>& dst_shape, int dim, DTypeConvert flag,
             float scale, float zp) const;

private:
  virtual void Compute(const OrtApi* api,
                       OrtKernelContext* context) const override final;
  virtual void inputs_preprocess(OrtKernelContext* context,
                                 std::vector<Tensor>& in_tensors) const;
  virtual std::vector<Ort::UnownedValue>
  outputs_preprocess(OrtKernelContext* context,
                     std::vector<Tensor>& out_tensors) const;
  virtual void outputs_postprocess(std::vector<Ort::UnownedValue> ort_outputs,
                                   std::vector<Tensor>& out_tensors) const;

  std::shared_ptr<void> runner_;
  std::vector<Tensor> in_tensors_;
  std::vector<Tensor> out_tensors_;
  size_t num_inputs_, num_outputs_;
  std::vector<std::vector<float>> in_scale_zps_;
  std::vector<std::vector<float>> out_scale_zps_;
  std::vector<std::vector<int64_t>> orig_output_shapes_;
  std::vector<std::string> in_dtypes_;
  std::vector<std::string> out_dtypes_;
  mutable std::vector<std::vector<uint16_t>> in_buffer_i16_;
  mutable std::vector<std::vector<uint8_t>> in_buffer_i8_;
  mutable std::vector<std::vector<uint16_t>> out_buffer_i16_;
  mutable std::vector<std::vector<uint8_t>> out_buffer_i8_;
  mutable std::vector<uint16_t> first_layer_data;
  std::vector<size_t> dod_in_index_;
  std::vector<size_t> dod_out_index_;
  std::string meta_json_;

  mutable std::vector<DDTimer> timestamps_;
  mutable DDTimer ddtimer_;
  mutable std::mutex execute_mutex_;
  std::shared_ptr<vaip::Context> shared_ctx_;
  mutable bool share_context_;
  mutable std::string perf_pref_run_cached_;
  mutable std::string perf_pref_session_cached_;
  std::shared_ptr<xrt::hw_context> hw_ctx_ptr_;
  // for XRT driver backward compatibility in non share context model
  bool support_eff_mode_;
};

} // namespace vaip_dod_custom_op