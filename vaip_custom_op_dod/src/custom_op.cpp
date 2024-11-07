/*
 *     The Xilinx Vitis AI Vaip in this distribution are provided under the
 * following free and permissive binary-only license, but are not provided in
 * source code form.  While the following free and permissive license is similar
 * to the BSD open source license, it is NOT the BSD open source license nor
 * other OSI-approved open source license.
 *
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

// must include cxx_api.hpp before custom_op.hpp otherise
// VAIP_ORT_API_VERSION is not defined we cannot use OrtAPI here.
#include <algorithm>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <vcruntime.h>

#include "../../xrt_shared_context/xrt_shared_context.hpp"

#include "vitis/ai/env_config.hpp"
#include "vitis/ai/profiling.hpp"

#include <xir/graph/graph.hpp>

#include "custom_op.hpp"
#pragma once

#include "../../vaip/src/qos_updater.hpp"
#include "utils.hpp"
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <numeric>
#include <op_fuser/fusion_rt.hpp>
#include <ops/ops_common/iconv_matrix.hpp>
#include <sstream>
#include <utils/meta_utils.hpp>

namespace fs = std::filesystem;
// using namespace ryzenai;

DEF_ENV_PARAM(DEBUG_DOD_CUSTOM_OP, "0");
#define LOG_THIS(n) LOG_IF(INFO, ENV_PARAM(DEBUG_DOD_CUSTOM_OP) >= n)

namespace vaip_dod_custom_op {

template <typename SrcDType, typename DstDType>
void MyCustomOp::pad(const SrcDType* src, std::vector<int64_t>& src_shape,
                     DstDType* dst, std::vector<size_t>& dst_shape, int dim,
                     DTypeConvert flag, float scale, float zp) const {
  WRAP(auto t1 = GET_TIMESTAMP();)
  int elems = 1;
  for (int i = dim + 1; i < src_shape.size(); ++i)
    elems *= src_shape[i];

  int iters = 1;
  for (int i = 0; i < dim; ++i)
    iters *= src_shape[i];

  int dimx = src_shape[dim];
  int dimy = dst_shape[dim];

  int s_off = dimx * elems;
  int d_off = dimy * elems;

  if (flag == DTypeConvert::AS_IS) {
    for (auto f = 0; f < iters; ++f) {
      std::memcpy(dst + (f * d_off), src + (f * s_off),
                  s_off * sizeof(DstDType));
    }
  } else if (flag == DTypeConvert::TO_BF16) {
    for (auto f = 0; f < iters; ++f) {
      for (auto idx = 0; idx < s_off; ++idx) {
        float value = (static_cast<float>(src[f * s_off + idx]) - zp) * scale;
        dst[f * d_off + idx] = float_to_bfloat16_1(value);
      }
    }
    // Assuming attention mask 4D only 1x1x1x77  would be padded to 1x1x1x128
    // with bfloat16(-zp*scale).
    if (dst_shape.size() == 4) {
      for (int i = 77; i < 128; i++) {
        float value = (float)((-zp) * (scale));
        dst[i] = float_to_bfloat16_1(value);
      }
    }
  } else {
    LOG(FATAL) << "- Incorrect flag for Padding input, only to_bf16 conversion "
                  "is possible.";
  }
  WRAP(auto t2 = GET_TIMESTAMP();)
  WRAP(ddtimer_.pad_time.push_back(GET_INTERVAL(t1, t2));)
}

template <typename SrcDType, typename DstDType>
void MyCustomOp::depad(SrcDType* src, std::vector<size_t>& src_shape,
                       DstDType* dst, std::vector<int64_t>& dst_shape, int dim,
                       DTypeConvert flag, float scale, float zp) const {
  WRAP(auto t1 = GET_TIMESTAMP();)

  int elems = 1;
  for (int i = dim + 1; i < src_shape.size(); ++i)
    elems *= src_shape[i];

  int iters = 1;
  for (int i = 0; i < dim; ++i)
    iters *= src_shape[i];

  int dimx = src_shape[dim];
  int dimy = dst_shape[dim];

  int s_off = dimx * elems;
  int d_off = dimy * elems;

  if (flag == DTypeConvert::AS_IS) {
    for (auto f = 0; f < iters; ++f) {
      std::memcpy(dst + (f * d_off), src + (f * s_off),
                  d_off * sizeof(DstDType));
    }
  } else if (flag == DTypeConvert::FROM_BF16) {
    for (auto f = 0; f < iters; ++f) {
      for (auto idx = 0; idx < d_off; ++idx) {
        float value = bfloat_to_float(src[f * s_off + idx]);
        dst[f * d_off + idx] = static_cast<DstDType>(
            std::clamp(std::roundf(value / scale) + zp, 0.0f, 65535.0f));
      }
    }
  } else {
    LOG(FATAL) << "- Incorrect flag for De-padding output, only from_bf16 "
                  "conversion is possible.";
  }
  WRAP(auto t2 = GET_TIMESTAMP();)
  WRAP(ddtimer_.depad_time.push_back(GET_INTERVAL(t1, t2));)
}

MyCustomOp::MyCustomOp(std::shared_ptr<const PassContext> context,
                       const std::shared_ptr<MetaDefProto>& meta_def,
                       onnxruntime::Model* model)
    : CustomOpImp(context, meta_def, model) { //,
  {
    auto mutable_context = const_cast<PassContext*>(context.get());
    auto custom_op_constructor =
        mutable_context->measure("dd_custom_op_constructor");
    auto all_provider_options = context->get_config_proto().provider_options();

    // "dod_txn_root" should be removed
    std::string dod_txn_root = "dynamic_dispatch_vaiep";
    auto xclbin_config_path = context->xclbin_path_to_cache_files(
        std::filesystem::path(all_provider_options["xclbin"]));
    auto xclbin_config_name = xclbin_config_path.filename();
    if (xclbin_config_path.empty()) {
      LOG(FATAL) << "- Error: Unable to find xclbin. Make sure to set xclbin "
                    "path in provider options.";
    }
    bool found = context->has_cache_file(xclbin_config_name.string());

    if (!found) {
      std::string err_msg = std::string{"The xclbin : "} +
                            xclbin_config_path.string() +
                            std::string{" does not exist."};
      LOG(FATAL) << err_msg;
    }
    // Get meta data for dod
    std::string meta_json = meta_def->generic_param().at("meta_json");
    std::string meta_state = meta_def->generic_param().at("meta_state");
    auto dd_cache_dir = context->get_log_dir();
    auto meta_json_path = dd_cache_dir / meta_json;
    auto meta_state_path = dd_cache_dir / meta_state;
    OpsFusion::Metadata meta;
    auto file = context->read_file_c8(meta_json).value();
    std::string str(file.data(), file.size());
    meta = OpsFusion::load_meta_json_str(str);
    meta_json_ = meta_json_path.string();

    in_tensors_ = OpsFusion::MetaUtils::get_input_tensors(meta);
    num_inputs_ = OpsFusion::MetaUtils::get_num_inputs(meta);

    auto dod_input_names = meta.fused_tensors.at("in").packed_tensors;
    auto ort_input_names = meta_def->inputs();

    for (int i = 0; i < dod_input_names.size(); i++) {
      for (int j = 0; j < ort_input_names.size(); j++) {
        if (ort_input_names[j] == dod_input_names[i]) {
          dod_in_index_.push_back(j);
          break;
        }
      }
    }

    for (size_t i = 0; i < num_inputs_; i++) {
      std::vector<size_t> shape = in_tensors_[i].shape;
      size_t size = std::accumulate(shape.begin(), shape.end(), size_t{1},
                                    std::multiplies{});
      in_buffer_i16_.emplace_back(size, 0);
      in_buffer_i8_.emplace_back(size, 0);
    }

    out_tensors_ = OpsFusion::MetaUtils::get_output_tensors(meta);
    num_outputs_ = OpsFusion::MetaUtils::get_num_outputs(meta);

    auto dod_output_names = meta.fused_tensors.at("out").packed_tensors;
    auto ort_output_names = meta_def->outputs();

    for (int i = 0; i < dod_output_names.size(); i++) {
      for (int j = 0; j < ort_output_names.size(); j++) {
        if (ort_output_names[j] == dod_output_names[i]) {
          dod_out_index_.push_back(j);
          break;
        }
      }
    }
    try {
      // get q params for input/output
      std::string input_q_params =
          meta_def->generic_param().at("input_q_params");
      std::string output_q_params =
          meta_def->generic_param().at("output_q_params");
      std::string original_output_shapes =
          meta_def->generic_param().at("original_output_shapes");

      LOG_THIS(1) << "input_q_params: " << input_q_params;
      LOG_THIS(1) << "output_q_params: " << output_q_params;
      LOG_THIS(1) << "original_output_shapes: " << original_output_shapes;

      if (meta_def->generic_param().contains("in_dtypes")) {
        auto in_dtypes_str = meta_def->generic_param().at("in_dtypes");
        in_dtypes_ = split_string(in_dtypes_str);
        if (in_dtypes_.size() != in_tensors_.size()) {
          LOG(FATAL) << "Mismatch in number of input tensors and corresponding "
                        "data types";
        }
        for (int i = 0; i < in_tensors_.size(); ++i) {
          in_tensors_.at(i).dtype = in_dtypes_.at(i);
        }
        LOG_THIS(1) << "input dtypes: " << in_dtypes_str;
      }

      if (meta_def->generic_param().contains("out_dtypes")) {
        auto out_dtypes_str = meta_def->generic_param().at("out_dtypes");
        out_dtypes_ = split_string(out_dtypes_str);
        if (out_dtypes_.size() != out_tensors_.size()) {
          LOG(FATAL)
              << "Mismatch in number of output tensors and corresponding "
                 "data types";
        }
        for (int i = 0; i < out_tensors_.size(); ++i) {
          out_tensors_.at(i).dtype = out_dtypes_.at(i);
        }
        LOG_THIS(1) << "output dtypes: " << out_dtypes_str;
      }

      in_scale_zps_ = cs_string_to_nested_list<float>(input_q_params);
      out_scale_zps_ = cs_string_to_nested_list<float>(output_q_params);
      orig_output_shapes_ =
          cs_string_to_nested_list<int64_t>(original_output_shapes);

      if (!in_scale_zps_.size())
        LOG_THIS(1) << "Input Scale/Zp size: " << in_scale_zps_.size();
      if (!out_scale_zps_.size())
        LOG_THIS(1) << "Output Scale/Zp size: " << out_scale_zps_.size();
      if (!orig_output_shapes_.size())
        LOG(FATAL) << "Original output shape size: "
                   << orig_output_shapes_.size();

      LOG_THIS(1) << "DOD Kernel Init .. start";
      // use shared context if enabled
      share_context_ = false;
      if (all_provider_options.contains(vaip::Context::CTX_SHARE_OPTION_KEY)) {
        try {
          share_context_ = std::stoi(
              all_provider_options.at(vaip::Context::CTX_SHARE_OPTION_KEY));
        } catch (...) {
          LOG_THIS(1) << "failed to convert provider option \""
                      << vaip::Context::CTX_SHARE_OPTION_KEY << "\" value \""
                      << all_provider_options.at(
                             vaip::Context::CTX_SHARE_OPTION_KEY)
                      << "\" to int, disable context sharing.";
        }
      }
      // The qos_map will be used by update_qos, regardless of whether
      // share_context_ is enabled or not.
      std::map<std::string, std::uint32_t> qos_map;
      std::uint32_t qos_priority;
      if (meta_def->generic_param().contains("qos_priority")) {
        qos_priority = static_cast<uint32_t>(
            std::stoul(meta_def->generic_param().at("qos_priority")));
        qos_map["perf_pref"] = 1;
        LOG_THIS(1) << "Setting custom QoS Priority to DPM0";
      }
      support_eff_mode_ = true;
      if (share_context_) {
        LOG_THIS(1) << "DOD using shared context";
        auto device_id = 0;
        auto context_id = 0;
        // share context already handled in-memory xclbin issue
        shared_ctx_ = vaip::Context::create_shared_context(
            *context, device_id, context_id, xclbin_config_path.string(),
            qos_map);
        shared_ctx_->update_qos(qos_map);
        runner_ = std::make_shared<OpsFusion::FusionRuntime>(
            &(shared_ctx_->xrt_hw_context()));

        // use XRTUpdateQosImpl object to update efficient mode directly through
        // xrt::hw_context
        auto qos_updater = std::make_shared<vaip_core::XRTUpdateQosImpl>(
            &shared_ctx_->xrt_hw_context());
        context->add_QosUpdater(qos_updater);
      } else {
        LOG_THIS(1) << "DOD NOT using shared context";
        std::unique_ptr<xrt::xclbin> xrt_xclbin_ptr;

        auto xclbin_context = context->read_xclbin(xclbin_config_path.string());
        if (xclbin_context.has_value()) {
          auto xclbin_context2 = std::vector<char>(
              xclbin_context.value().begin(), xclbin_context.value().end());
          xrt_xclbin_ptr = std::make_unique<xrt::xclbin>(xclbin_context2);
        } else {
          xrt_xclbin_ptr =
              std::make_unique<xrt::xclbin>(xclbin_config_path.string());
        }
        auto xrt_device_ptr = std::make_unique<xrt::device>(0);
        xrt_device_ptr->register_xclbin(*xrt_xclbin_ptr);

        hw_ctx_ptr_ = std::make_shared<xrt::hw_context>(
            *xrt_device_ptr, xrt_xclbin_ptr->get_uuid(), qos_map);
        runner_ = std::make_shared<OpsFusion::FusionRuntime>(hw_ctx_ptr_.get());

        // use XRTUpdateQosImpl object to update efficient mode directly through
        // xrt::hw_context
        auto qos_updater =
            std::make_shared<vaip_core::XRTUpdateQosImpl>(hw_ctx_ptr_.get());
        context->add_QosUpdater(qos_updater);
      }

      OpsFusion::FusionRuntime* ptr = (OpsFusion::FusionRuntime*)runner_.get();
      LOG_THIS(1) << "DOD Actual Init .. done";

      OpsFusion::DDConfig cfg;
      if (meta_def->generic_param().contains("model_category"))
        cfg.model_name = meta_def->generic_param().at("model_category");
      cfg.cache_dir = dd_cache_dir.string();
      auto fusion_runtime_load_state =
          mutable_context->measure("fusion_runtime_load_state");
      load_function func = nullptr;
      func = [&context](const std::string& path) -> FILE* {
        auto filename = std::filesystem::path(path).filename().string();
        return context->open_file(filename);
      };
      ptr->load_state(meta_state_path.string(), func);
      fusion_runtime_load_state = nullptr;
      auto fusion_runtime_init =
          mutable_context->measure("fusion_runtime_init");
      ptr->init(meta, dod_txn_root, cfg);
      fusion_runtime_init = nullptr;
      LOG_THIS(1) << "DOD Kernel Init .. done";

    } catch (std::exception& e) {
      LOG(FATAL) << "- Error: Failed to create DoD Instance: " << e.what();
    }
  }
  context->save_context_json();
}

MyCustomOp::~MyCustomOp() {
#ifdef PROFILE_DOD
  static DDFileLogger ofs("dd_custom_op_perf.csv");
  for (const auto& timer : timestamps_) {
    ofs << timer.compute_time << "," << timer.pre_dod_time << ","
        << timer.dod_time << "," << timer.post_dod_time << ","
        << sum_of(timer.pad_time) << "," << sum_of(timer.depad_time) << ","
        << sum_of(timer.nchw2nhwc_time) << "," << sum_of(timer.nhwc2nchw_time)
        << "," << sum_of(timer.c4hw2hwc4_time) << ","
        << sum_of(timer.data_conv_time) << "," << sum_of(timer.iconv_prep_time)
        << "," << meta_json_ << std::endl;
  }
#endif
}

void MyCustomOp::Compute(const OrtApi* api, OrtKernelContext* context) const {
  std::lock_guard<std::mutex> guard(execute_mutex_);
  WRAP(ddtimer_.reset();)
  WRAP(auto compute_begin = GET_TIMESTAMP());

  if (Ort::Global<void>::api_ == nullptr) {
    Ort::Global<void>::api_ = api;
  }
  Ort::KernelContext ctx(context);

  std::vector<Tensor> in_tensors = in_tensors_;
  std::vector<Tensor> out_tensors = out_tensors_;

  inputs_preprocess(context, in_tensors);

  auto ort_outputs = outputs_preprocess(context, out_tensors);

  // Invoke kernel
  OpsFusion::FusionRuntime* ptr = (OpsFusion::FusionRuntime*)runner_.get();

  auto dod_exec_start = GET_TIMESTAMP();
  ptr->execute(in_tensors, out_tensors);
  auto dod_exec_end = GET_TIMESTAMP();

  outputs_postprocess(ort_outputs, out_tensors);

  WRAP(auto compute_end = GET_TIMESTAMP();)
  WRAP(ddtimer_.compute_time = GET_INTERVAL(compute_begin, compute_end);)
  WRAP(ddtimer_.dod_time = GET_INTERVAL(dod_exec_start, dod_exec_end);)
  WRAP(ddtimer_.pre_dod_time = GET_INTERVAL(compute_begin, dod_exec_start);)
  WRAP(ddtimer_.post_dod_time = GET_INTERVAL(dod_exec_end, compute_end);)
  WRAP(timestamps_.push_back(std::move(ddtimer_));)
}

void MyCustomOp::inputs_preprocess(OrtKernelContext* context,
                                   std::vector<Tensor>& in_tensors) const {
  Ort::KernelContext ctx(context);
  auto num_inputs = ctx.GetInputCount();

  if (num_inputs != num_inputs_)
    LOG(FATAL) << "Mismatch in number of inputs between subgraph and ORT";

  // Inputs
  for (size_t i = 0; i < num_inputs; i++) {
    auto input_tensor = ctx.GetInput(dod_in_index_[i]);
    auto input_type = input_tensor.GetTensorTypeAndShapeInfo().GetElementType();
    auto input_shape = input_tensor.GetTensorTypeAndShapeInfo().GetShape();
    size_t elems = std::accumulate(input_shape.begin(), input_shape.end(),
                                   (size_t)1, std::multiplies<int64_t>());

    // Enable pad if required
    bool en_pad = false;
    int pad_dim = -1;
    for (auto dim = 0; dim < input_shape.size(); ++dim) {
      if (input_shape[dim] != in_tensors[i].shape[dim]) {
        en_pad = true;
        pad_dim = dim;
        break;
      }
    }

    if (in_tensors[i].dtype == "bfloat16" ||
        (input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)) {
      auto scale = in_scale_zps_[i][0];
      auto zp = in_scale_zps_[i][1];

      if (input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16) {
        auto input_data = input_tensor.GetTensorData<uint16_t>();
        if (en_pad) {
          pad<uint16_t, uint16_t>(input_data, input_shape,
                                  in_buffer_i16_[i].data(), in_tensors[i].shape,
                                  pad_dim, DTypeConvert::TO_BF16, scale, zp);
        } else {
          WRAP(auto t1 = GET_TIMESTAMP();)
          for (auto cnt = 0; cnt < elems; cnt++) {
            float value = (static_cast<float>(input_data[cnt]) - zp) * scale;
            in_buffer_i16_[i][cnt] = float_to_bfloat16_1(value);
          }
          WRAP(auto t2 = GET_TIMESTAMP();)
          WRAP(ddtimer_.data_conv_time.push_back(GET_INTERVAL(t1, t2));)
        }
        in_tensors[i].data = (void*)(in_buffer_i16_[i].data());
      } else if (input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8) {
        auto input_data = input_tensor.GetTensorData<uint8_t>();
        if (en_pad) {
          pad<uint8_t, uint16_t>(input_data, input_shape,
                                 in_buffer_i16_[i].data(), in_tensors[i].shape,
                                 pad_dim, DTypeConvert::TO_BF16, scale, zp);
        } else {
          WRAP(auto t1 = GET_TIMESTAMP();)
          for (auto cnt = 0; cnt < elems; cnt++) {
            float value = (static_cast<float>(input_data[cnt]) - zp) * scale;
            in_buffer_i16_[i][cnt] = float_to_bfloat16_1(value);
          }
          WRAP(auto t2 = GET_TIMESTAMP();)
          WRAP(ddtimer_.data_conv_time.push_back(GET_INTERVAL(t1, t2));)
        }
      }
      // Provide data pointers to DOD
      if (input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16 ||
          input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8) {
        in_tensors[i].data = (void*)(in_buffer_i16_[i].data());
      } else if (input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT &&
                 in_tensors[i].dtype == "uint16") {
        // Input_3 mzdk5
        auto input_data = input_tensor.GetTensorData<float>();
        for (auto cnt = 0; cnt < elems; cnt++) {
          in_buffer_i16_[i][cnt] = static_cast<uint16_t>(std::clamp(
              std::roundf(input_data[cnt] / scale) + zp, 0.0f, 65535.0f));
        }
        in_tensors[i].data = (void*)(in_buffer_i16_[i].data());
      } else if (input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT &&
                 in_tensors[i].dtype == "bfloat16") {
        auto input_data = input_tensor.GetTensorData<float>();
        for (auto cnt = 0; cnt < elems; cnt++) {
          float value = (static_cast<float>(input_data[cnt]));
          in_buffer_i16_[i][cnt] = float_to_bfloat16_1(value);
        }
        in_tensors[i].data = (void*)(in_buffer_i16_[i].data());
      } else {
        in_tensors[i].data = (void*)(input_tensor.GetTensorData<void>());
      }
    } else {
      // No conversion required
      if (C4HW_to_FORMAT_ICONV_IFM_required(input_shape, in_tensors[i].shape)) {
        auto input_data = input_tensor.GetTensorData<uint16_t>();
        first_layer_data.reserve(16 * 64 * 64);

        iconv_matrix::ActTensor<uint16_t> X(16, 64, 64,
                                            first_layer_data.data());
        bool is_nhwc_input = input_shape[1] == input_shape[2] ? 1 : 0;
        iconv_matrix::format_conv_ifm<uint16_t>(input_data, 30840, 4, X,
                                                is_nhwc_input);
        in_tensors[i].data = first_layer_data.data();
        in_tensors[i].shape = {1, 16, 64, 64};

      } else if (NCHW_to_NHWC_conversion_required(input_shape,
                                                  in_tensors[i].shape)) {
        auto input_data = input_tensor.GetTensorData<uint16_t>();
        convert_NCHW_to_NHWC<uint16_t>(
            input_data, in_buffer_i16_[i].data(), input_shape[0],
            input_shape[1], input_shape[2], input_shape[3], ddtimer_);

        in_tensors[i].data = (void*)(in_buffer_i16_[i].data());

      } else if (C3HW_to_FOLD_ICONV_IFM_required(input_shape,
                                                 in_tensors[i].shape)) {
        int Sx_no_fold = 4;
        int pad_no_fold = 3;
        int fold_factor = 2;
        int Ci_gran = 4;
        int Xi_gran = 4;
        int CI = input_shape[1];
        int YI = input_shape[2];
        int XI = input_shape[3];
        int XO = 56;
        auto input_data = input_tensor.GetTensorData<uint16_t>();
        first_layer_data.reserve(8 * 230 * 116);

        WRAP(auto t1 = GET_TIMESTAMP();)
        iconv_matrix::ActTensor<uint16_t> X(8, 230, 116,
                                            first_layer_data.data());
        iconv_matrix::fold_conv_ifm<uint16_t>(input_data, 29172, CI, YI, XI, XO,
                                              7, Sx_no_fold, pad_no_fold,
                                              fold_factor, Ci_gran, Xi_gran, X);
        WRAP(auto t2 = GET_TIMESTAMP();)
        WRAP(ddtimer_.iconv_prep_time.push_back(GET_INTERVAL(t1, t2));)

        in_tensors[i].data = first_layer_data.data(); // working
        in_tensors[i].shape = {1, 8, 230, 116};
      } else if (en_pad) {
        if (input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16) {
          auto input_data = input_tensor.GetTensorData<uint16_t>();
          pad<uint16_t, uint16_t>(input_data, input_shape,
                                  in_buffer_i16_[i].data(), in_tensors[i].shape,
                                  pad_dim, DTypeConvert::AS_IS, 0, 0);
          if (C3HW_to_HWC4_conversion_required(input_shape,
                                               in_tensors[i].shape)) {
            convert_C4HW_to_HWC4(in_buffer_i16_[i], in_buffer_i16_[i],
                                 input_shape[2], input_shape[3],
                                 uint16_t{29172}, ddtimer_);
          } else if (NC3HW_to_HNWC4_conversion_required(input_shape,
                                                        in_tensors[i].shape)) {
            convert_C4HW_to_HWC4(in_buffer_i16_[i], in_buffer_i16_[i],
                                 input_shape[2], input_shape[3], uint16_t{0},
                                 ddtimer_);
          } else if (NC3HW_to_HNWC8_conversion_required(input_shape,
                                                        in_tensors[i].shape)) {
            convert_NC3HW_to_HNWC8(in_buffer_i16_[i], in_buffer_i16_[i],
                                   input_shape[2], input_shape[3], uint16_t{0});
          }
          in_tensors[i].data = (void*)(in_buffer_i16_[i].data());
        } else if (input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8) {
          auto input_data = input_tensor.GetTensorData<uint8_t>();
          pad<uint8_t, uint8_t>(input_data, input_shape,
                                in_buffer_i8_[i].data(), in_tensors[i].shape,
                                pad_dim, DTypeConvert::AS_IS, 0, 0);
          if (C3HW_to_HWC4_conversion_required(input_shape,
                                               in_tensors[i].shape)) {
            convert_C4HW_to_HWC4(in_buffer_i8_[i], in_buffer_i8_[i],
                                 input_shape[2], input_shape[3], uint8_t{0},
                                 ddtimer_);
          }
          in_tensors[i].data = (void*)(in_buffer_i8_[i].data());
        }
      } else {
        auto input_data = input_tensor.GetTensorData<uint16_t>();
        in_tensors[i].data = (void*)(input_data);
      }
    }
  }
}

std::vector<Ort::UnownedValue>
MyCustomOp::outputs_preprocess(OrtKernelContext* context,
                               std::vector<Tensor>& out_tensors) const {
  Ort::KernelContext ctx(context);
  auto num_outputs = ctx.GetOutputCount();

  if (num_outputs != num_outputs_)
    LOG(FATAL) << "Mismatch in number of outputs between subgraph and ORT";

  // Outputs
  std::vector<Ort::UnownedValue> ort_outputs;
  ort_outputs.reserve(num_outputs);

  out_buffer_i16_.clear();
  out_buffer_i8_.clear();
  for (size_t i = 0; i < num_outputs_; i++) {
    // Create ORT output tensor based on original shape
    size_t ort_out_index = dod_out_index_[i];
    auto output_tensor = ctx.GetOutput(
        ort_out_index, {orig_output_shapes_[ort_out_index].begin(),
                        orig_output_shapes_[ort_out_index].end()});
    // Output tensor data pointer and shape
    auto output_data = output_tensor.GetTensorMutableData<void>();
    auto output_shape = output_tensor.GetTensorTypeAndShapeInfo().GetShape();
    auto output_type =
        output_tensor.GetTensorTypeAndShapeInfo().GetElementType();

    // Enable depad if required
    bool en_depad = false;
    int depad_dim = -1;
    for (auto dim = 0; dim < output_shape.size(); ++dim) {
      if (output_shape[dim] != out_tensors[i].shape[dim]) {
        en_depad = true;
        depad_dim = dim;
        break;
      }
    }

    size_t size = std::accumulate(out_tensors_[i].shape.begin(),
                                  out_tensors_[i].shape.end(), size_t{1},
                                  std::multiplies{});
    // Convert to bfloat16, if kernel expects it
    if (out_tensors[i].dtype == "bfloat16") {
      // no need to check for depad as it is already full sized buffer
      out_buffer_i16_.emplace_back(size, 0);
      out_tensors[i].data = (void*)(out_buffer_i16_.back().data());
    } else {
      // if type is either uint16_t or uint8_t
      if (output_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16) {
        if (NCHW_to_NHWC_conversion_required(output_shape,
                                             out_tensors[i].shape)) {
          out_buffer_i16_.emplace_back(size, 0);
          out_tensors[i].data = (void*)(out_buffer_i16_.back().data());

        } else if (en_depad) {
          out_buffer_i16_.emplace_back(size, 0);
          out_tensors[i].data = (void*)(out_buffer_i16_.back().data());
        } else {
          out_tensors[i].data = (void*)output_data;
          out_buffer_i16_.emplace_back(0, 0);
        }
      } else if (output_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8) {
        if (en_depad) {
          out_buffer_i8_.emplace_back(size, 0);
          out_tensors[i].data = (void*)(out_buffer_i8_.back().data());
        } else {
          out_buffer_i8_.emplace_back(0, 0);
          out_tensors[i].data = (void*)output_data;
        }
      }
    }
    // Save ORT output tensor
    ort_outputs.push_back(output_tensor);
  }

  return ort_outputs;
}

void MyCustomOp::outputs_postprocess(std::vector<Ort::UnownedValue> ort_outputs,
                                     std::vector<Tensor>& out_tensors) const {

  for (size_t i = 0; i < num_outputs_; i++) {
    auto output_shape =
        ort_outputs[dod_out_index_[i]].GetTensorTypeAndShapeInfo().GetShape();

    size_t elems = std::accumulate(output_shape.begin(), output_shape.end(),
                                   (size_t)1, std::multiplies<int64_t>());
    // ORT tensor type
    auto tensor_type = ort_outputs[dod_out_index_[i]]
                           .GetTensorTypeAndShapeInfo()
                           .GetElementType();
    bool en_depad = false;
    int depad_dim = -1;
    for (auto dim = 0; dim < output_shape.size(); ++dim) {
      if (output_shape[dim] != out_tensors[i].shape[dim]) {
        en_depad = true;
        depad_dim = dim;
        break;
      }
    }

    if (out_tensors[i].dtype == "bfloat16") {
      // Scale / zp for output
      float scale, zp;
      if (!out_scale_zps_[i].empty()) {
        scale = out_scale_zps_[i][0];
        zp = out_scale_zps_[i][1];
      }
      if (tensor_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        auto output_data =
            ort_outputs[dod_out_index_[i]].GetTensorMutableData<float>();
        for (auto cnt = 0; cnt < elems; cnt++) {
          float value = bfloat_to_float(out_buffer_i16_[i][cnt]);
          output_data[cnt] = value;
        }
      } else if (tensor_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16) {
        auto output_data =
            ort_outputs[dod_out_index_[i]].GetTensorMutableData<uint16_t>();
        if (en_depad) {
          depad<uint16_t, uint16_t>(
              out_buffer_i16_[i].data(), out_tensors[i].shape, output_data,
              output_shape, depad_dim, DTypeConvert::FROM_BF16, scale, zp);
        } else {
          for (auto cnt = 0; cnt < elems; cnt++) {
            float value = bfloat_to_float(out_buffer_i16_[i][cnt]);
            output_data[cnt] = static_cast<uint16_t>(
                std::clamp(std::roundf(value / scale) + zp, 0.0f, 65535.0f));
          }
        }
      } else if (tensor_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8) {
        auto output_data =
            ort_outputs[dod_out_index_[i]].GetTensorMutableData<uint8_t>();
        if (en_depad) {
          depad<uint16_t, uint8_t>(
              out_buffer_i16_[i].data(), out_tensors[i].shape, output_data,
              output_shape, depad_dim, DTypeConvert::FROM_BF16, scale, zp);
        } else {
          for (auto cnt = 0; cnt < elems; cnt++) {
            float value = bfloat_to_float(out_buffer_i16_[i][cnt]);
            output_data[cnt] = static_cast<uint8_t>(
                std::clamp(std::roundf(value / scale) + zp, 0.0f, 255.0f));
          }
        }
      }
    } else {
      // De-pad
      // No conversion required
      if (NCHW_to_NHWC_conversion_required(output_shape,
                                           out_tensors[i].shape)) {
        // Channel Transpose (output)
        auto output_data =
            ort_outputs[dod_out_index_[i]].GetTensorMutableData<uint16_t>();

        convert_NHWC_to_NCHW<uint16_t>(
            out_buffer_i16_[i].data(), output_data, out_tensors[i].shape[0],
            out_tensors[i].shape[1], out_tensors[i].shape[2],
            out_tensors[i].shape[3], ddtimer_);
      } else if (NCHW_to_HNWC_conversion_required(output_shape,
                                                  out_tensors[i].shape)) {
        auto output_data =
            ort_outputs[dod_out_index_[i]].GetTensorMutableData<uint16_t>();

        convert_HNWC_to_NCHW<uint16_t>(
            out_buffer_i16_[i].data(), output_data, out_tensors[i].shape[1],
            out_tensors[i].shape[0], out_tensors[i].shape[2],
            out_tensors[i].shape[3]);

      } else if (en_depad) {
        if (tensor_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16) {
          auto output_data =
              ort_outputs[dod_out_index_[i]].GetTensorMutableData<uint16_t>();
          if (out_buffer_i16_[i].data() != 0) {
            depad<uint16_t, uint16_t>(
                out_buffer_i16_[i].data(), out_tensors[i].shape, output_data,
                output_shape, depad_dim, DTypeConvert::AS_IS, 0, 0);
          }
        } else if (tensor_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8) {
          auto output_data =
              ort_outputs[dod_out_index_[i]].GetTensorMutableData<uint8_t>();
          if (out_buffer_i8_[i].data() != 0) {
            depad<uint8_t, uint8_t>(
                out_buffer_i8_[i].data(), out_tensors[i].shape, output_data,
                output_shape, depad_dim, DTypeConvert::AS_IS, 0, 0);
          }
        } // else, no data conversion / depad required
      }
    }
  }
}
} // namespace vaip_dod_custom_op
