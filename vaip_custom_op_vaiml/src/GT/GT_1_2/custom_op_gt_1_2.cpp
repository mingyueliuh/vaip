
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

#include "onnxruntime_api.hpp"

#include "../../common/bf16_utils.h"
#include "../../common/timer.h"
#include "./custom_op_gt_1_2.hpp"
#include "constant_fold_result.h"
#include "gen_gt_wts.h"
#include <cassert>
#include <codecvt>
#include <deque>
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <istream>
#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <vaip/util.hpp>
#include <vaip/vaip.hpp>
namespace vaip_vaiml_custom_op {

size_t MyCustomOpGT1_2::gt_qkv_compute_iter = 0;
std::map<std::string, std::vector<char>> MyCustomOpGT1_2::node_cache;
SUBGRAPH_ID
MyCustomOpGT1_2::IdentifySubgraph(
    const std::shared_ptr<MetaDefProto>& meta_def) {
  // use with caution, this function identifies sg using the node name
  VAIML_DEBUG_PRINT("IdentifySubgraph sg_name=", sg_name_,
                    " meta_def size=", meta_def->nodes().size());
  SUBGRAPH_ID sg = SUBGRAPH_ID::UNKNOWN;
  // EPContext based subgraphs
  if (meta_def->nodes().size() == 1) {
    // All nodes in a subgraph becomes a EPContext node
    VAIML_DEBUG_PRINT("    EPContext model_version_=", model_version_,
                      " sg_name_=", sg_name_);
    if (model_version_ == "GT_v1.2") {
      if (sg_name_ == "vaiml_par_0") {
        sg = SUBGRAPH_ID::GT_TRANSFORMER_BLOCK;
      } else if (sg_name_ == "vaiml_par_1") {
        sg = SUBGRAPH_ID::GT_CACHE_FRAMES_SLICE;
      } else {
        printf("unknown cache name: %s", sg_name_.data());
      }
    }
  } else {
    // Normal ONNX model based subgraphs
    for (auto& node : meta_def->nodes()) {
      if (node.rfind("/linear_k", 0) != std::string::npos) {
        // vaiml_par_0 subgraph_000
        sg = SUBGRAPH_ID::GT_TRANSFORMER_BLOCK;
        break;
      } else if (node.rfind("/lin_dec/fc/", 0) != std::string::npos) {
        sg = SUBGRAPH_ID::HT_LN_SG_LSTM;
        break;
      } else if (node.rfind("/norm_k", 0) != std::string::npos) {
        // GT vaiml_par_1
        // sg = SUBGRAPH_ID::GT_NORM_K;
        // break;
      } else if (node.rfind("oup_cache_frames", 0) != std::string::npos) {
        // GT vaiml_par_4 subgraph_147
        sg = SUBGRAPH_ID::GT_CACHE_FRAMES_SLICE;
        break;
      } else if (node.rfind("h0_QuantizeLinear", 0) != std::string::npos) {
        sg = SUBGRAPH_ID::HT_SLICE;
        break;
      } else if (node.rfind("c1_QuantizeLinear", 0) != std::string::npos) {
        sg = SUBGRAPH_ID::HT_CONCAT;
      }
    }
  }

  if (sg == SUBGRAPH_ID::UNKNOWN) {
    throw std::runtime_error("Cannot identify subgraph ID for " + sg_name_);
  }

  return sg;
}

inline std::string
get_xclbin_fullpath(std::shared_ptr<const PassContext> context,
                    const std::string& xclbin) {
  VAIML_DEBUG_PRINT("    xclbin: ", xclbin);
  // Check if xclbin is cached
  std::string xclbin_config_path =
      context->xclbin_path_to_cache_files(std::filesystem::path(xclbin))
          .string();
  VAIML_DEBUG_PRINT("    xclbin_config_path: ", xclbin_config_path);
  if (!xclbin_config_path.empty()) {
    if (fs::exists(xclbin_config_path) &&
        (xclbin_config_path.find(".xclbin") != std::string::npos)) {
      return (xclbin_config_path);
    }
  }

  return xclbin;
}

MyCustomOpGT1_2::MyCustomOpGT1_2(std::shared_ptr<const PassContext> context,
                                 const std::shared_ptr<MetaDefProto>& meta_def,
                                 onnxruntime::Model* model)
    : CustomOpImp(context, meta_def, model) {
  auto& session_option = context->get_config_proto().provider_options();
  model_version_ = session_option.at("model_name");
  sg_name_ = meta_def->vaiml_param().vaiml_model_path();
  subgraph_id_ = IdentifySubgraph(meta_def);
  TIMER(CONSTRUCTOR_TOP, sg_name_ + " MyCustomOpGT1_2 constructor total ")
  VAIML_DEBUG_PRINT("MyCustomOpGT1_2::MyCustomOpGT1_2 ", sg_name_,
                    " id: ", subgraph_id_, " model_version=", model_version_);
  size_t batchsize = 0;
  for (auto& vaiml_shapes : meta_def->vaiml_param().output_shapes()) {
    ort_output_shapes_.emplace_back(vaiml_shapes.shapes().begin(),
                                    vaiml_shapes.shapes().end());
    auto& a_shape = ort_output_shapes_.back();
    // handle GT_FRONT specially. Only set batchsize for
    // out/Add_output_0_QuantizeLinear_Output (1x25x512)
    if (subgraph_id_ == SUBGRAPH_ID::GT_FRONT) {
      if (a_shape[1] == 25 && a_shape[2] == 512) {
        batchsize = a_shape[0];
      }
    } else {
      batchsize = a_shape[0];
    }

    std::string str = "==>";
    for (auto item : a_shape) {
      str += " ";
      str += std::to_string(item);
    }
    VAIML_DEBUG_PRINT2("    output shape dump: ", str);
  }

  bool gt_mode_ = model_version_ == "GT_v1.2";
  datatype_to_size[ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT] = sizeof(float);
  datatype_to_size[ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8] = sizeof(uint8_t);
  datatype_to_size[ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8] = sizeof(int8_t);
  datatype_to_size[ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16] = sizeof(uint16_t);
  datatype_to_size[ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16] = sizeof(int16_t);
  datatype_to_size[ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32] = sizeof(int32_t);
  datatype_to_size[ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64] = sizeof(int64_t);
  datatype_to_size[ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE] = sizeof(double);
  datatype_to_size[ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT32] = sizeof(uint32_t);
  datatype_to_size[ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT64] = sizeof(uint64_t);

  datatype_to_string[ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT] = "float";
  datatype_to_string[ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8] = "uint8_t";
  datatype_to_string[ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8] = "int8_t";
  datatype_to_string[ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16] = "uint16_t";
  datatype_to_string[ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16] = "int16_t";
  datatype_to_string[ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32] = "int32_t";
  datatype_to_string[ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64] = "int64_t";
  datatype_to_string[ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE] = "double";
  datatype_to_string[ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT32] = "uint32_t";
  datatype_to_string[ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT64] = "uint64_t";

  // retrieve wts file from the cache directory to populate wts_

  std::vector<char> wts_file;
  auto wts_file_opt = context->read_file_c8(constants_file_name_);
  wts_file = wts_file_opt.value();

  auto const_info = meta_def->vaiml_param().const_data_info();
  wts_buffers_.resize(const_info.size());

  VAIML_DEBUG_PRINT("const_info.size(): ", const_info.size());
  int wts_vec_idx = 0;
  // Following constants are preformatted for HT model, so they need to be
  // excluded

  for (auto it = const_info.begin(); it != const_info.end(); ++it) {
    flexmlrt::client::ErtIoTypeNew wts_vec;
    auto const_name = it->first;

    // if (std::find(ht_weight_names_preformat.begin(),
    // ht_weight_names_preformat.end(), const_name) !=
    // ht_weight_names_preformat.end()) {
    //   VAIML_DEBUG_PRINT("Skip preformatted weight: ", const_name);
    //   continue;
    // }

    VAIML_DEBUG_PRINT2("Constant idx: ", wts_vec_idx, " name: ", const_name);
    auto const_data_info = const_info.at(const_name);

    wts_vec.name = const_name;
    wts_vec.idx = wts_vec_idx;
    wts_vec.type = data_type_to_string(const_data_info.type());
    wts_vec.size = const_data_info.size();

    if (wts_vec.size == 0) {
      // Preformateed weights. Skip
      VAIML_DEBUG_PRINT("    Skip preformatted weight: ", const_name);
      continue;
    }

    std::string wt_shape_str;
    wts_vec.shape.push_back(batchsize);
    wt_shape_str += std::to_string(wts_vec.shape[wts_vec.shape.size() - 1]);
    for (auto dim : const_data_info.shape()) {
      wts_vec.shape.push_back(dim);
      wt_shape_str += " ";
      wt_shape_str += std::to_string(wts_vec.shape[wts_vec.shape.size() - 1]);
    }

    // Seek to the specified position
    size_t rd_offset = const_data_info.offset();
    if (rd_offset >= wts_file.size()) {
      throw std::runtime_error(
          "Failed to seek to the specified position in the wts file.");
    }
    // Resize the buffer to the specified size
    wts_buffers_[wts_vec_idx].resize(wts_vec.size);
    if (rd_offset + wts_vec.size > wts_file.size()) {
      throw std::runtime_error("Failed to read the specified amount of data.");
    }
    std::memcpy(wts_buffers_[wts_vec_idx].data(), wts_file.data() + rd_offset,
                wts_vec.size);
    wts_vec.data = wts_buffers_[wts_vec_idx].data();
    wts_[wts_vec.name] = wts_vec;
    wts_vec_idx++;

    VAIML_DEBUG_PRINT2("    weight name: ", wts_vec.name);
    VAIML_DEBUG_PRINT2("    weight id: ", wts_vec.idx);
    VAIML_DEBUG_PRINT2("    weight size: ", wts_vec.size);
    VAIML_DEBUG_PRINT2("    weight type: ", wts_vec.type);
    VAIML_DEBUG_PRINT2("    wt_shape_str: ", wt_shape_str);
    VAIML_DEBUG_PRINT2("    weight offset: ", rd_offset);
  }

  VAIML_DEBUG_PRINT("Total constants loaded: ", wts_vec_idx);

  std::string xclbinFileName;
  std::string txnBinFile_front;
  std::string txnBinFile;
  std::vector<std::string> txnbins;
  std::string ctrlPktBinFile;
  std::vector<std::string> ctrlPktbins;
  std::vector<XRTRunOffset> xrt_offset;
  xclbinFileName = get_xclbin_fullpath(context, session_option.at("xclbin"));
  // xclbinFileName = vaiml_model_path_ + '/' + "design.xclbin";

  VAIML_DEBUG_PRINT("xclbin path: ", xclbinFileName);

  size_t wts_size_front = 0;
  size_t ifm_size_front = 0;
  size_t ofm_size_front = 0;
  size_t tmp_size_front = 0;
  size_t wts_size = 0;
  size_t ifm_size = 0;
  size_t ofm_size = 0;
  size_t tmp_size = 0;
  std::vector<KERNEL_NM> kernel_indices;
  std::vector<BO_ORDER> bo_orders;
  XRTRunOffset off;
  // All configuration below are version specific
  if (model_version_ == "GT_v1.2") {
    if (subgraph_id_ == SUBGRAPH_ID::GT_TRANSFORMER_BLOCK) {
      wts_size = (2501056 + 10697152) /*GT_FRONT*/ +
                 (1783296 + 1472 + 9823296) * TRANSFORMER_BLOCK_NUM +
                 590400 /*GT_TAIL*/;
      ifm_size = 16480 /*GT_FRONT in*/ + 552960 /*GT_FRONT out*/ +
                 4300800 * TRANSFORMER_BLOCK_NUM + 25600 /*GT_TAIL IN*/ +
                 25600 /*GT_TAIL OUT*/;
      ofm_size = 0;
      tmp_size = 2129920 /*GT_FRONT*/ + 102400 + 576000 +
                 499200 /*no extra required for tail*/;
      off.ifm_offset = 0;
      off.wts_offset = 0;
      off.ofm_offset = 16480;
      off.tmp_offset = 0;
      xrt_offset.push_back(off);
      off.ifm_offset = 16480;
      off.wts_offset = 2501056;
      off.ofm_offset = 16480 + 552960;
      off.tmp_offset = 0;
      xrt_offset.push_back(off);
      txnBinFile_front = getBins_front_sub_ml_txn_1_2(model_version_);
      txnbins.push_back(txnBinFile_front);
      txnBinFile = getBins_out_matmul_add_ml_txn_1_2(model_version_);
      txnbins.push_back(txnBinFile);

      ctrlPktBinFile = getBins_out_matmul_add_ctrl_pkt_1_2(model_version_);
      // we push a dummy ctrl pkt here for conv, but not used
      ctrlPktbins.push_back(ctrlPktBinFile);
      ctrlPktbins.push_back(ctrlPktBinFile);
      kernel_indices.push_back(KERNEL_NM::GT_CONV);
      kernel_indices.push_back(KERNEL_NM::GT_MM);
      bo_orders.push_back(BO_ORDER::ODR_GT_CONV);
      bo_orders.push_back(BO_ORDER::ODR_GT_HEAD);
      for (int i = 0; i < TRANSFORMER_BLOCK_NUM; i++) {
        txnBinFile =
            getBins_linear_qkv_ln_bmm_concat_ml_txn_1_2(model_version_);
        txnbins.push_back(txnBinFile);
        txnBinFile = getBins_matmul_reduce_ml_txn_1_2(model_version_);
        txnbins.push_back(txnBinFile);
        txnBinFile =
            getBins_softmax_linear_out_feed_forward_ml_txn_1_2(model_version_);
        txnbins.push_back(txnBinFile);

        ctrlPktBinFile =
            getBins_linear_qkv_ln_bmm_concat_ctrl_pkt_1_2(model_version_);
        ctrlPktbins.push_back(ctrlPktBinFile);
        ctrlPktBinFile = getBins_matmul_reduce_ctrl_pkt_1_2(model_version_);
        ctrlPktbins.push_back(ctrlPktBinFile);
        ctrlPktBinFile = getBins_softmax_linear_out_feed_forward_ctrl_pkt_1_2(
            model_version_);
        ctrlPktbins.push_back(ctrlPktBinFile);

        size_t inout_base_ddr = i * 4300800 + 16480 + 552960;
        size_t wts_base_ddr =
            (2501056 + 10697152) + i * (1783296 + 1472 + 9823296);
        off.ifm_offset = inout_base_ddr;
        off.wts_offset = wts_base_ddr;
        off.ofm_offset = inout_base_ddr;
        off.tmp_offset = 0;
        xrt_offset.push_back(off);
        off.ifm_offset = inout_base_ddr;
        off.wts_offset = wts_base_ddr + 1783296;
        off.ofm_offset = inout_base_ddr;
        off.tmp_offset = 102400;
        xrt_offset.push_back(off);
        off.ifm_offset = inout_base_ddr;
        off.wts_offset = wts_base_ddr + 1783296 + 1472;
        off.ofm_offset = inout_base_ddr;
        off.tmp_offset = 102400 + 576000;
        xrt_offset.push_back(off);
        kernel_indices.push_back(KERNEL_NM::GT_MM);
        kernel_indices.push_back(KERNEL_NM::GT_MM);
        kernel_indices.push_back(KERNEL_NM::GT_MM);
        bo_orders.push_back(BO_ORDER::ODR_GT_TRANSFORMER);
        bo_orders.push_back(BO_ORDER::ODR_GT_TRANSFORMER);
        bo_orders.push_back(BO_ORDER::ODR_GT_TRANSFORMER);
      }
      txnBinFile = getBins_ln_matmul_add_ln_ml_txn_1_2(model_version_);
      txnbins.push_back(txnBinFile);
      ctrlPktBinFile = getBins_ln_matmul_add_ln_ctrl_pkt_1_2(model_version_);
      ctrlPktbins.push_back(ctrlPktBinFile);
      bo_orders.push_back(
          BO_ORDER::ODR_GT_TRANSFORMER); // the order is same as transformer and
                                         // we will be needing offsets
      kernel_indices.push_back(KERNEL_NM::GT_MM);
      off.ifm_offset = 16480 + 552960 + 4300800 * TRANSFORMER_BLOCK_NUM;
      off.wts_offset = (2501056 + 10697152) +
                       TRANSFORMER_BLOCK_NUM * (1783296 + 1472 + 9823296);
      off.ofm_offset = 16480 + 552960 + 4300800 * TRANSFORMER_BLOCK_NUM + 25600;
      off.tmp_offset = 0;
      xrt_offset.push_back(off);
      g.set_bo_order_vec(bo_orders);
    } else if (subgraph_id_ == SUBGRAPH_ID::GT_LN_MATMUL_ADD_LN) {
      wts_size = 590400;
      ifm_size = 25600;
      ofm_size = 25600;
      tmp_size = 51200; // dummy tmp, minimum size for xrt::bo
      txnBinFile = getBins_ln_matmul_add_ln_ml_txn_1_2(model_version_);
      txnbins.push_back(txnBinFile);
      ctrlPktBinFile = getBins_ln_matmul_add_ln_ctrl_pkt_1_2(model_version_);
      ctrlPktbins.push_back(ctrlPktBinFile);
      g.set_bo_order_vec({
          BO_ORDER::ODR_GT_TAIL,
      });
      kernel_indices.push_back(KERNEL_NM::GT_MM);
      xrt_offset.push_back(off); // dummy offset
    }
  }

  if (subgraph_id_ < GT_CPU_OR_CONSTANT) {
    auto read_xclbin = context->read_xclbin(xclbinFileName);
    auto xclbin = std::vector<char>(read_xclbin.value().begin(),
                                    read_xclbin.value().end());
    g.load_xclbin(xclbin);

    VAIML_DEBUG_PRINT("load xclbin done");
    g.load_txn_bin(txnbins);
    VAIML_DEBUG_PRINT("load txn done");
    if (ctrlPktbins.size() != 0) {
      // not used by HT
      g.load_ctrl_pkt_bin(ctrlPktbins);
    }
    VAIML_DEBUG_PRINT("load ctrl pkt done");
    g.hw_runner_init(ifm_size, wts_size, ofm_size, tmp_size, gt_mode_,
                     xrt_offset, kernel_indices);
    g.get_bo_ptrs(ifm_ptr_, wts_ptr_, ofm_ptr_);
  }

  TIMER(CONSTRUCTOR_WEIGHTS_FROMAT, "    " + sg_name_ + " weight format total ")
  VAIML_DEBUG_PRINT("Begin wts format for ", model_version_);
  if (model_version_ == "GT_v1.2") {
    size_t total_wts_bytes = 0;
    if (subgraph_id_ == SUBGRAPH_ID::GT_TRANSFORMER_BLOCK) {
      VAIML_DEBUG_PRINT("formatting transformer wts");
      subgraph_id_ = SUBGRAPH_ID::GT_FRONT;
      size_t total_wts_bytes_front = InitGtFrontWeight(wts_, wts_ptr_);
      wts_ptr_ += 2501056;
      total_wts_bytes = InitGtWeight(wts_, wts_ptr_);
      wts_ptr_ += 10697152;
      for (int i = 0; i < TRANSFORMER_BLOCK_NUM; i++) {
        subgraph_id_ = SUBGRAPH_ID::GT_QKV;
        total_wts_bytes = InitGtWeight(wts_, wts_ptr_);
        wts_ptr_ += 1783296;

        subgraph_id_ = SUBGRAPH_ID::GT_MATMUL_REDUCE;
        total_wts_bytes = InitGtWeight(wts_, wts_ptr_);

        wts_ptr_ += 1472;

        subgraph_id_ = SUBGRAPH_ID::GT_SM_LINEAR_OUT_FEED_FORWARD;
        total_wts_bytes = InitGtWeight(wts_, wts_ptr_);

        wts_ptr_ += 9823296;
      }
      // reset subgraph_id
      subgraph_id_ = SUBGRAPH_ID::GT_LN_MATMUL_ADD_LN;
      total_wts_bytes = InitGtWeight(wts_, wts_ptr_);
      wts_ptr_ += 590400;
      subgraph_id_ = SUBGRAPH_ID::GT_TRANSFORMER_BLOCK;
    } else if (subgraph_id_ < GT_CPU_OR_CONSTANT) {
      VAIML_DEBUG_PRINT("formatting other part wts");
      printf("initializing wts for unknown subgraph %d\n", subgraph_id_);
      size_t total_wts_bytes = InitGtWeight(wts_, wts_ptr_);
    }
  }

  VAIML_DEBUG_PRINT("Finish wts formated");
  // read_file_c8(wts_ptr_, vaiml_model_path_ + '/' + "wts32.txt", wts_size);

  if (subgraph_id_ < GT_CPU_OR_CONSTANT) {
    g.pre_run_bo_sync();
  }
  VAIML_DEBUG_PRINT("DEBUG: MyCustomOpGT1_2 created for ", sg_name_);
}

MyCustomOpGT1_2::~MyCustomOpGT1_2() {}

size_t MyCustomOpGT1_2::InitGtFrontWeight(
    std::unordered_map<std::string, flexmlrt::client::ErtIoTypeNew>& wts_,
    int8_t* wts_ptr_front) {
  std::string ifm_s_name, ifm_z_name, wts_s_name, wts_z_name, ofm_s_name,
      ofm_z_name, wts_w_name;
  size_t total_wts_bytes = 0;
  if (subgraph_id_ == SUBGRAPH_ID::GT_FRONT) {
    ifm_s_name = "cache_frames_scale";
    ifm_z_name = "cache_frames_zero_point";
    wts_s_name = "encoder_embedding.global_mean_scale";
    wts_z_name = "encoder_embedding.global_mean_zero_point";
    ofm_s_name = "/encoder_embedding/Sub_output_0_scale";
    ofm_z_name = "/encoder_embedding/Sub_output_0_zero_point";
    wts_w_name = "encoder_embedding.global_mean_quantized";
    std::string ifm_s_name1, ifm_z_name1, wts_s_name1, wts_z_name1, ofm_s_name1,
        ofm_z_name1, wts_w_name1;
    ifm_s_name1 = "/encoder_embedding/Sub_output_0_scale";
    ifm_z_name1 = "/encoder_embedding/Sub_output_0_zero_point";
    wts_s_name1 = "encoder_embedding.global_invstd_scale";
    wts_z_name1 = "encoder_embedding.global_invstd_zero_point";
    ofm_s_name1 = "/encoder_embedding/Mul_output_0_scale";
    ofm_z_name1 = "/encoder_embedding/Mul_output_0_zero_point";
    wts_w_name1 = "encoder_embedding.global_invstd_quantized";
    total_wts_bytes += GT_SUB_MUL_WTS_convert(
        wts_, wts_ptr_front, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, ifm_s_name1, ifm_z_name1,
        wts_s_name1, wts_z_name1, ofm_s_name1, ofm_z_name1, wts_w_name1);
    VAIML_DEBUG_PRINT2("Finish to format WTS for front subgraph SUB-MUL");

    std::string bias_s_name, bias_z_name, bias_w_name;
    ifm_s_name = "/encoder_embedding/Mul_output_0_scale";
    ifm_z_name = "/encoder_embedding/Mul_output_0_zero_point";
    wts_s_name = "encoder.embed.conv.0.weight_scale";
    wts_z_name = "encoder.embed.conv.0.weight_zero_point";
    bias_s_name = "encoder.embed.conv.0.bias_quantized_scale";
    bias_z_name = "encoder.encoders.6.norm2.bias_quantized_zero_point";
    ofm_s_name = "/conv/conv.1/Relu_output_0_scale";
    ofm_z_name = "/conv/conv.1/Relu_output_0_zero_point";
    wts_w_name = "encoder.embed.conv.0.weight_quantized";
    bias_w_name = "encoder.embed.conv.0.bias_quantized";
    total_wts_bytes += GT_CONV_WTS_convert(
        wts_, wts_ptr_front, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        bias_s_name, bias_z_name, ofm_s_name, ofm_z_name, wts_w_name,
        bias_w_name, 512, 1, 3, 3, 8, const_cast<uint32_t*>(conv0_c0),
        const_cast<uint32_t*>(conv0_lp));
    VAIML_DEBUG_PRINT2("Finish to format WTS for front subgraph CONV");

    ifm_s_name = "/conv/conv.1/Relu_output_0_scale";
    ifm_z_name = "/conv/conv.1/Relu_output_0_zero_point";
    wts_s_name = "encoder.embed.conv.2.weight_scale";
    wts_z_name = "encoder.embed.conv.2.weight_zero_point";
    bias_s_name = "encoder.embed.conv.2.bias_quantized_scale";
    bias_z_name = "encoder.encoders.6.norm2.bias_quantized_zero_point";
    ofm_s_name = "/conv/conv.3/Relu_output_0_scale";
    ofm_z_name = "/conv/conv.3/Relu_output_0_zero_point";
    wts_w_name = "encoder.embed.conv.2.weight_quantized";
    bias_w_name = "encoder.embed.conv.2.bias_quantized";
    total_wts_bytes += GT_CONV_WTS_convert(
        wts_, wts_ptr_front, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        bias_s_name, bias_z_name, ofm_s_name, ofm_z_name, wts_w_name,
        bias_w_name, 512, 512, 3, 3, 32, const_cast<uint32_t*>(conv1_c0),
        const_cast<uint32_t*>(conv1_lp));
    VAIML_DEBUG_PRINT2("Finish to format WTS for front subgraph CONV");
  }
  std::string out_filename(vaiml_model_path_ + '/' + sg_name_ +
                           "_dump_wts32_front_conv.txt");
  // write_file((uint32_t*)(wts_ptr_front - total_wts_bytes), total_wts_bytes,
  //  out_filename);
  return total_wts_bytes;
}

size_t MyCustomOpGT1_2::InitGtWeight(
    std::unordered_map<std::string, flexmlrt::client::ErtIoTypeNew>& wts_,
    int8_t* wts_ptr) {
  static int subgraph_index_linear_out_feed_forward = 0;
  static int subgraph_index_qkv = 0;
  static int subgraph_index_matmul_reduce = 0;
  int8_t* ori_wts_ptr = wts_ptr;
  int8_t* rtp_ptr = nullptr;
  if (subgraph_id_ == SUBGRAPH_ID::GT_QKV ||
      subgraph_id_ == SUBGRAPH_ID::GT_MATMUL_REDUCE ||
      subgraph_id_ == SUBGRAPH_ID::GT_SM_LINEAR_OUT_FEED_FORWARD ||
      subgraph_id_ == SUBGRAPH_ID::GT_LN_MATMUL_ADD_LN ||
      subgraph_id_ == SUBGRAPH_ID::GT_FRONT) {
    rtp_ptr = wts_ptr + gt_global_rtp_offset_.at(subgraph_id_);
    wts_ptr += 1472;
  }
  std::string ifm_s_name, ifm_z_name, wts_s_name, wts_z_name, ofm_s_name,
      ofm_z_name, wts_w_name;
  size_t total_wts_bytes = 0;
  if (subgraph_id_ == SUBGRAPH_ID::GT_SM_LINEAR_OUT_FEED_FORWARD) {
    std::string scale_s_name, scale_z_name, bias_s_name, bias_z_name,
        scale_name, bias_name;
    if (subgraph_index_linear_out_feed_forward > 0) {
      ifm_s_name =
          softmax_ifm_prefix_.at(subgraph_index_linear_out_feed_forward);
      // 6384, 6412, 6440, 6468, ..., 7364
      ifm_z_name =
          std::to_string(28 * subgraph_index_linear_out_feed_forward + 6384) +
          "_zero_point";
      ofm_s_name = "/Slice_61_output_0_scale";
      ofm_z_name = "/Softmax_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "_output_0_intermediate_zero_point";
    } else {
      ifm_s_name = "/Add_output_0_scale";
      ifm_z_name = "6384_zero_point";
      ofm_s_name = "/Slice_61_output_0_scale";
      ofm_z_name = "/Softmax_output_0_intermediate_zero_point";
    }
    total_wts_bytes += GT_SOFTMAX_WTS_convert(
        wts_, wts_ptr, ifm_s_name, ifm_z_name, ofm_s_name, ofm_z_name,
        480 /*K*/, 475 /*K_valid*/);
    VAIML_DEBUG_PRINT2("Finish to format WTS for linear_out subgraph-",
                       subgraph_index_linear_out_feed_forward, "-> Softmax");

    if (subgraph_index_linear_out_feed_forward > 0) {
      ifm_s_name = "/Slice_61_output_0_scale";
      ifm_z_name = "/Softmax_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "_output_0_intermediate_zero_point";
      std::set<int> set_slice = {1, 5, 9, 18, 22, 23, 28};
      std::set<int> set_concat = {2,  4,  10, 12, 13, 15, 16, 17, 19,
                                  20, 24, 25, 26, 27, 29, 32, 35};
      std::set<int> set_linear_v = {3, 6, 8, 30, 31, 34};
      std::set<int> set_unsqueeze = {7, 11, 14, 21, 33};
      if (set_slice.find(subgraph_index_linear_out_feed_forward) !=
          set_slice.end()) {
        // 3, 7, 11, 15, 18, ..., 143
        wts_s_name =
            "/Slice_" +
            std::to_string(4 * subgraph_index_linear_out_feed_forward + 3) +
            "_output_0_scale";
      } else if (set_concat.find(subgraph_index_linear_out_feed_forward) !=
                 set_concat.end()) {
        wts_s_name =
            "/Concat_" +
            std::to_string(8 * subgraph_index_linear_out_feed_forward + 115) +
            "_output_0_scale";
      } else if (set_linear_v.find(subgraph_index_linear_out_feed_forward) !=
                 set_linear_v.end()) {
        wts_s_name = "/linear_v_" +
                     std::to_string(subgraph_index_linear_out_feed_forward) +
                     "/Add_output_0_scale";
      } else if (set_unsqueeze.find(subgraph_index_linear_out_feed_forward) !=
                 set_unsqueeze.end()) {
        wts_s_name =
            "/Unsqueeze_" +
            std::to_string(25 * subgraph_index_linear_out_feed_forward + 385) +
            "_output_0_scale";
      }
      // 7, 123, 131, 139, 147, ..., 395
      wts_z_name =
          "/Concat_" +
          std::to_string(8 * subgraph_index_linear_out_feed_forward + 115) +
          "_output_0_zero_point";
      // 2, 5, 8, 11, 14, 107
      ofm_s_name =
          "/MatMul_" +
          std::to_string(3 * subgraph_index_linear_out_feed_forward + 2) +
          "_output_0_scale";
      ofm_z_name =
          "/MatMul_" +
          std::to_string(3 * subgraph_index_linear_out_feed_forward + 2) +
          "_output_0_zero_point";
      wts_w_name =
          "/Concat_" +
          std::to_string(8 * subgraph_index_linear_out_feed_forward + 115) +
          "_output_0_QuantizeLinear_Output";
    } else {
      ifm_s_name = "/Slice_61_output_0_scale";
      ifm_z_name = "/Softmax_output_0_intermediate_zero_point";
      wts_s_name = "/Slice_3_output_0_scale";
      wts_z_name = "/Concat_7_output_0_zero_point";
      ofm_s_name = "/MatMul_2_output_0_scale";
      ofm_z_name = "/MatMul_2_output_0_zero_point";
      wts_w_name = "/Concat_7_output_0_QuantizeLinear_Output";
    }
    total_wts_bytes += GT_BMM_WTS_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, false, 25, 512, 64, 16, 128, 16);
    VAIML_DEBUG_PRINT(
        "Finish to format WTS for linear_out_feed_forward subgraph-",
        subgraph_index_linear_out_feed_forward, "-> MM2");

    if (subgraph_index_linear_out_feed_forward > 0) {
      // 2, 5, 8, 11, ..., 107
      ifm_s_name =
          "/MatMul_" +
          std::to_string(3 * subgraph_index_linear_out_feed_forward + 2) +
          "_output_0_scale";
      ifm_z_name =
          "/MatMul_" +
          std::to_string(3 * subgraph_index_linear_out_feed_forward + 2) +
          "_output_0_zero_point";
      wts_s_name = "/linear_out_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Transpose_output_0_scale";
      wts_z_name = "/linear_out_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Transpose_output_0_zero_point";
      ofm_s_name = "/linear_out_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/MatMul_output_0_scale";
      ofm_z_name = "/linear_out_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/MatMul_output_0_zero_point";
      wts_w_name = "/linear_out_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Transpose_output_0_QuantizeLinear_Output";
    } else {
      ifm_s_name = "/MatMul_2_output_0_scale";
      ifm_z_name = "/MatMul_2_output_0_zero_point";
      wts_s_name = "/linear_out/Transpose_output_0_scale";
      wts_z_name = "/linear_out/Transpose_output_0_zero_point";
      ofm_s_name = "/linear_out/MatMul_output_0_scale";
      ofm_z_name = "/linear_out/MatMul_output_0_zero_point";
      wts_w_name = "/linear_out/Transpose_output_0_QuantizeLinear_Output";
    }
    total_wts_bytes += GT_MM_WTS_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, 25);
    VAIML_DEBUG_PRINT(
        "Finish to format WTS for linear_out_feed_forward subgraph-",
        subgraph_index_linear_out_feed_forward, "-> MM");

    if (subgraph_index_linear_out_feed_forward > 0) {
      ifm_s_name = "/linear_out_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/MatMul_output_0_scale";
      ifm_z_name = "/linear_out_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/MatMul_output_0_zero_point";
      wts_s_name = "encoder.encoders." +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   ".self_attn.linear_out.bias_scale";
      wts_z_name = "encoder.encoders." +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   ".self_attn.linear_out.bias_zero_point";
      if (subgraph_index_linear_out_feed_forward == 18) {
        ofm_s_name = "/linear_out_" +
                     std::to_string(subgraph_index_linear_out_feed_forward) +
                     "/MatMul_output_0_scale";
      } else {
        ofm_s_name = "/linear_out_" +
                     std::to_string(subgraph_index_linear_out_feed_forward) +
                     "/Add_output_0_scale";
      }
      ofm_z_name = "/linear_out_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Add_output_0_zero_point";
      wts_w_name = "encoder.encoders." +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   ".self_attn.linear_out.bias_quantized";
    } else {
      ifm_s_name = "/linear_out/MatMul_output_0_scale";
      ifm_z_name = "/linear_out/MatMul_output_0_zero_point";
      wts_s_name = "encoder.encoders.0.self_attn.linear_out.bias_scale";
      wts_z_name = "encoder.encoders.0.self_attn.linear_out.bias_zero_point";
      ofm_s_name = "/linear_out/Add_output_0_scale";
      ofm_z_name = "/linear_out/Add_output_0_zero_point";
      wts_w_name = "encoder.encoders.0.self_attn.linear_out.bias_quantized";
    }
    total_wts_bytes += GT_ADD_WTS_convert(wts_, wts_ptr, rtp_ptr, ifm_s_name,
                                          ifm_z_name, wts_s_name, wts_z_name,
                                          ofm_s_name, ofm_z_name, wts_w_name);
    VAIML_DEBUG_PRINT(
        "Finish to format WTS for linear_out_feed_forward subgraph-",
        subgraph_index_linear_out_feed_forward, "-> ADD");

    if (subgraph_index_linear_out_feed_forward > 0) {
      if (subgraph_index_linear_out_feed_forward == 18) {
        ifm_s_name = "/linear_out_" +
                     std::to_string(subgraph_index_linear_out_feed_forward) +
                     "/MatMul_output_0_scale";
      } else {
        ifm_s_name = "/linear_out_" +
                     std::to_string(subgraph_index_linear_out_feed_forward) +
                     "/Add_output_0_scale";
      }
      ifm_z_name = "/linear_out_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Add_output_0_zero_point";
      // out, 4, 9, 14, ... 174
      wts_s_name =
          "/Add_" +
          std::to_string(5 * subgraph_index_linear_out_feed_forward - 1) +
          "_output_0_scale";
      wts_z_name =
          "/Add_" +
          std::to_string(5 * subgraph_index_linear_out_feed_forward - 1) +
          "_output_0_zero_point";
      // 3, 8, 13, 18, ... 178
      ofm_s_name =
          "/Add_" +
          std::to_string(5 * subgraph_index_linear_out_feed_forward + 3) +
          "_output_0_scale";
      ofm_z_name =
          "/Add_" +
          std::to_string(5 * subgraph_index_linear_out_feed_forward + 3) +
          "_output_0_zero_point";
      wts_w_name =
          "/Add_" +
          std::to_string(5 * subgraph_index_linear_out_feed_forward - 1) +
          "_output_0_QuantizeLinear_Output";
    } else {
      ifm_s_name = "/linear_out/Add_output_0_scale";
      ifm_z_name = "/linear_out/Add_output_0_zero_point";
      wts_s_name = "/out/Add_output_0_scale";
      wts_z_name = "/out/Add_output_0_zero_point";
      ofm_s_name = "/Add_3_output_0_scale";
      ofm_z_name = "/Add_3_output_0_zero_point";
      wts_w_name = "/out/Add_output_0_QuantizeLinear_Output";
    }
    total_wts_bytes += GT_ADD_WTS_QDQ_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, 25 * 512);
    VAIML_DEBUG_PRINT(
        "Finish to format WTS for linear_out_feed_forward subgraph-",
        subgraph_index_linear_out_feed_forward, "-> ADD3");

    if (subgraph_index_linear_out_feed_forward > 0) {
      // 3, 8, 13, ..., 178
      ifm_s_name =
          "/Add_" +
          std::to_string(5 * subgraph_index_linear_out_feed_forward + 3) +
          "_output_0_scale";
      ifm_z_name =
          "/Add_" +
          std::to_string(5 * subgraph_index_linear_out_feed_forward + 3) +
          "_output_0_zero_point";
      if (subgraph_index_linear_out_feed_forward == 18) {
        scale_s_name = "encoder.encoders.21.norm2.weight_scale";
      } else {
        scale_s_name = "encoder.encoders." +
                       std::to_string(subgraph_index_linear_out_feed_forward) +
                       ".norm2.weight_scale";
      }
      scale_z_name = "encoder.encoders." +
                     std::to_string(subgraph_index_linear_out_feed_forward) +
                     ".norm2.weight_zero_point";
      bias_s_name = "encoder.encoders." +
                    std::to_string(subgraph_index_linear_out_feed_forward) +
                    ".norm2.bias_quantized_scale";
      bias_z_name = "encoder.encoders.6.norm2.bias_quantized_zero_point";
      ofm_s_name = "/norm2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Add_1_output_0_scale";
      ofm_z_name = "/norm2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Add_1_output_0_zero_point";
      scale_name = "encoder.encoders." +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   ".norm2.weight_quantized";
      bias_name = "encoder.encoders." +
                  std::to_string(subgraph_index_linear_out_feed_forward) +
                  ".norm2.bias_quantized";
    } else {
      ifm_s_name = "/Add_3_output_0_scale";
      ifm_z_name = "/Add_3_output_0_zero_point";
      scale_s_name = "encoder.encoders.0.norm2.weight_scale";
      scale_z_name = "encoder.encoders.0.norm2.weight_zero_point";
      bias_s_name = "encoder.encoders.0.norm2.bias_quantized_scale";
      bias_z_name = "encoder.encoders.6.norm2.bias_quantized_zero_point";
      ofm_s_name = "/norm2/Add_1_output_0_scale";
      ofm_z_name = "/norm2/Add_1_output_0_zero_point";
      scale_name = "encoder.encoders.0.norm2.weight_quantized";
      bias_name = "encoder.encoders.0.norm2.bias_quantized";
    }
    total_wts_bytes +=
        GT_LN_WTS_convert(wts_, wts_ptr, ifm_s_name, ifm_z_name, scale_s_name,
                          scale_z_name, bias_s_name, bias_z_name, ofm_s_name,
                          ofm_z_name, scale_name, bias_name);
    VAIML_DEBUG_PRINT(
        "Finish to format WTS for linear_out_feed_forward subgraph-",
        subgraph_index_linear_out_feed_forward, "-> LN");

    // feed forward
    if (subgraph_index_linear_out_feed_forward > 0) {
      ifm_s_name = "/norm2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Add_1_output_0_scale";
      ifm_z_name = "/norm2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Add_1_output_0_zero_point";
      wts_s_name = "/feed_forward/w_1_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Transpose_output_0_scale";
      wts_z_name = "/feed_forward/w_1_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Transpose_output_0_zero_point";
      ofm_s_name = "/feed_forward/w_1_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/MatMul_output_0_scale";
      ofm_z_name = "/feed_forward/w_1_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/MatMul_output_0_zero_point";
      wts_w_name = "/feed_forward/w_1_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Transpose_output_0_QuantizeLinear_Output";
    } else {
      ifm_s_name = "/norm2/Add_1_output_0_scale";
      ifm_z_name = "/norm2/Add_1_output_0_zero_point";
      wts_s_name = "/feed_forward/w_1/Transpose_output_0_scale";
      wts_z_name = "/feed_forward/w_1/Transpose_output_0_zero_point";
      ofm_s_name = "/feed_forward/w_1/MatMul_output_0_scale";
      ofm_z_name = "/feed_forward/w_1/MatMul_output_0_zero_point";
      wts_w_name = "/feed_forward/w_1/Transpose_output_0_QuantizeLinear_Output";
    }
    total_wts_bytes += GT_MM_WTS_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, 25);
    VAIML_DEBUG_PRINT(
        "Finish to format WTS for linear_out_feed_forward subgraph-",
        subgraph_index_linear_out_feed_forward, "-> MM_W1");

    if (subgraph_index_linear_out_feed_forward > 0) {
      ifm_s_name = "/feed_forward/w_1_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/MatMul_output_0_scale";
      ifm_z_name = "/feed_forward/w_1_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/MatMul_output_0_zero_point";
      wts_s_name = "encoder.encoders." +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   ".feed_forward.w_1.bias_scale";
      wts_z_name = "encoder.encoders." +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   ".feed_forward.w_1.bias_zero_point";
      ofm_s_name = "/feed_forward/act_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Relu_output_0_scale";
      ofm_z_name = "/feed_forward/act_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Relu_output_0_zero_point";
      wts_w_name = "encoder.encoders." +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   ".feed_forward.w_1.bias_quantized";
    } else {
      ifm_s_name = "/feed_forward/w_1/MatMul_output_0_scale";
      ifm_z_name = "/feed_forward/w_1/MatMul_output_0_zero_point";
      wts_s_name = "encoder.encoders.0.feed_forward.w_1.bias_scale";
      wts_z_name = "encoder.encoders.0.feed_forward.w_1.bias_zero_point";
      ofm_s_name = "/feed_forward/act/Relu_output_0_scale";
      ofm_z_name = "/feed_forward/act/Relu_output_0_zero_point";
      wts_w_name = "encoder.encoders.0.feed_forward.w_1.bias_quantized";
    }
    total_wts_bytes += GT_ADD_WTS_convert(wts_, wts_ptr, rtp_ptr, ifm_s_name,
                                          ifm_z_name, wts_s_name, wts_z_name,
                                          ofm_s_name, ofm_z_name, wts_w_name);
    VAIML_DEBUG_PRINT(
        "Finish to format WTS for linear_out_feed_forward subgraph-",
        subgraph_index_linear_out_feed_forward, "-> ADD_W1");

    if (subgraph_index_linear_out_feed_forward > 0) {
      ifm_s_name = "/feed_forward/act_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Relu_output_0_scale";
      ifm_z_name = "/feed_forward/act_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Relu_output_0_zero_point";
      wts_s_name = "/feed_forward/w_2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Transpose_output_0_scale";
      wts_z_name = "/feed_forward/w_2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Transpose_output_0_zero_point";
      ofm_s_name = "/feed_forward/w_2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/MatMul_output_0_scale";
      ofm_z_name = "/feed_forward/w_2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/MatMul_output_0_zero_point";
      wts_w_name = "/feed_forward/w_2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Transpose_output_0_QuantizeLinear_Output";
    } else {
      ifm_s_name = "/feed_forward/act/Relu_output_0_scale";
      ifm_z_name = "/feed_forward/act/Relu_output_0_zero_point";
      wts_s_name = "/feed_forward/w_2/Transpose_output_0_scale";
      wts_z_name = "/feed_forward/w_2/Transpose_output_0_zero_point";
      ofm_s_name = "/feed_forward/w_2/MatMul_output_0_scale";
      ofm_z_name = "/feed_forward/w_2/MatMul_output_0_zero_point";
      wts_w_name = "/feed_forward/w_2/Transpose_output_0_QuantizeLinear_Output";
    }
    total_wts_bytes += GT_MM_WTS_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, 25);
    VAIML_DEBUG_PRINT(
        "Finish to format WTS for linear_out_feed_forward subgraph-",
        subgraph_index_linear_out_feed_forward, "-> MM_W2");
    if (subgraph_index_linear_out_feed_forward > 0) {
      ifm_s_name = "/feed_forward/w_2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/MatMul_output_0_scale";
      ifm_z_name = "/feed_forward/w_2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/MatMul_output_0_zero_point";
      wts_s_name = "encoder.encoders." +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   ".feed_forward.w_2.bias_scale";
      wts_z_name = "encoder.encoders." +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   ".feed_forward.w_2.bias_zero_point";
      ofm_s_name = "/feed_forward/w_2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Add_output_0_scale";
      ofm_z_name = "/feed_forward/w_2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Add_output_0_zero_point";
      wts_w_name = "encoder.encoders." +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   ".feed_forward.w_2.bias_quantized";
    } else {
      ifm_s_name = "/feed_forward/w_2/MatMul_output_0_scale";
      ifm_z_name = "/feed_forward/w_2/MatMul_output_0_zero_point";
      wts_s_name = "encoder.encoders.0.feed_forward.w_2.bias_scale";
      wts_z_name = "encoder.encoders.0.feed_forward.w_2.bias_zero_point";
      ofm_s_name = "/feed_forward/w_2/Add_output_0_scale";
      ofm_z_name = "/feed_forward/w_2/Add_output_0_zero_point";
      wts_w_name = "encoder.encoders.0.feed_forward.w_2.bias_quantized";
    }
    total_wts_bytes += GT_ADD_WTS_convert(wts_, wts_ptr, rtp_ptr, ifm_s_name,
                                          ifm_z_name, wts_s_name, wts_z_name,
                                          ofm_s_name, ofm_z_name, wts_w_name);
    VAIML_DEBUG_PRINT(
        "Finish to format WTS for linear_out_feed_forward subgraph-",
        subgraph_index_linear_out_feed_forward, "-> ADD_W2");
    if (subgraph_index_linear_out_feed_forward > 0) {
      ifm_s_name = "/feed_forward/w_2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Add_output_0_scale";
      ifm_z_name = "/feed_forward/w_2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Add_output_0_zero_point";
      // 3, 8, 13, 18, ..., 178
      wts_s_name =
          "/Add_" +
          std::to_string(5 * subgraph_index_linear_out_feed_forward + 3) +
          "_output_0_scale";
      wts_z_name =
          "/Add_" +
          std::to_string(5 * subgraph_index_linear_out_feed_forward + 3) +
          "_output_0_zero_point";
      // 4, 9, 14, 19, ..., 179
      ofm_s_name =
          "/Add_" +
          std::to_string(5 * subgraph_index_linear_out_feed_forward + 4) +
          "_output_0_scale";
      ofm_z_name =
          "/Add_" +
          std::to_string(5 * subgraph_index_linear_out_feed_forward + 4) +
          "_output_0_zero_point";
      wts_w_name =
          "/Add_" +
          std::to_string(5 * subgraph_index_linear_out_feed_forward + 3) +
          "_output_0_QuantizeLinear_Output";
    } else {
      ifm_s_name = "/feed_forward/w_2/Add_output_0_scale";
      ifm_z_name = "/feed_forward/w_2/Add_output_0_zero_point";
      wts_s_name = "/Add_3_output_0_scale";
      wts_z_name = "/Add_3_output_0_zero_point";
      ofm_s_name = "/Add_4_output_0_scale";
      ofm_z_name = "/Add_4_output_0_zero_point";
      wts_w_name = "/Add_3_output_0_QuantizeLinear_Output";
    }
    total_wts_bytes += GT_ADD_WTS_QDQ_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, 25 * 512);
    VAIML_DEBUG_PRINT(
        "Finish to format WTS for linear_out_feed_forward subgraph-",
        subgraph_index_linear_out_feed_forward, "-> ADD4");
    subgraph_index_linear_out_feed_forward++;
  } else if (subgraph_id_ == SUBGRAPH_ID::GT_QKV) {
    std::string scale_s_name, scale_z_name, bias_s_name, bias_z_name,
        scale_name, bias_name;
    if (subgraph_index_qkv > 0) {
      ifm_s_name =
          str_fmt("/Add_%d_output_0_scale", subgraph_index_qkv * 5 - 1);
      ifm_z_name =
          str_fmt("/Add_%d_output_0_zero_point", subgraph_index_qkv * 5 - 1);
      scale_s_name =
          str_fmt("encoder.encoders.%d.norm1.weight_scale", subgraph_index_qkv);
      scale_z_name = str_fmt("encoder.encoders.%d.norm1.weight_zero_point",
                             subgraph_index_qkv);
      bias_s_name = str_fmt("encoder.encoders.%d.norm1.bias_quantized_scale",
                            subgraph_index_qkv);
      bias_z_name = "encoder.encoders.6.norm2.bias_quantized_zero_point";
      ofm_s_name =
          str_fmt("/norm1_%d/Add_1_output_0_scale", subgraph_index_qkv);
      ofm_z_name =
          str_fmt("/norm1_%d/Add_1_output_0_zero_point", subgraph_index_qkv);
      scale_name = str_fmt("encoder.encoders.%d.norm1.weight_quantized",
                           subgraph_index_qkv);
      bias_name = str_fmt("encoder.encoders.%d.norm1.bias_quantized",
                          subgraph_index_qkv);
    } else {
      ifm_s_name = "/out/Add_output_0_scale";
      ifm_z_name = "/out/Add_output_0_zero_point";
      scale_s_name = "encoder.encoders.0.norm1.weight_scale";
      scale_z_name = "encoder.encoders.0.norm1.weight_zero_point";
      bias_s_name = "encoder.encoders.0.norm1.bias_quantized_scale";
      bias_z_name = "encoder.encoders.6.norm2.bias_quantized_zero_point";
      ofm_s_name = "/norm1/Add_1_output_0_scale";
      ofm_z_name = "/norm1/Add_1_output_0_zero_point";
      scale_name = "encoder.encoders.0.norm1.weight_quantized";
      bias_name = "encoder.encoders.0.norm1.bias_quantized";
    }
    total_wts_bytes +=
        GT_LN_WTS_convert(wts_, wts_ptr, ifm_s_name, ifm_z_name, scale_s_name,
                          scale_z_name, bias_s_name, bias_z_name, ofm_s_name,
                          ofm_z_name, scale_name, bias_name);
    VAIML_DEBUG_PRINT2("Finish to format WTS for qkv subgraph-",
                       subgraph_index_qkv, "-> LN ");
    for (std::string fanout : std::vector<std::string>({"q", "k", "v"})) {
      if (subgraph_index_qkv > 0) {
        ifm_s_name =
            str_fmt("/norm1_%d/Add_1_output_0_scale", subgraph_index_qkv);
        ifm_z_name =
            str_fmt("/norm1_%d/Add_1_output_0_zero_point", subgraph_index_qkv);
        wts_s_name = str_fmt("/linear_%s_%d/Transpose_output_0_scale",
                             fanout.c_str(), subgraph_index_qkv);
        wts_z_name = str_fmt("/linear_%s_%d/Transpose_output_0_zero_point",
                             fanout.c_str(), subgraph_index_qkv);
        ofm_s_name = str_fmt("/linear_%s_%d/MatMul_output_0_scale", fanout,
                             subgraph_index_qkv);
        ofm_z_name = str_fmt("/linear_%s_%d/MatMul_output_0_zero_point", fanout,
                             subgraph_index_qkv);
        wts_w_name =
            str_fmt("/linear_%s_%d/Transpose_output_0_QuantizeLinear_Output",
                    fanout, subgraph_index_qkv);
      } else {
        ifm_s_name = "/norm1/Add_1_output_0_scale";
        ifm_z_name = "/norm1/Add_1_output_0_zero_point";
        wts_s_name =
            str_fmt("/linear_%s/Transpose_output_0_scale", fanout.c_str());
        wts_z_name =
            str_fmt("/linear_%s/Transpose_output_0_zero_point", fanout.c_str());
        ofm_s_name =
            str_fmt("/linear_%s/MatMul_output_0_scale", fanout.c_str());
        ofm_z_name =
            str_fmt("/linear_%s/MatMul_output_0_zero_point", fanout.c_str());
        wts_w_name = str_fmt(
            "/linear_%s/Transpose_output_0_QuantizeLinear_Output", fanout);
      }
      total_wts_bytes += GT_MM_WTS_convert(
          wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name,
          wts_z_name, ofm_s_name, ofm_z_name, wts_w_name, 25);
      VAIML_DEBUG_PRINT2("Finish to format WTS for qkv subgraph-",
                         subgraph_index_qkv, "-> MM ", fanout);
      if (subgraph_index_qkv > 0) {
        ifm_s_name = str_fmt("/linear_%s_%d/MatMul_output_0_scale", fanout,
                             subgraph_index_qkv);
        ifm_z_name = str_fmt("/linear_%s_%d/MatMul_output_0_zero_point", fanout,
                             subgraph_index_qkv);
        wts_s_name =
            str_fmt("encoder.encoders.%d.self_attn.linear_%s.bias_scale",
                    subgraph_index_qkv, fanout.c_str());
        wts_z_name =
            str_fmt("encoder.encoders.%d.self_attn.linear_%s.bias_zero_point",
                    subgraph_index_qkv, fanout.c_str());
        ofm_s_name = str_fmt("/linear_%s_%d/Add_output_0_scale", fanout.c_str(),
                             subgraph_index_qkv);
        ofm_z_name = str_fmt("/linear_%s_%d/Add_output_0_zero_point",
                             fanout.c_str(), subgraph_index_qkv);
        wts_w_name =
            str_fmt("encoder.encoders.%d.self_attn.linear_%s.bias_quantized",
                    subgraph_index_qkv, fanout.c_str());
      } else {
        ifm_s_name =
            str_fmt("/linear_%s/MatMul_output_0_scale", fanout.c_str());
        ifm_z_name =
            str_fmt("/linear_%s/MatMul_output_0_zero_point", fanout.c_str());
        wts_s_name =
            str_fmt("encoder.encoders.0.self_attn.linear_%s.bias_scale",
                    fanout.c_str());
        wts_z_name =
            str_fmt("encoder.encoders.0.self_attn.linear_%s.bias_zero_point",
                    fanout.c_str());
        ofm_s_name = str_fmt("/linear_%s/Add_output_0_scale", fanout.c_str());
        ofm_z_name =
            str_fmt("/linear_%s/Add_output_0_zero_point", fanout.c_str());
        wts_w_name =
            str_fmt("encoder.encoders.0.self_attn.linear_%s.bias_quantized",
                    fanout.c_str());
      }
      total_wts_bytes += GT_ADD_WTS_convert(wts_, wts_ptr, rtp_ptr, ifm_s_name,
                                            ifm_z_name, wts_s_name, wts_z_name,
                                            ofm_s_name, ofm_z_name, wts_w_name);
      VAIML_DEBUG_PRINT2("Finish to format WTS for qkv subgraph-",
                         subgraph_index_qkv, "-> ADD ", fanout);
    } // end of qkv matmul + add

    // skip linear_v reshape + transpose + concat don't have weights

    // start of linear_q reshape + transpose + mul + reshape + transpose + bmm
    if (subgraph_index_qkv > 0) {
      ifm_s_name =
          str_fmt("/linear_q_%d/Add_output_0_scale", subgraph_index_qkv);
      ifm_z_name =
          str_fmt("/linear_q_%d/Add_output_0_zero_point", subgraph_index_qkv);
      wts_s_name = "/Constant_303_output_0_scale";
      wts_z_name = mul_wts_prefix_.at(subgraph_index_qkv) + "zero_point";
      ofm_s_name = str_fmt("/Mul_%d_output_0_scale", subgraph_index_qkv * 5);
      ofm_z_name =
          str_fmt("/Mul_%d_output_0_zero_point", subgraph_index_qkv * 5);
      wts_w_name =
          mul_wts_prefix_.at(subgraph_index_qkv) + "QuantizeLinear_Output";
    } else {
      ifm_s_name = "/linear_q/Add_output_0_scale";
      ifm_z_name = "/linear_q/Add_output_0_zero_point";
      wts_s_name = "/Constant_303_output_0_scale";
      wts_z_name = mul_wts_prefix_.at(subgraph_index_qkv) + "zero_point";
      ofm_s_name = "/Mul_output_0_scale";
      ofm_z_name = "/Mul_output_0_zero_point";
      wts_w_name =
          mul_wts_prefix_.at(subgraph_index_qkv) + "QuantizeLinear_Output";
    }
    total_wts_bytes += GT_MUL_WTS_convert(wts_, wts_ptr, rtp_ptr, ifm_s_name,
                                          ifm_z_name, wts_s_name, wts_z_name,
                                          ofm_s_name, ofm_z_name, wts_w_name);
    VAIML_DEBUG_PRINT2("Finish to format WTS for qkv subgraph-",
                       subgraph_index_qkv, "-> Mul ");
    if (subgraph_index_qkv > 0) {
      ifm_s_name = str_fmt("/Mul_%d_output_0_scale", subgraph_index_qkv * 5);
      ifm_z_name =
          str_fmt("/Mul_%d_output_0_zero_point", subgraph_index_qkv * 5);
      wts_s_name = "/norm_k/Div_output_0_scale";
      wts_z_name = "/norm_k/Div_output_0_zero_point";
      ofm_s_name =
          str_fmt("/MatMul_%d_output_0_scale", subgraph_index_qkv * 3 + 1);
      ofm_z_name =
          str_fmt("/MatMul_%d_output_0_zero_point", subgraph_index_qkv * 3 + 1);
    } else {
      ifm_s_name = "/Mul_output_0_scale";
      ifm_z_name = "/Mul_output_0_zero_point";
      wts_s_name = "/norm_k/Div_output_0_scale";
      wts_z_name = "/norm_k/Div_output_0_zero_point";
      ofm_s_name = "/MatMul_1_output_0_scale";
      ofm_z_name = "/MatMul_1_output_0_zero_point";
    }
    total_wts_bytes += GT_BMM_WTS_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, false, 8, 64, 512, 16, 64,
        32); // 8 64 475,  N / row / col
    VAIML_DEBUG_PRINT2("Finish to format WTS for qkv subgraph-",
                       subgraph_index_qkv, "-> BMM ");
    // end of linear_q reshape + transpose + mul + reshape + transpose + bmm

    // skip linear_k reshape + transpose + concat don't have weights

    // qdq rtps for k,v gather-concat
    total_wts_bytes += GT_QDQ_convert(wts_, wts_ptr, rtp_ptr,
                                      v_unsqueeze_scale_[subgraph_index_qkv],
                                      v_unsqueeze_zp_[subgraph_index_qkv],
                                      v_concat_slice_scale_[subgraph_index_qkv],
                                      v_concat_slice_zp_[subgraph_index_qkv]);
    VAIML_DEBUG_PRINT2("Finish to format WTS for qkv subgraph-",
                       subgraph_index_qkv, "-> v concat QDQ ");

    total_wts_bytes += GT_QDQ_convert(wts_, wts_ptr, rtp_ptr,
                                      k_unsqueeze_scale_[subgraph_index_qkv],
                                      k_unsqueeze_zp_[subgraph_index_qkv],
                                      k_concat_slice_scale_[subgraph_index_qkv],
                                      k_concat_slice_zp_[subgraph_index_qkv]);
    VAIML_DEBUG_PRINT2("Finish to format WTS for qkv subgraph-",
                       subgraph_index_qkv, "-> k concat QDQ ");
    subgraph_index_qkv++;
  } else if (subgraph_id_ == SUBGRAPH_ID::GT_MATMUL_REDUCE) {
#define NAME_CONCAT(name1, beta, scale, name2)                                 \
  name1 + std::to_string(beta + subgraph_index_matmul_reduce * scale) + name2;
    if (subgraph_index_matmul_reduce > 0) {
      ifm_s_name = NAME_CONCAT("/Mul_", 0, 5, "_output_0_scale");
      ifm_z_name = NAME_CONCAT("/Mul_", 0, 5, "_output_0_zero_point");
      wts_s_name = NAME_CONCAT("/Concat_", 114, 8, "_output_0_scale");
      wts_z_name = NAME_CONCAT("/Concat_", 114, 8, "_output_0_zero_point");
      ofm_s_name = NAME_CONCAT("/MatMul_", 0, 3, "_output_0_scale");
      ofm_z_name = NAME_CONCAT("/MatMul_", 0, 3, "_output_0_zero_point");
    } else {
      ifm_s_name = "/Mul_output_0_scale";
      ifm_z_name = "/Mul_output_0_zero_point";
      wts_s_name = "/Concat_6_output_0_scale";
      wts_z_name = "/Concat_6_output_0_zero_point";
      ofm_s_name = "/MatMul_output_0_scale";
      ofm_z_name = "/MatMul_output_0_zero_point";
    }
    total_wts_bytes += GT_BMM_WTS_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, "", true, 25, 64, 512, 32, 64,
        32); // 25 64 475, N / row / col
    VAIML_DEBUG_PRINT2("Finish to format WTS for Matmul-Reduce subgraph-",
                       subgraph_index_matmul_reduce, "-> MATMUL");

    if (subgraph_index_matmul_reduce > 0) {
      ifm_s_name = NAME_CONCAT("/MatMul_", 0, 3, "_output_0_scale");
      ifm_z_name = NAME_CONCAT("/MatMul_", 0, 3, "_output_0_zero_point");
      wts_s_name = NAME_CONCAT("/MatMul_", 1, 3, "_output_0_scale");
      wts_z_name = NAME_CONCAT("/MatMul_", 1, 3, "_output_0_zero_point");
      ofm_s_name = NAME_CONCAT("/Add_", 0, 5, "_output_0_scale");
      ofm_z_name = NAME_CONCAT("/Add_", 0, 5, "_output_0_zero_point");
      wts_w_name = "";
    } else {
      ifm_s_name = "/MatMul_output_0_scale";
      ifm_z_name = "/MatMul_output_0_zero_point";
      wts_s_name = "/MatMul_1_output_0_scale";
      wts_z_name = "/MatMul_1_output_0_zero_point";
      ofm_s_name = "/Add_output_0_scale";
      ofm_z_name = "/Add_output_0_zero_point";
      wts_w_name = "";
    }
    total_wts_bytes += GT_ADD_WTS_QDQ_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, 8 * 25 * 475);
    VAIML_DEBUG_PRINT2("Finish to format WTS for Matmul-Reduce subgraph-",
                       subgraph_index_matmul_reduce, "-> ADD");

    if (subgraph_index_matmul_reduce > 0) {
      ifm_s_name = NAME_CONCAT("/Add_", 0, 5, "_output_0_scale");
      ifm_z_name = NAME_CONCAT("/Add_", 0, 5, "_output_0_zero_point");
      wts_s_name = "/Slice_61_output_0_scale";
      wts_z_name = NAME_CONCAT("/Slice_", 1, 4, "_output_0_zero_point");
      ofm_s_name = NAME_CONCAT("/Add_", 0, 5, "_output_0_scale");
      ofm_z_name = NAME_CONCAT("/Mul_", 3, 5, "_output_0_zero_point");
      wts_w_name = "";
    } else {
      ifm_s_name = "/Add_output_0_scale";
      ifm_z_name = "/Add_output_0_zero_point";
      wts_s_name = "/Slice_61_output_0_scale";
      wts_z_name = "/Slice_1_output_0_zero_point";
      ofm_s_name = "/Add_output_0_scale";
      ofm_z_name = "/Mul_3_output_0_zero_point";
      wts_w_name = "";
    }
    total_wts_bytes += GT_MUL_WTS_QDQ_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, 8 * 25 * 480);
    VAIML_DEBUG_PRINT2("Finish to format WTS for Matmul-Reduce subgraph-",
                       subgraph_index_matmul_reduce, "-> Mul ");

    if (subgraph_index_matmul_reduce > 0) {
      ifm_s_name = NAME_CONCAT("/Add_", 0, 5, "_output_0_scale");
      ifm_z_name = NAME_CONCAT("/Mul_", 3, 5, "_output_0_zero_point");
      wts_s_name = "/Slice_61_output_0_scale";
      wts_z_name = NAME_CONCAT("/Slice_", 0, 4, "_output_0_zero_point");
      ofm_s_name = NAME_CONCAT("/Mul_", 4, 5, "_output_0_scale");
      ofm_z_name = NAME_CONCAT("/Mul_", 4, 5, "_output_0_zero_point");
      wts_w_name = "";
    } else {
      ifm_s_name = "/Add_output_0_scale";
      ifm_z_name = "/Mul_3_output_0_zero_point";
      wts_s_name = "/Slice_61_output_0_scale";
      wts_z_name = "/Slice_output_0_zero_point";
      ofm_s_name = "/Tile_output_0_scale";
      ofm_z_name = "/Mul_4_output_0_zero_point";
      wts_w_name = "";
    }
    total_wts_bytes += GT_MUL_WTS_QDQ_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, 8 * 25 * 480);
    VAIML_DEBUG_PRINT2("Finish to format WTS for Matmul-Reduce subgraph-",
                       subgraph_index_matmul_reduce, "-> Mul ");

    if (subgraph_index_matmul_reduce > 0) {
      ifm_s_name = NAME_CONCAT("/Mul_", 3, 5, "_output_0_scale");
      ifm_z_name = NAME_CONCAT("/Mul_", 3, 5, "_output_0_zero_point");
      wts_s_name = NAME_CONCAT("/Mul_", 4, 5, "_output_0_scale");
      wts_z_name = NAME_CONCAT("/Mul_", 4, 5, "_output_0_zero_point");
      ofm_s_name = NAME_CONCAT("/Add_", 2, 5, "_output_0_scale");
      ofm_z_name = NAME_CONCAT("/Add_", 2, 5, "_output_0_zero_point");
      wts_w_name = "";
    } else {
      ifm_s_name = "/Add_output_0_scale";
      ifm_z_name = "/Add_output_0_zero_point";
      wts_s_name = "/Tile_output_0_scale";
      wts_z_name = "/Tile_output_0_zero_point";
      ofm_s_name = "/Add_output_0_scale";
      ofm_z_name = "/Add_output_0_zero_point";
      wts_w_name = "";
    }
    total_wts_bytes += GT_ADD_WTS_QDQ_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, 8 * 25 * 480);
    VAIML_DEBUG_PRINT2("Finish to format WTS for Matmul-Reduce subgraph-",
                       subgraph_index_matmul_reduce, "-> ADD");
    subgraph_index_matmul_reduce++;
#undef NAME_CONCAT
  } else if (subgraph_id_ == SUBGRAPH_ID::GT_LN_MATMUL_ADD_LN) {
    std::string scale_s_name, scale_z_name, bias_s_name, bias_z_name,
        scale_name, bias_name;
    ifm_s_name = "/Add_179_output_0_scale";
    ifm_z_name = "/Add_179_output_0_zero_point";
    scale_s_name = "encoder.after_norm.weight_scale";
    scale_z_name = "encoder.after_norm.weight_zero_point";
    bias_s_name = "encoder.after_norm.bias_quantized_scale";
    bias_z_name = "encoder.encoders.6.norm2.bias_quantized_zero_point";
    ofm_s_name = "/after_norm/Add_1_output_0_scale";
    ofm_z_name = "/after_norm/Add_1_output_0_zero_point";
    scale_name = "encoder.after_norm.weight_quantized";
    bias_name = "encoder.after_norm.bias_quantized";
    total_wts_bytes +=
        GT_LN_WTS_convert(wts_, wts_ptr, ifm_s_name, ifm_z_name, scale_s_name,
                          scale_z_name, bias_s_name, bias_z_name, ofm_s_name,
                          ofm_z_name, scale_name, bias_name);
    VAIML_DEBUG_PRINT2("Finish to format WTS for LN-MATMUL-ADD-LN subgraph-",
                       subgraph_index_qkv, "-> LN ");

    ifm_s_name = "/after_norm/Add_1_output_0_scale";
    ifm_z_name = "/after_norm/Add_1_output_0_zero_point";
    wts_s_name = "/lin_enc/fc/Transpose_output_0_scale";
    wts_z_name = "/lin_enc/fc/Transpose_output_0_zero_point";
    ofm_s_name = "/lin_enc/fc/MatMul_output_0_scale";
    ofm_z_name = "/lin_enc/fc/MatMul_output_0_zero_point";
    wts_w_name = "/lin_enc/fc/Transpose_output_0_QuantizeLinear_Output";
    total_wts_bytes += GT_MM_WTS_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, 25);
    VAIML_DEBUG_PRINT2("Finish to format WTS for LN-MATMUL-ADD-LN subgraph-",
                       subgraph_index_qkv, "-> MM");

    ifm_s_name = "/lin_enc/fc/MatMul_output_0_scale";
    ifm_z_name = "/lin_enc/fc/MatMul_output_0_zero_point";
    wts_s_name = "joint_network.lin_enc.fc.bias_scale";
    wts_z_name = "joint_network.lin_enc.fc.bias_zero_point";
    ofm_s_name = "/lin_enc/fc/Add_output_0_scale";
    ofm_z_name = "/lin_enc/fc/Add_output_0_zero_point";
    wts_w_name = "joint_network.lin_enc.fc.bias_quantized";
    total_wts_bytes += GT_ADD_WTS_convert(wts_, wts_ptr, rtp_ptr, ifm_s_name,
                                          ifm_z_name, wts_s_name, wts_z_name,
                                          ofm_s_name, ofm_z_name, wts_w_name);
    VAIML_DEBUG_PRINT2("Finish to format WTS for LN-MATMUL-ADD-LN subgraph-",
                       subgraph_index_qkv, "-> ADD");

    ifm_s_name = "/lin_enc/fc/Add_output_0_scale";
    ifm_z_name = "/lin_enc/fc/Add_output_0_zero_point";
    scale_s_name = "joint_network.lin_enc.Lnorm.weight_scale";
    scale_z_name = "joint_network.lin_enc.Lnorm.weight_zero_point";
    bias_s_name = "joint_network.lin_enc.Lnorm.bias_quantized_scale";
    bias_z_name = "encoder.encoders.6.norm2.bias_quantized_zero_point";
    ofm_s_name = "hidden_state_scale";
    ofm_z_name = "hidden_state_zero_point";
    scale_name = "joint_network.lin_enc.Lnorm.weight_quantized";
    bias_name = "joint_network.lin_enc.Lnorm.bias_quantized";
    total_wts_bytes +=
        GT_LN_WTS_convert(wts_, wts_ptr, ifm_s_name, ifm_z_name, scale_s_name,
                          scale_z_name, bias_s_name, bias_z_name, ofm_s_name,
                          ofm_z_name, scale_name, bias_name);
    VAIML_DEBUG_PRINT2("Finish to format WTS for LN-MATMUL-ADD-LN subgraph-",
                       subgraph_index_qkv, "-> LN ");
  } else if (subgraph_id_ == SUBGRAPH_ID::GT_FRONT) {
    const size_t dim1 = 512;
    const size_t dim2 = 19;
    const size_t dim3 = 512;
    const size_t new_dim2 = 20;
    uint16_t* data = new uint16_t[dim1 * new_dim2 * dim3];
    ifm_s_name = "/conv/conv.3/Relu_output_0_scale";
    ifm_z_name = "/conv/conv.3/Relu_output_0_zero_point";
    wts_s_name = "/out/Transpose_output_0_scale";
    wts_z_name = "/out/Transpose_output_0_zero_point";
    ofm_s_name = "/out/MatMul_output_0_scale";
    ofm_z_name = "/out/MatMul_output_0_zero_point";
    wts_w_name = "/out/Transpose_output_0_QuantizeLinear_Output";
    std::fill(data, data + dim1 * new_dim2 * dim3, uint16_t(33329));
    uint16_t* original_data = (uint16_t*)(wts_[wts_w_name].data);
    // 512x19x512 -> 512x20x512
    for (size_t i = 0; i < dim1; ++i) {
      for (size_t j = 0; j < dim2; ++j) {
        std::memcpy(&data[(i * new_dim2 + j) * dim3],
                    &original_data[(i * dim2 + j) * dim3],
                    dim3 * sizeof(uint16_t));
      }
    }
    total_wts_bytes += GT_MM_WTS_convert_ptr(
        wts_, data, 25, dim1 * new_dim2, dim3, wts_ptr, rtp_ptr, ifm_s_name,
        ifm_z_name, wts_s_name, wts_z_name, ofm_s_name, ofm_z_name, wts_w_name);
    VAIML_DEBUG_PRINT2("Finish to format WTS for front subgraph-",
                       subgraph_index_qkv, "-> MM");
    delete[] data;

    wts_s_name = "encoder.embed.out.bias_scale";
    wts_z_name = "encoder.embed.out.bias_zero_point";
    ifm_s_name = "/out/MatMul_output_0_scale";
    ifm_z_name = "/out/MatMul_output_0_zero_point";
    ofm_s_name = "/out/Add_output_0_scale";
    ofm_z_name = "/out/Add_output_0_zero_point";
    wts_w_name = "encoder.embed.out.bias_quantized";
    total_wts_bytes += GT_ADD_WTS_convert(wts_, wts_ptr, rtp_ptr, ifm_s_name,
                                          ifm_z_name, wts_s_name, wts_z_name,
                                          ofm_s_name, ofm_z_name, wts_w_name);
    VAIML_DEBUG_PRINT2("Finish to format WTS for front subgraph-",
                       subgraph_index_qkv, "-> ADD");
  }
  std::string out_filename(vaiml_model_path_ + '/' + sg_name_ +
                           "_dump_wts32.txt");
  // write_file((uint32_t*)(ori_wts_ptr), 10697152, out_filename);

  if (subgraph_id_ == SUBGRAPH_ID::GT_QKV) {
    // rtp layer k, q, v
    // wts layer q, k, v
    // need to swap rtp of k, q
    int8_t swap_buffer[128];
    memcpy(swap_buffer, ori_wts_ptr, 128);
    memcpy(ori_wts_ptr, ori_wts_ptr + 128, 128);
    memcpy(ori_wts_ptr + 128, swap_buffer, 128);
  }
  // std::string out_filename(vaiml_model_path_ +
  // '/'+sg_name_+"_dump_wts32.txt");
  // write_file((uint32_t*)(wts_ptr -
  // total_wts_bytes), total_wts_bytes, out_filename);
  // if (subgraph_id_ == SUBGRAPH_ID::GT_QKV) {
  //   write_file_binary((uint32_t*)(ori_wts_ptr), 1783296,
  //   vaiml_model_path_ +
  //   '/'+sg_name_+"_" +std::to_string(subgraph_id_) + "_dump_wts32.txt");
  // } else if (subgraph_id_ == SUBGRAPH_ID::GT_MATMUL_REDUCE) {
  //   write_file_binary((uint32_t*)(ori_wts_ptr), 1472,
  //   vaiml_model_path_ +
  //   '/'+sg_name_+"_" +std::to_string(subgraph_id_) + "_dump_wts32.txt");
  // } else if (subgraph_id_ == SUBGRAPH_ID::GT_SM_LINEAR_OUT_FEED_FORWARD) {
  //   write_file_binary((uint32_t*)(ori_wts_ptr), 9823296,
  //   vaiml_model_path_ +
  //   '/'+sg_name_+"_" +std::to_string(subgraph_id_) + "_dump_wts32.txt");
  // } else if (subgraph_id_ == SUBGRAPH_ID::GT_LN_MATMUL_ADD_LN) {
  //   write_file_binary((uint32_t*)(ori_wts_ptr), 590400,
  //   vaiml_model_path_ +
  //   '/'+sg_name_+"_" +std::to_string(subgraph_id_) + "_dump_wts32.txt");
  // }
  return total_wts_bytes;
}

void MyCustomOpGT1_2::SetUpGTBmmWithConstants(int8_t* ifm_ptr) const {
  auto cache_iter = node_cache.find("BMM_IFM2");
  if (cache_iter == node_cache.end()) {
    for (int i = 0; i < 25 * 64; i++) {
      memcpy((uint16_t*)ifm_ptr + i * 480,
             (const uint16_t*)
                     transpose_6_output_0_QuantizeLinear_output_str.data() +
                 i * 475,
             475 * sizeof(uint16_t));
    }
    std::vector<char> bmm_ifm2_cache(25 * 64 * 480 * sizeof(uint16_t), 0);
    memcpy(bmm_ifm2_cache.data(), ifm_ptr, bmm_ifm2_cache.size());
    node_cache.emplace("BMM_IFM2", std::move(bmm_ifm2_cache));
  } else {
    const std::vector<char>& bmm_ifm2_cache = cache_iter->second;
    memcpy(ifm_ptr, bmm_ifm2_cache.data(), bmm_ifm2_cache.size());
  }
}

int32_t MyCustomOpGT1_2::GetInputDataAndSet_GT(Ort::KernelContext& ctx,
                                               int index, int8_t* ifm_ptr,
                                               SUBGRAPH_ID subgraph_id) const {
  auto inputvalue = ctx.GetInput(index);
  const void* input = inputvalue.GetTensorRawData();
  auto type_and_shape = inputvalue.GetTensorTypeAndShapeInfo();
  auto num_elements = type_and_shape.GetElementCount();
  auto input_type = type_and_shape.GetElementType();
  CHECK(datatype_to_size.count(input_type))
      << "unsupported data type " << input_type;
  if (subgraph_id == SUBGRAPH_ID::GT_TRANSFORMER_BLOCK &&
      num_elements == 190000) {
    // Mul before & after ReduceMin
    // need to perform slicing
    // 8x25x950 -> 8x25x475(0-475, padded to 480) for mul_4 (onnx index 4)
    // 8x25x950 -> 8x25x475(475-950, padded to 480) for mul_3 (onnx index 5)
    auto cache_iter = node_cache.find("Mul_4");
    if (cache_iter == node_cache.end()) {
      for (int slice_id = 0; slice_id < 2; ++slice_id) {
        uint32_t inner_offset = slice_id == 0 ? 0 : 475;
        for (int i = 0; i < 8 * 25; i++) {
          memcpy((uint16_t*)ifm_ptr + slice_id * 8 * 25 * 480 + i * 480,
                 (const uint16_t*)input + i * 950 + inner_offset,
                 475 * sizeof(uint16_t));
        }
      }
      std::vector<char> mul4_cache(8 * 25 * 480 * sizeof(uint16_t), 0);
      memcpy(mul4_cache.data(), ifm_ptr, mul4_cache.size());
      node_cache.emplace("Mul_4", std::move(mul4_cache));

      std::vector<char> mul3_cache(8 * 25 * 480 * sizeof(uint16_t), 0);
      memcpy(mul3_cache.data(), ifm_ptr + 8 * 25 * 480 * sizeof(uint16_t),
             mul3_cache.size());
      node_cache.emplace("Mul_3", std::move(mul3_cache));
    } else {
      const std::vector<char>& mul4_cache = cache_iter->second;
      memcpy(ifm_ptr, mul4_cache.data(), mul4_cache.size());
      const std::vector<char>& mul3_cache = node_cache.at("Mul_3");
      memcpy(ifm_ptr + 8 * 25 * 480 * sizeof(uint16_t), mul3_cache.data(),
             mul3_cache.size());
    }

    num_elements = 8 * 25 * 480 * 2;
  } else if (subgraph_id == SUBGRAPH_ID::GT_TRANSFORMER_BLOCK &&
             num_elements == 8294400 &&
             input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
    // k v gather
    // slice and pad
    // 36x8x450x64 -> 1x8x475x64
    for (int i = 0; i < 8; i++) {
      float_to_bfloat16_avx512_unrolled(
          (const float*)input + (gt_qkv_compute_iter - 1) * 8 * 450 * 64 +
              i * 450 * 64,
          (uint16_t*)ifm_ptr + i * 475 * 64, 450 * 64);
    }
    num_elements = 8 * 475 * 64;
  } else {
    memcpy(ifm_ptr, input, num_elements * sizeof(uint16_t));
  }

  return num_elements * sizeof(uint16_t);
}
int32_t MyCustomOpGT1_2::GetOutputDataAndSet_GT(Ort::KernelContext& ctx,
                                                int index,
                                                int8_t* ofm_ptr) const {
  auto output_shapes = ort_output_shapes_;
  auto& output_shape = output_shapes[index];
  auto ortvalue =
      ctx.GetOutput(index, output_shape.data(), output_shape.size());
  auto type_and_shape = ortvalue.GetTensorTypeAndShapeInfo();
  auto tensor_type = type_and_shape.GetElementType();
  auto num_elements = type_and_shape.GetElementCount();

  void* data = ortvalue.GetTensorMutableRawData();
  if (subgraph_id_ == SUBGRAPH_ID::GT_TRANSFORMER_BLOCK &&
      num_elements == 36 * 8 * 450 * 64) {
    // k v concat split concat
    // 8x475x64 -> 8x450x64 -> 36x8x450x64
    bool is_v_concat = (index == 1);
    size_t gt_front_sz = 16480 + 552960;
    size_t xrt_offset_kv = is_v_concat ? 1561600 : 2048000;
    for (int iter = 0; iter < TRANSFORMER_BLOCK_NUM; iter++) {
      size_t xrt_base_ddr = iter * 4300800 + xrt_offset_kv + gt_front_sz;
      size_t ort_base_ddr = iter * 8 * 450 * 64;
      for (int i = 0; i < 8; i++) {
        bfloat16_to_float_avx512_unrolled(
            (uint16_t*)(ofm_ptr + xrt_base_ddr) + i * 475 * 64 + 25 * 64,
            (float*)data + ort_base_ddr + i * 450 * 64, 450 * 64);
      }
    }
    num_elements = 36 * 8 * 450 * 64;
  } else {
    memcpy(data, ofm_ptr, num_elements * sizeof(uint16_t));
  }

  // flexmlrt::client::ErtIoType ofm_vec;
  // ofm_vec.data = std::move(data);
  // ofm_vec.name =
  //     (index == 0) ? "ofm_ddr" : ("ofm_ddr_" + std::to_string(index));
  // ofm_vec.idx = (int)index;
  // ofm_vec.size = num_elements * datatype_to_size.at(tensor_type);
  // VAIML_DEBUG_PRINT("    output ", index, " data buffer size: ",
  // ofm_vec.size); ofm_vec.type = datatype_to_string.at(tensor_type);
  // VAIML_DEBUG_PRINT("    output ", index, " type: ", ofm_vec.type);
  // for (int i = 0; i < 10; ++i)
  //   VAIML_DEBUG_PRINT("    output[", index, "]  ",
  //                     (int)(((uint16_t*)(data))[i]));

  return num_elements * sizeof(uint16_t);
}

int32_t MyCustomOpGT1_2::Slice144Compute_GT(Ort::KernelContext& ctx) const {
  auto inputvalue = ctx.GetInput(0);
  const void* input = inputvalue.GetTensorRawData();

  auto output_shapes = ort_output_shapes_;
  auto& output_shape = output_shapes[0];
  auto ortvalue = ctx.GetOutput(0, output_shape.data(), output_shape.size());
  void* output = ortvalue.GetTensorMutableRawData();
  int zp = 0;
  float scale = 0.000409241154557094;
  for (int i = 0; i < 3 * 80; i++) {
    ((float*)output)[i] = (((const uint16_t*)input)[100 * 80 + i] - zp) * scale;
  }
  return 3 * 80 * sizeof(float);
}

void MyCustomOpGT1_2::Compute(const OrtApi* api,
                              OrtKernelContext* context) const {
  if (Ort::Global<void>::api_ == nullptr) {
    Ort::Global<void>::api_ = api;
  }
  Ort::KernelContext ctx(context);
  TIMER(Compute, sg_name_ + " ort compute total ")

  // GT_NORM_K is now merged into GT_FRONT because EPContext does not support
  // subgraph without input
  // if (subgraph_id_ == SUBGRAPH_ID::GT_NORM_K) {
  //  // cpu constant folding, return the cached result directly
  //  GetOutputDataAndSet_GT(
  //      ctx, 0,
  //      (int8_t*)transpose_6_output_0_QuantizeLinear_output_str.data());
  //  return;
  //}

  if (subgraph_id_ == SUBGRAPH_ID::GT_CACHE_FRAMES_SLICE) {
    Slice144Compute_GT(ctx);
    return;
  }

  auto num_inputs = ctx.GetInputCount();
  VAIML_DEBUG_PRINT("    inputs number: ", num_inputs);
  int32_t ifm_offset = 0;
  if (model_version_ == "GT_v1.2") {
    TIMER(IFM, sg_name_ + " gt mode input memcpy ")
    if (subgraph_id_ == SUBGRAPH_ID::GT_LN_MATMUL_ADD_LN) {
      ifm_offset += GetInputDataAndSet_GT(ctx, 0, ifm_ptr_, subgraph_id_);
    } else if (subgraph_id_ == SUBGRAPH_ID::GT_TRANSFORMER_BLOCK) {
      GetInputDataAndSet_GT(
          ctx, 0, ifm_ptr_,
          SUBGRAPH_ID::GT_FRONT); // this is the input for conv part
      const size_t gt_front_io_sz = 16480 + 552960;
      for (int i = 0; i < TRANSFORMER_BLOCK_NUM; i++) {
        gt_qkv_compute_iter++;
        if (gt_qkv_compute_iter == 37) {
          gt_qkv_compute_iter = 1;
        }
        /* onnx order:
                cache_frames_QuantizeLinear_Output --> conv
                inp_cache_k (k-gather)
                inp_cache_v (v-gather)
                mask_QuantizeLinear_Output (2 mul)
          txn order:
            bmm, v gather, k gather,
            |mul after reducemin(0:475 of mask), mul before reducemin(475:950 of
          mask)|
        */
        // txn order: qkv ln, bmm, v gather, k gather, |mul after
        // reducemin(0:475 of mask), mul before reducemin(475:950 of mask)|
        size_t base_ddr = i * 4300800;
        // setting bmm using directly result of constant folding
        SetUpGTBmmWithConstants(ifm_ptr_ + gt_front_io_sz + base_ddr + 25600);
        GetInputDataAndSet_GT(ctx, 2,
                              ifm_ptr_ + gt_front_io_sz + base_ddr + 1561600,
                              subgraph_id_);
        GetInputDataAndSet_GT(ctx, 1,
                              ifm_ptr_ + gt_front_io_sz + base_ddr + 2048000,
                              subgraph_id_);
        GetInputDataAndSet_GT(ctx, 3,
                              ifm_ptr_ + gt_front_io_sz + base_ddr + 2534400,
                              subgraph_id_);
      }
      // no need to set tail input as it is to be produced by intermediate
      // blocks
      if (node_cache.find("Mul_3") != node_cache.end()) {
        node_cache.erase("Mul_3");
        node_cache.erase("Mul_4");
      }
    }
  }

  // write_file((uint32_t*)ifm_ptr_, ifm_offset, vaiml_model_path_ +
  // '/'+sg_name_+"_ifm.txt");

  auto output_shapes = ort_output_shapes_;
  VAIML_DEBUG_PRINT("    outputs number: ", output_shapes.size());
  auto err_status = const_cast<hw_runner&>(g).run(
      (void*)ifm_ptr_, (void*)wts_ptr_, (void*)ofm_ptr_);

  int32_t ofm_offset = 0;
  if (model_version_ == "GT_v1.2") {
    TIMER(OFM, sg_name_ + " gt mode output memcpy ")
    if (subgraph_id_ == SUBGRAPH_ID::GT_TRANSFORMER_BLOCK) {
      /*
       onnx order:
            oup_lid_QuantizeLinear_Output
            oup_cache_v
            oup_cache_k
            /Add_179_output_0_QuantizeLinear_Output
       */
      {
        // v concat /concat_400
        GetOutputDataAndSet_GT(ctx, 1, ofm_ptr_);
        // k concat /concat_399
        GetOutputDataAndSet_GT(ctx, 2, ofm_ptr_);
      }
      // /oup_lid_QuantizeLinear_Output
      size_t gt_front_sz = 16480 + 552960;
      size_t gt_tail_in_sz = 25600;
      GetOutputDataAndSet_GT(ctx, 0, ofm_ptr_ + gt_front_sz + 64512000);
      // /Add_179_output_0_QuantizeLinear_Output
      GetOutputDataAndSet_GT(
          ctx, 3, ofm_ptr_ + gt_front_sz + 154828800 + gt_tail_in_sz);
    }
  }
  if (err_status == -2) {
    printf("BOARD CRASHED\n");
  }

  // write_file((uint32_t*)ofm_ptr_, ofm_offset, vaiml_model_path_ +
  // '/'+sg_name_+"_ofm.txt");
  // write_file_binary((uint32_t*)ofm_ptr_, 12928000, vaiml_model_path_ +
  // '/'+sg_name_+"_ofm.txt");
}

} // namespace vaip_vaiml_custom_op
