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

/*

**/
#pragma once

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#endif

// #define WTS_SIZE_HT 17868672
// #define IFM_SIZE_HT 15104
// #define OFM_SIZE_HT 9344
// #define TMP_SIZE_HT 11904
#define TRANSFORMER_BLOCK_NUM 36
#include "txn_pkg_gt_1_2.hpp"
// #include "ht_txn_pkg.hpp"
#include "../../common/hw_runner.h"
// #include "load_wts.h"
#include "../../common/utils.h"
#include "../../common/vaiml_client.h"
#include "onnxruntime_api.hpp"
#include "vaip/vaip.hpp"
#include "vitis/ai/env_config.hpp"
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
using VaimlTensorShape = std::vector<int64_t>;
using VaimlShapeVec = std::vector<VaimlTensorShape>;

namespace vaip_vaiml_custom_op {
using namespace vaip_core;

class MyCustomOpGT1_2 : public CustomOpImp {
public:
  MyCustomOpGT1_2(std::shared_ptr<const PassContext> context,
                  const std::shared_ptr<MetaDefProto>& meta_def,
                  onnxruntime::Model* model);

  virtual ~MyCustomOpGT1_2();

private:
  void real_compute() const;
  void MyCustomOpGraphMode(std::shared_ptr<const PassContext> context,
                           const onnxruntime::Graph* graph,
                           const std::shared_ptr<MetaDefProto>& meta_def);
  void MyCustomOpTransformerMode(std::shared_ptr<const PassContext> context,
                                 const onnxruntime::Graph* graph,
                                 const std::shared_ptr<MetaDefProto>& meta_def);

  void forwardUkernel(std::map<int, int>& datatype_to_size,
                      std::map<int, std::string>& datatype_to_string,
                      const VaimlShapeVec& output_shapes,
                      Ort::KernelContext& ctx) const;

  virtual void Compute(const OrtApi* api,
                       OrtKernelContext* context) const override final;
  // int32_t GetInputDataAndSet(Ort::KernelContext& ctx, int index,
  //                            int8_t* ifm_ptr) const;
  // int32_t GetOnputDataAndSet(Ort::KernelContext& ctx, int index,
  //                            int8_t* ofm_ptr) const;

  // bool InitHtWeight(
  //     std::unordered_map<std::string, flexmlrt::client::ErtIoTypeNew>& wts_,
  //     int8_t* wts);
  size_t InitGtFrontWeight(
      std::unordered_map<std::string, flexmlrt::client::ErtIoTypeNew>& wts_,
      int8_t* wts_ptr_front);
  size_t InitGtWeight(
      std::unordered_map<std::string, flexmlrt::client::ErtIoTypeNew>& wts_,
      int8_t* wts_ptr);
  int32_t GetInputDataAndSet_GT(Ort::KernelContext& ctx, int index,
                                int8_t* ifm_ptr, SUBGRAPH_ID subgraph_id) const;
  void SetUpGTBmmWithConstants(int8_t* ifm_ptr) const;
  int32_t GetOutputDataAndSet_GT(Ort::KernelContext& ctx, int index,
                                 int8_t* ofm_ptr) const;
  int32_t Slice144Compute_GT(Ort::KernelContext& ctx) const;
  // int32_t SliceCompute_HT(Ort::KernelContext& ctx) const;
  // int32_t ConcatCompute_HT(Ort::KernelContext& ctx) const;

  SUBGRAPH_ID IdentifySubgraph(const std::shared_ptr<MetaDefProto>& meta_def);

  SUBGRAPH_ID subgraph_id_ = SUBGRAPH_ID::UNKNOWN;
  const std::map<SUBGRAPH_ID, size_t> gt_global_rtp_offset_ = {
      {SUBGRAPH_ID::GT_QKV, 0},
      {SUBGRAPH_ID::GT_MATMUL_REDUCE, 512},
      {SUBGRAPH_ID::GT_SM_LINEAR_OUT_FEED_FORWARD, 832},
      {SUBGRAPH_ID::GT_LN_MATMUL_ADD_LN, 896},
      {SUBGRAPH_ID::GT_FRONT, 896}};
  std::string sg_name_;
  static std::map<std::string, std::vector<char>> node_cache;
  static size_t gt_qkv_compute_iter;
  bool debug_ = false;
  std::string vaiml_model_path_ = "vaiml_par_0";
  std::string device_name_ = "phx";
  std::string config_filename_ = "";
  std::shared_ptr<flexmlrt::client::Model> runner_;
  VaimlShapeVec ort_output_shapes_;
  hw_runner g;
  int8_t* wts_ptr_front_;
  int8_t* ifm_ptr_front_;
  int8_t* ofm_ptr_front_;
  int8_t* wts_ptr_;
  int8_t* ifm_ptr_;
  int8_t* ofm_ptr_;
  std::string constants_file_name_ = "wts.bin";
  std::unordered_map<std::string, flexmlrt::client::ErtIoTypeNew> wts_;
  std::vector<std::vector<char>> wts_buffers_;
  std::map<int, int> datatype_to_size;
  std::map<int, std::string> datatype_to_string;
  std::string model_version_;

  const std::map<int, std::string> mul_wts_prefix_ = {
      {0, "/Constant_15_output_0_"},   {1, "/Constant_37_output_0_"},
      {2, "/Constant_54_output_0_"},   {3, "/Constant_71_output_0_"},
      {4, "/Constant_88_output_0_"},   {5, "/Constant_106_output_0_"},
      {6, "/Constant_124_output_0_"},  {7, "/Constant_142_output_0_"},
      {8, "/Constant_159_output_0_"},  {9, "/Constant_177_output_0_"},
      {10, "/Constant_195_output_0_"}, {11, "/Constant_213_output_0_"},
      {12, "/Constant_231_output_0_"}, {13, "/Constant_249_output_0_"},
      {14, "/Constant_267_output_0_"}, {15, "/Constant_285_output_0_"},
      {16, "/Constant_303_output_0_"}, {17, "/Constant_321_output_0_"},
      {18, "/Constant_339_output_0_"}, {19, "/Constant_357_output_0_"},
      {20, "/Constant_375_output_0_"}, {21, "/Constant_393_output_0_"},
      {22, "/Constant_411_output_0_"}, {23, "/Constant_429_output_0_"},
      {24, "/Constant_447_output_0_"}, {25, "/Constant_465_output_0_"},
      {26, "/Constant_483_output_0_"}, {27, "/Constant_501_output_0_"},
      {28, "/Constant_519_output_0_"}, {29, "/Constant_537_output_0_"},
      {30, "/Constant_555_output_0_"}, {31, "/Constant_573_output_0_"},
      {32, "/Constant_591_output_0_"}, {33, "/Constant_609_output_0_"},
      {34, "/Constant_627_output_0_"}, {35, "/Constant_645_output_0_"},
  };

  const std::map<int, std::string> softmax_ifm_prefix_ = {
      {0, "/Add_output_0_scale"},
      {1, "/Add_5_output_0_scale"},
      {2, "6440_scale"},
      {3, "/Add_17_output_0_scale"},
      {4, "/Add_22_output_0_scale"},
      {5, "/Mul_28_output_0_scale"},
      {6, "6552_scale"},
      {7, "/Add_37_output_0_scale"},
      {8, "/Add_42_output_0_scale"},
      {9, "/Add_47_output_0_scale"},
      {10, "6664_scale"},
      {11, "/Add_55_output_0_scale"},
      {12, "/Add_62_output_0_scale"},
      {13, "/Mul_68_output_0_scale"},
      {14, "/Mul_73_output_0_scale"},
      {15, "/Add_75_output_0_scale"},
      {16, "/Mul_83_output_0_scale"},
      {17, "/Add_87_output_0_scale"},
      {18, "6888_scale"},
      {19, "6916_scale"},
      {20, "/Add_102_output_0_scale"},
      {21, "6972_scale"},
      {22, "/Add_110_output_0_scale"},
      {23, "/Add_115_output_0_scale"},
      {24, "/Mul_123_output_0_scale"},
      {25, "7084_scale"},
      {26, "/Add_132_output_0_scale"},
      {27, "/Add_135_output_0_scale"},
      {28, "/Add_142_output_0_scale"},
      {29, "/Add_145_output_0_scale"},
      {30, "/Mul_153_output_0_scale"},
      {31, "/Mul_158_output_0_scale"},
      {32, "/Mul_163_output_0_scale"},
      {33, "/Add_167_output_0_scale"},
      {34, "/Mul_173_output_0_scale"},
      {35, "/Mul_178_output_0_scale"},
  };
  const std::vector<std::string> k_unsqueeze_scale_ = {
      "/Unsqueeze_26_output_0_scale",   "/Unsqueeze_411_output_0_scale",
      "/Unsqueeze_436_output_0_scale",  "/Unsqueeze_461_output_0_scale",
      "/Slice_18_output_0_scale",       "/Unsqueeze_511_output_0_scale",
      "/Unsqueeze_536_output_0_scale",  "/Unsqueeze_561_output_0_scale",
      "/Unsqueeze_586_output_0_scale",  "/Unsqueeze_611_output_0_scale",
      "/Unsqueeze_636_output_0_scale",  "/Unsqueeze_661_output_0_scale",
      "/Unsqueeze_686_output_0_scale",  "/Unsqueeze_711_output_0_scale",
      "/Unsqueeze_736_output_0_scale",  "/Unsqueeze_761_output_0_scale",
      "/Concat_242_output_0_scale",     "/Unsqueeze_811_output_0_scale",
      "/Slice_74_output_0_scale",       "/Unsqueeze_861_output_0_scale",
      "/Unsqueeze_886_output_0_scale",  "/Unsqueeze_911_output_0_scale",
      "/Unsqueeze_936_output_0_scale",  "/Unsqueeze_961_output_0_scale",
      "/Unsqueeze_986_output_0_scale",  "/Unsqueeze_1011_output_0_scale",
      "/Unsqueeze_1036_output_0_scale", "/Slice_110_output_0_scale",
      "/Slice_114_output_0_scale",      "/Concat_346_output_0_scale",
      "/Concat_354_output_0_scale",     "/Unsqueeze_1161_output_0_scale",
      "/Concat_370_output_0_scale",     "/Unsqueeze_1211_output_0_scale",
      "/Unsqueeze_1236_output_0_scale", "/Unsqueeze_1261_output_0_scale"};
  const std::vector<std::string> k_unsqueeze_zp_ = {
      "/Unsqueeze_26_output_0_zero_point",
      "/Unsqueeze_411_output_0_zero_point",
      "/Unsqueeze_436_output_0_zero_point",
      "/Unsqueeze_461_output_0_zero_point",
      "/Unsqueeze_486_output_0_zero_point",
      "/Unsqueeze_511_output_0_zero_point",
      "/Unsqueeze_536_output_0_zero_point",
      "/Unsqueeze_561_output_0_zero_point",
      "/Unsqueeze_586_output_0_zero_point",
      "/Unsqueeze_611_output_0_zero_point",
      "/Unsqueeze_636_output_0_zero_point",
      "/Unsqueeze_661_output_0_zero_point",
      "/Unsqueeze_686_output_0_zero_point",
      "/Unsqueeze_711_output_0_zero_point",
      "/Unsqueeze_736_output_0_zero_point",
      "/Unsqueeze_761_output_0_zero_point",
      "/Unsqueeze_786_output_0_zero_point",
      "/Unsqueeze_811_output_0_zero_point",
      "/Unsqueeze_836_output_0_zero_point",
      "/Unsqueeze_861_output_0_zero_point",
      "/Unsqueeze_886_output_0_zero_point",
      "/Unsqueeze_911_output_0_zero_point",
      "/Unsqueeze_936_output_0_zero_point",
      "/Unsqueeze_961_output_0_zero_point",
      "/Unsqueeze_986_output_0_zero_point",
      "/Unsqueeze_1011_output_0_zero_point",
      "/Unsqueeze_1036_output_0_zero_point",
      "/Unsqueeze_1061_output_0_zero_point",
      "/Unsqueeze_1086_output_0_zero_point",
      "/Unsqueeze_1111_output_0_zero_point",
      "/Unsqueeze_1136_output_0_zero_point",
      "/Unsqueeze_1161_output_0_zero_point",
      "/Unsqueeze_1186_output_0_zero_point",
      "/Unsqueeze_1211_output_0_zero_point",
      "/Unsqueeze_1236_output_0_zero_point",
      "/Unsqueeze_1261_output_0_zero_point"};
  const std::vector<std::string> v_unsqueeze_scale_ = {
      "/linear_v/Add_output_0_scale",    "/Slice_7_output_0_scale",
      "/Unsqueeze_435_output_0_scale",   "/Unsqueeze_460_output_0_scale",
      "/Unsqueeze_485_output_0_scale",   "/Unsqueeze_510_output_0_scale",
      "/Unsqueeze_535_output_0_scale",   "/Unsqueeze_560_output_0_scale",
      "/linear_v_8/Add_output_0_scale",  "/Unsqueeze_610_output_0_scale",
      "/linear_v_10/Add_output_0_scale", "/Unsqueeze_660_output_0_scale",
      "/Unsqueeze_685_output_0_scale",   "/Unsqueeze_710_output_0_scale",
      "/Unsqueeze_735_output_0_scale",   "/Concat_235_output_0_scale",
      "/Concat_243_output_0_scale",      "/Concat_251_output_0_scale",
      "/Unsqueeze_835_output_0_scale",   "/Unsqueeze_860_output_0_scale",
      "/Concat_275_output_0_scale",      "/Unsqueeze_910_output_0_scale",
      "/linear_v_22/Add_output_0_scale", "/Slice_95_output_0_scale",
      "/linear_v_24/Add_output_0_scale", "/Unsqueeze_1010_output_0_scale",
      "/Unsqueeze_1035_output_0_scale",  "/linear_v_27/Add_output_0_scale",
      "/Unsqueeze_1085_output_0_scale",  "/Concat_347_output_0_scale",
      "/Unsqueeze_1135_output_0_scale",  "/Unsqueeze_1160_output_0_scale",
      "/Concat_371_output_0_scale",      "/Unsqueeze_1210_output_0_scale",
      "/Unsqueeze_1235_output_0_scale",  "/Unsqueeze_1260_output_0_scale"};
  const std::vector<std::string> v_unsqueeze_zp_ = {
      "/Unsqueeze_25_output_0_zero_point",
      "/Unsqueeze_410_output_0_zero_point",
      "/Unsqueeze_435_output_0_zero_point",
      "/Unsqueeze_460_output_0_zero_point",
      "/Unsqueeze_485_output_0_zero_point",
      "/Unsqueeze_510_output_0_zero_point",
      "/Unsqueeze_535_output_0_zero_point",
      "/Unsqueeze_560_output_0_zero_point",
      "/Unsqueeze_585_output_0_zero_point",
      "/Unsqueeze_610_output_0_zero_point",
      "/Unsqueeze_635_output_0_zero_point",
      "/Unsqueeze_660_output_0_zero_point",
      "/Unsqueeze_685_output_0_zero_point",
      "/Unsqueeze_710_output_0_zero_point",
      "/Unsqueeze_735_output_0_zero_point",
      "/Unsqueeze_760_output_0_zero_point",
      "/Unsqueeze_785_output_0_zero_point",
      "/Unsqueeze_810_output_0_zero_point",
      "/Unsqueeze_835_output_0_zero_point",
      "/Unsqueeze_860_output_0_zero_point",
      "/Unsqueeze_885_output_0_zero_point",
      "/Unsqueeze_910_output_0_zero_point",
      "/Unsqueeze_935_output_0_zero_point",
      "/Unsqueeze_960_output_0_zero_point",
      "/Unsqueeze_985_output_0_zero_point",
      "/Unsqueeze_1010_output_0_zero_point",
      "/Unsqueeze_1035_output_0_zero_point",
      "/Unsqueeze_1060_output_0_zero_point",
      "/Unsqueeze_1085_output_0_zero_point",
      "/Unsqueeze_1110_output_0_zero_point",
      "/Unsqueeze_1135_output_0_zero_point",
      "/Unsqueeze_1160_output_0_zero_point",
      "/Unsqueeze_1185_output_0_zero_point",
      "/Unsqueeze_1210_output_0_zero_point",
      "/Unsqueeze_1235_output_0_zero_point",
      "/Unsqueeze_1260_output_0_zero_point"};
  const std::vector<std::string> k_concat_slice_scale_ = {
      "/Concat_6_output_0_scale",        "/Concat_122_output_0_scale",
      "/Slice_10_output_0_scale",        "/Slice_14_output_0_scale",
      "/Slice_18_output_0_scale",        "/Concat_154_output_0_scale",
      "/linear_k_6/Add_output_0_scale",  "/Concat_170_output_0_scale",
      "/Unsqueeze_586_output_0_scale",   "/Slice_38_output_0_scale",
      "/Concat_194_output_0_scale",      "/Unsqueeze_661_output_0_scale",
      "/linear_k_12/Add_output_0_scale", "/linear_k_13/Add_output_0_scale",
      "/Concat_226_output_0_scale",      "/Concat_234_output_0_scale",
      "/Concat_242_output_0_scale",      "/Concat_250_output_0_scale",
      "/Slice_74_output_0_scale",        "/Concat_266_output_0_scale",
      "/Concat_274_output_0_scale",      "/Slice_86_output_0_scale",
      "/Concat_290_output_0_scale",      "/Slice_94_output_0_scale",
      "/Concat_306_output_0_scale",      "/Concat_314_output_0_scale",
      "/Unsqueeze_1036_output_0_scale",  "/Slice_110_output_0_scale",
      "/Slice_114_output_0_scale",       "/Concat_346_output_0_scale",
      "/Concat_354_output_0_scale",      "/Unsqueeze_1161_output_0_scale",
      "/Concat_370_output_0_scale",      "/Unsqueeze_1211_output_0_scale",
      "/Concat_386_output_0_scale",      "/Slice_142_output_0_scale"};
  const std::vector<std::string> k_concat_slice_zp_ = {
      "/Slice_2_output_0_zero_point",   "/Slice_6_output_0_zero_point",
      "/Slice_10_output_0_zero_point",  "/Slice_14_output_0_zero_point",
      "/Slice_18_output_0_zero_point",  "/Slice_22_output_0_zero_point",
      "/Slice_26_output_0_zero_point",  "/Slice_30_output_0_zero_point",
      "/Slice_34_output_0_zero_point",  "/Slice_38_output_0_zero_point",
      "/Slice_42_output_0_zero_point",  "/Slice_46_output_0_zero_point",
      "/Slice_50_output_0_zero_point",  "/Slice_54_output_0_zero_point",
      "/Slice_58_output_0_zero_point",  "/Slice_62_output_0_zero_point",
      "/Slice_66_output_0_zero_point",  "/Slice_70_output_0_zero_point",
      "/Slice_74_output_0_zero_point",  "/Slice_78_output_0_zero_point",
      "/Slice_82_output_0_zero_point",  "/Slice_86_output_0_zero_point",
      "/Slice_90_output_0_zero_point",  "/Slice_94_output_0_zero_point",
      "/Slice_98_output_0_zero_point",  "/Slice_102_output_0_zero_point",
      "/Slice_106_output_0_zero_point", "/Slice_110_output_0_zero_point",
      "/Slice_114_output_0_zero_point", "/Slice_118_output_0_zero_point",
      "/Slice_122_output_0_zero_point", "/Slice_126_output_0_zero_point",
      "/Slice_130_output_0_zero_point", "/Slice_134_output_0_zero_point",
      "/Slice_138_output_0_zero_point", "/Slice_142_output_0_zero_point"};
  const std::vector<std::string> v_concat_slice_scale_ = {
      "/Slice_3_output_0_scale",         "/Slice_7_output_0_scale",
      "/Concat_131_output_0_scale",      "/linear_v_3/Add_output_0_scale",
      "/Concat_147_output_0_scale",      "/Slice_23_output_0_scale",
      "/linear_v_6/Add_output_0_scale",  "/Unsqueeze_560_output_0_scale",
      "/linear_v_8/Add_output_0_scale",  "/Slice_39_output_0_scale",
      "/Concat_195_output_0_scale",      "/Unsqueeze_660_output_0_scale",
      "/Concat_211_output_0_scale",      "/Concat_219_output_0_scale",
      "/Unsqueeze_735_output_0_scale",   "/Concat_235_output_0_scale",
      "/Concat_243_output_0_scale",      "/Concat_251_output_0_scale",
      "/Slice_75_output_0_scale",        "/Concat_267_output_0_scale",
      "/Concat_275_output_0_scale",      "/Unsqueeze_910_output_0_scale",
      "/Slice_91_output_0_scale",        "/Slice_95_output_0_scale",
      "/Concat_307_output_0_scale",      "/Concat_315_output_0_scale",
      "/Concat_323_output_0_scale",      "/Concat_331_output_0_scale",
      "/Slice_115_output_0_scale",       "/Concat_347_output_0_scale",
      "/linear_v_30/Add_output_0_scale", "/linear_v_31/Add_output_0_scale",
      "/Concat_371_output_0_scale",      "/Unsqueeze_1210_output_0_scale",
      "/linear_v_34/Add_output_0_scale", "/Concat_395_output_0_scale"};
  const std::vector<std::string> v_concat_slice_zp_ = {
      "/Slice_3_output_0_zero_point",   "/Slice_7_output_0_zero_point",
      "/Slice_11_output_0_zero_point",  "/Slice_15_output_0_zero_point",
      "/Slice_19_output_0_zero_point",  "/Slice_23_output_0_zero_point",
      "/Slice_27_output_0_zero_point",  "/Slice_31_output_0_zero_point",
      "/Slice_35_output_0_zero_point",  "/Slice_39_output_0_zero_point",
      "/Slice_43_output_0_zero_point",  "/Slice_47_output_0_zero_point",
      "/Slice_51_output_0_zero_point",  "/Slice_55_output_0_zero_point",
      "/Slice_59_output_0_zero_point",  "/Slice_63_output_0_zero_point",
      "/Slice_67_output_0_zero_point",  "/Slice_71_output_0_zero_point",
      "/Slice_75_output_0_zero_point",  "/Slice_79_output_0_zero_point",
      "/Slice_83_output_0_zero_point",  "/Slice_87_output_0_zero_point",
      "/Slice_91_output_0_zero_point",  "/Slice_95_output_0_zero_point",
      "/Slice_99_output_0_zero_point",  "/Slice_103_output_0_zero_point",
      "/Slice_107_output_0_zero_point", "/Slice_111_output_0_zero_point",
      "/Slice_115_output_0_zero_point", "/Slice_119_output_0_zero_point",
      "/Slice_123_output_0_zero_point", "/Slice_127_output_0_zero_point",
      "/Slice_131_output_0_zero_point", "/Slice_135_output_0_zero_point",
      "/Slice_139_output_0_zero_point", "/Slice_143_output_0_zero_point"};

  const uint32_t conv0_lp[16] = {134283525, 33620739, 34406913, 3407881,
                                 7168,      25165840, 2097536,  1049344,
                                 8388864,   0,        1,        4028112085,
                                 2404101,   7168,     0,        0};
  const uint32_t conv1_lp[16] = {135268609, 33817347,  33751553, 786468,
                                 7174,      100663312, 2097536,  1051648,
                                 8388864,   6,         1,        3770259994,
                                 180544448, 0,         0,        0};
  const uint32_t conv0_c0[1024] = {
      828701906,  4294966931, 3036725065, 4294966758, 1329702793, 4294966290,
      3854170780, 4294966845, 418786763,  4294967255, 3467380613, 4294967162,
      858260151,  4294967149, 3580850455, 4294967268, 1135225986, 4294967284,
      390199833,  4294967278, 1215477353, 4294967291, 3932976142, 4294967211,
      9294935,    4294966892, 946527792,  4294967254, 3280510581, 4294966999,
      2389144018, 4294966956, 17491772,   4294967015, 1621061389, 4294967073,
      1022575739, 568,        3609372791, 4294967276, 4073953442, 4294967291,
      4272315509, 4294967261, 1445411797, 4294966959, 55453855,   4294967145,
      3559446985, 4294966821, 909975963,  4294967266, 2583522885, 4294967200,
      2684593971, 4294967227, 2486094904, 4294967025, 3144185857, 4294967217,
      3751997690, 4294967198, 4136688100, 4294967256, 3347768627, 28,
      4099645957, 4294967280, 217104561,  4294967278, 512400389,  4294967288,
      350667964,  4294967157, 2854840796, 442,        941082963,  3,
      1855727723, 4294966331, 1782573594, 237,        3719127122, 4294967267,
      4280011815, 4294966396, 1979414845, 4294966893, 3836726477, 4294967066,
      895959952,  4294967256, 2865163828, 4294967250, 3781884312, 4294966849,
      981656148,  4294967218, 3468232053, 4294967200, 1909764416, 4294967009,
      3451403346, 4294967200, 1297931227, 4294967097, 2686787165, 4294967291,
      1223189283, 4294967014, 3170140654, 4294967229, 3165938735, 4294967273,
      3868344822, 4294967263, 3112128573, 4294967119, 755851217,  4294967243,
      3548977640, 46,         466349626,  4294966797, 2254922957, 4294966946,
      2173501086, 4294967203, 2445483563, 4294966901, 2144629342, 290,
      153763620,  4294967269, 1226908700, 4294967276, 2687233916, 190,
      163643807,  4294966733, 1669985536, 80,         710207548,  5,
      3926921124, 4294967065, 611356094,  4294967102, 18252180,   24,
      1800998525, 4294967288, 844988924,  4294966856, 2126992758, 45,
      3280273236, 3,          3638613473, 4294967237, 3242869967, 4294967273,
      3470734998, 4294967227, 1575503345, 4294967276, 3494302819, 4294967268,
      1667100434, 4294967123, 1207331891, 4294967255, 1713646918, 4294967190,
      3399124814, 4294966832, 2593139289, 4294967200, 2576852271, 4294967275,
      3990902598, 4294967176, 811056009,  4294966951, 3390510069, 4294966984,
      3788403206, 4294967219, 88174204,   4294966962, 2683596218, 4294967222,
      2657903703, 4294967233, 1602305676, 4294967179, 1712645259, 4294967038,
      1558541544, 4294967191, 976984042,  4294966185, 3364709397, 4294967233,
      3542390246, 4294966856, 2461601736, 4294967242, 2830508661, 4294967245,
      3459248370, 4294967008, 468865790,  4294966706, 2316964206, 4294967281,
      1823431593, 4294967092, 3533250937, 4294966962, 246117211,  4294967274,
      1842732901, 4294967208, 1517899859, 4294967156, 1160425782, 4294966579,
      2097116950, 4294966866, 1770020713, 4294967193, 3362240702, 4294967264,
      2561058569, 4294966220, 1796049417, 4294966909, 3606063470, 4294966445,
      472090987,  4294966833, 4213680020, 4294967078, 2777044001, 4294966553,
      3959400222, 4294967035, 2044840823, 4294967204, 3835724818, 4294966914,
      1820898304, 4294967154, 3373886862, 36,         3119418689, 4294966970,
      2715799815, 4294967287, 1222651500, 4294967086, 2574221639, 4294966751,
      1881659988, 4294966726, 2853650762, 4294967267, 185206809,  4294967029,
      1909208007, 315,        1964722550, 4294967282, 2506705217, 4294966550,
      41244062,   4294967228, 303250510,  4294966317, 288720152,  4294967261,
      2441224862, 4294967270, 4178255132, 4294967033, 3749184994, 4294967208,
      275129861,  4294967270, 603687727,  4294967172, 2468582101, 4294967130,
      543925297,  4294967046, 2779729914, 15,         1273076933, 4294967264,
      1106768244, 4294967245, 2423455184, 4294966940, 2735260655, 4294967252,
      3738342805, 4294967238, 2490124072, 4294967250, 3177132737, 262,
      3772370658, 4294967023, 996588193,  4294967235, 2903062818, 4294966852,
      2843509794, 4294967221, 2400592490, 4294966970, 3435478955, 4294966766,
      4036059859, 4294967277, 2427501477, 4294967194, 417521321,  343,
      3257879825, 4294967141, 3811971027, 4294967260, 1313275773, 4294967251,
      1925294932, 4294966776, 1255860662, 154,        1078845880, 4294966428,
      3225533821, 4294967256, 507278530,  4294967178, 3987078930, 3,
      3512313748, 149,        3857842728, 4294967167, 1543906031, 4294967255,
      554474860,  4294967082, 2018636963, 4294967051, 3855131281, 614,
      1528057952, 4294967231, 2259429220, 4294967277, 3788716863, 33,
      2057811612, 4294967269, 3670048850, 4294966703, 3899922606, 4294966549,
      2463606555, 4294966987, 3266012068, 4294967295, 2282635915, 4294967268,
      4018012275, 4294966336, 273487669,  4294967016, 2237497280, 4294966637,
      3967056871, 4294966524, 1431549911, 4294967210, 4289305249, 4294966551,
      2512462202, 4294967174, 3262572058, 4294967085, 3336333374, 4294967192,
      1955657148, 4294967092, 1376436151, 4294967235, 855791456,  4294967180,
      3773243129, 4294967237, 1060630355, 4294967168, 2912833347, 4294967113,
      3444487575, 4294967281, 1264868378, 4294966700, 2100169396, 4294967262,
      1110352162, 4294966716, 2262655918, 4294966845, 2070860214, 4294967185,
      528292031,  4294967105, 1351294638, 4294967056, 1595975157, 4294967128,
      2033367414, 4294966867, 715342626,  4294967293, 754915653,  4294966501,
      1716056426, 4294966778, 525401522,  4294967264, 4275726676, 4294967001,
      687534730,  4294967091, 3620263950, 4294966627, 3584236685, 4294966685,
      2030824812, 4294967194, 2853650762, 4294967267, 2202513736, 4294967199,
      2634632414, 4294967273, 1390697319, 4294967239, 1043096521, 4294967097,
      680404146,  4294967089, 3215014602, 4294967131, 2083564815, 4294967080,
      3723406854, 4294967074, 1683653640, 4294967218, 3074557960, 4294966950,
      1800367305, 4294966921, 958286015,  4294967231, 2510023851, 4294967116,
      1402292104, 4294967220, 4248090030, 4294967089, 3262937090, 4294967282,
      1610318044, 4294967130, 3667398091, 4294967268, 4182137987, 4294967291,
      2864836952, 4294967258, 1388840314, 4294966902, 3167998835, 4294967252,
      1718046525, 4294967200, 1437536429, 4294967240, 458756667,  4294966012,
      2733356181, 4294966975, 208208004,  4294966672, 901377439,  4294966182,
      2481695297, 4294967015, 305123139,  4294967242, 1497971237, 4294966820,
      1796526512, 4294967015, 3760239591, 4294966555, 1605395374, 4294966515,
      898084646,  4294967204, 2931881686, 4294966941, 709838610,  4294966957,
      582432071,  4294966133, 2255029613, 4294967267, 3101510510, 4294966967,
      3888499071, 4294966858, 3806100478, 4294967286, 1210501807, 4294967148,
      1675572772, 4294967151, 1595815625, 4294967279, 4282486821, 4294967218,
      4207989130, 4294965864, 956390854,  4294966689, 1625073432, 4294967269,
      866521582,  4294967241, 1179369870, 4294966792, 3428651214, 402,
      716142691,  4294967244, 1091131072, 4294967157, 1129420031, 4294967279,
      4134976504, 4294965621, 25964110,   4294967043, 1378915660, 4294966380,
      123392091,  4294967218, 2725055093, 4294967237, 1715675770, 4294965993,
      2854068670, 4294966992, 164275027,  4294967100, 225572993,  4294967159,
      2722698461, 4294967177, 2465271279, 4294966858, 3554367188, 4294967063,
      3316003065, 4294965895, 847094992,  38,         3364726522, 4294967262,
      3307987695, 4294967062, 1831303055, 4294966664, 1890599204, 4294965860,
      920114526,  4294966606, 2265755833, 4294967181, 4137501384, 4294967089,
      1024620215, 4294967255, 132105680,  4294967093, 2694563689, 4294966983,
      1583950746, 4294966981, 1444330824, 219,        2450905860, 4294967239,
      1726446457, 4294966965, 3600573577, 4294967256, 3394358674, 4294967184,
      3125929771, 4294967046, 1416643400, 4294965692, 1788700114, 4294966677,
      4104252565, 4294967079, 3348707193, 4294966948, 1776875796, 4294967290,
      606388360,  4294967253, 1565014470, 4294967062, 2081018307, 4294967260,
      2360982808, 4294966998, 1235369320, 4294966863, 2359921962, 4294966465,
      979217797,  4294967160, 3759253556, 4294966991, 625096604,  4294967207,
      2196513999, 4294967287, 2983192809, 4294967215, 150933799,  4294967250,
      1683206889, 4294967023, 204886368,  4294967224, 945817258,  299,
      3424355858, 7,          1531984370, 4294967282, 1669018127, 4294967282,
      1598555915, 4294967006, 1085359367, 4294967210, 419801641,  4294967289,
      2190708044, 4294967282, 1037351254, 4294966914, 3334166618, 4294966892,
      2078253080, 4294967210, 1715306832, 4294965649, 3111444477, 4294967224,
      2927529548, 4294966871, 447734819,  4294967282, 1531915870, 4294967166,
      1938570065, 4294967216, 1270889146, 4294966788, 903027443,  4294966730,
      1415310362, 4294967225, 3743374229, 4294965791, 1460923687, 4294967256,
      2090385648, 4294967119, 4127884980, 4294967089, 162576053,  4294967171,
      1654327333, 4294967112, 2467679286, 4294967005, 808496282,  4294967249,
      3508809144, 4294967266, 3266992696, 4294967271, 2446669691, 4294967225,
      4151929896, 4294967236, 1736952457, 4294967208, 2505916870, 4294967040,
      3896359719, 4294967254, 2645630229, 4294966945, 168695665,  4294967286,
      1589403387, 4294967230, 2052921691, 4294967271, 1719831124, 4294967274,
      2746383752, 4294966715, 2556287022, 4294966984, 629324961,  4294966927,
      3368529159, 4294966963, 332067877,  4294966965, 633603192,  4294967293,
      4283098511, 4294966850, 2051941063, 4294967295, 3973389795, 4294967281,
      1517298983, 4294966700, 526705120,  4294967085, 4152777430, 4294967127,
      2098707767, 4294967033, 3759880870, 4294967211, 3845938192, 4294967223,
      4050889154, 4294967120, 2991528147, 4294967011, 462731458,  4294967268,
      1292006898, 4294966330, 355038728,  4294966697, 2699951736, 4294967263,
      555061613,  4294966391, 1713868639, 4294966302, 3097420654, 4294966920,
      3294233966, 4294967075, 599778434,  4294967150, 2968158014, 4294967024,
      430179954,  4294967035, 4284220045, 4294967205, 3850080924, 4294966798,
      472459925,  4294967177, 519776130,  4294967284, 1097706748, 4294967202,
      2855948207, 4294966946, 2129178140, 4294967111, 110734959,  4294967263,
      3931428888, 4294966837, 1179586184, 4294966316, 3905434434, 4294967179,
      2088424392, 4294967167, 1423890857, 4294967015, 1922721986, 4294967192,
      1661982481, 4294967160, 1713479574, 4294967047, 3257866606, 4294967259,
      3041343391, 4294966998, 2912101782, 4294967278, 2677480512, 4294967254,
      2080527993, 4294967272, 3814065377, 1,          798554503,  4294966698,
      3468786961, 4294967157, 1526109915, 4294967161, 2919933587, 4294967204,
      175482248,  4294967267, 4188384382, 4294966638, 681685212,  4294967293,
      3867494883, 4294966666, 3754010321, 4294967237, 602746756,  4294966842,
      1891567517, 4294967219, 2401461055, 4294967037, 1647459031, 4294967133,
      1134735672, 0,          1287807384, 4294967080, 1316041000, 5,
      3782958377, 4294967264, 3854568561, 363,        850574659,  4294967190,
      499150193,  4294967171, 1766008670, 4294966997, 19608654,   4294966669,
      933672972,  4294967245, 3485488885, 629,        2943858628, 4294967148,
      3576885881, 4294967012, 2067066890, 4294967219, 2389560425, 4294967240,
      2486094904, 4294967025, 2601349345, 4294967205, 2266053866, 4294966703,
      4133428653, 4294967071, 2258857187, 4294967291, 1079454568, 4294967178,
      3286750068, 21,         3336204186, 4294967254, 3472205940, 4294967191,
      2832023166, 4294967002, 1645760057, 4294967204, 950075959,  4294967226,
      2395822444, 4294967175, 3901034827, 4294967169, 918186019,  4294967271,
      4200454761, 4294967284, 3368236533, 4294967029, 4194648806, 4294967279,
      1841687679, 4294967263, 3263132373, 4294966630, 696303600,  4294967200,
      3821028617, 4294967156, 3297555602, 4294966523, 879243905,  4294965341,
      444935342,  4294967174, 1390697319, 4294967239, 380661242,  4294967129,
      1473890570, 4294967174, 3108427782, 4294966327, 2066447388, 4294967293,
      617475706,  4294967217, 1722708414, 4294967233, 2656622637, 4294967029,
      716142691,  4294967244, 2167256192, 1,          888911087,  4294967252,
      2715438689, 4294967237, 1341798109, 4294967259, 3648111503, 4294966475,
      3676391087, 4294967195, 4021188502, 4294967082, 76735045,   4294966683,
      1617262658, 223,        2288441870, 4294967273, 3213161503, 4294966941,
      2237740936, 4294967190, 3883522024, 4294967274, 3053626178, 4294967021,
      2340170901, 4294967272, 1164367824, 4294967218, 2616900892, 4294967148,
      1546490695, 4294967280, 2869118185, 4294966506, 1665238022, 4294967198,
      2109230892, 9,          3774795790, 4294967199, 2902072877, 4294967141,
      339202367,  4294967114, 4162940930, 4294966790, 390199833,  4294967278,
      3851779898, 4294966727, 3989254999, 38,         4223748582, 4294966861,
      2721244644, 4294967242, 212150046,  15};
  const uint32_t conv1_c0[1024] = {
      2497985837, 4294967295, 1993564875, 33,         4077305635, 4294967267,
      1309390200, 4294967235, 3795649057, 4294967295, 347363367,  4294967231,
      3679891431, 4294967200, 3254301220, 17,         1991948172, 20,
      856951668,  4294967256, 3947411251, 4294967170, 3692539548, 4294967286,
      586294781,  59,         1029123584, 4294966906, 35467617,   4294967220,
      3354113877, 4294967292, 2490282998, 4294967254, 1305331996, 4294967157,
      4222888864, 4294967286, 1485909337, 4294967248, 2527367318, 4294967191,
      938920656,  59,         2048492864, 4294967146, 342641320,  4294967289,
      956764325,  4294967235, 520429377,  86,         2095090574, 120,
      628545270,  4294967262, 482392197,  0,          468349438,  4294967211,
      1936002707, 4294967257, 3530949074, 4294967236, 1683601888, 4294967262,
      4023801671, 4294967273, 2040855811, 4294967287, 2388853638, 4294967262,
      3266882381, 4294967128, 3811148734, 4294967294, 2465147719, 4294967270,
      3153187920, 4294967252, 1319818877, 4294967270, 3356142979, 35,
      1263842859, 2,          931629046,  4294967251, 2287011879, 4294967244,
      1373831579, 89,         3703602685, 65,         3485338287, 4294967295,
      236516956,  4294967001, 2402071599, 4294967295, 2225012001, 4294967159,
      1455195490, 4294967268, 1751180334, 47,         904684386,  4294967064,
      2159687058, 13,         1734951028, 2,          1799009391, 4294967246,
      2758659339, 4294967278, 475323818,  4294967295, 3788928461, 4294967227,
      4149258345, 61,         1418014831, 4294967236, 2422040709, 13,
      1815302143, 4294967295, 1916349657, 4294967174, 2240606847, 4294967164,
      3802906604, 4294967219, 2584613426, 36,         2089828066, 55,
      3161999894, 4294967274, 1099432548, 41,         1496846752, 108,
      2374875495, 4294967270, 1434466198, 4294967295, 1783194824, 109,
      1942437796, 11,         1647818211, 111,        1301878869, 4294967295,
      3003517104, 35,         3478649414, 4294967229, 1739830520, 4294967161,
      3272557288, 4294967219, 3758943073, 4294967204, 2308694031, 70,
      4045165423, 4294967286, 3707660889, 143,        3190877317, 109,
      1929123496, 4294967179, 2896825236, 4294967274, 2907350252, 108,
      1836698788, 103,        2422040709, 13,         4285839262, 46,
      2488473617, 4294967051, 3003420765, 4294967236, 2420771789, 4294967229,
      616280169,  82,         971598989,  4294967281, 2510252108, 4294967268,
      1682714814, 4294967295, 145516273,  4294967044, 978226756,  225,
      4100824211, 4294967238, 1282638218, 4294967238, 3653648187, 28,
      1811783230, 4294967251, 1749563631, 34,         1587435036, 39,
      2053948050, 105,        1002852127, 4294967295, 539670028,  143,
      1422891983, 4294966921, 2029698675, 4294967295, 637357244,  4294967284,
      152556439,  14,         1913501607, 232,        885416692,  65,
      3110428693, 4294967224, 1649242236, 4294967230, 1457128253, 4294967212,
      4237817527, 4294967249, 664521625,  11,         2485625567, 109,
      961614434,  4294967274, 1784271066, 4294967295, 329423359,  4294967256,
      1457034254, 4294967295, 2326537700, 4294967246, 1729910581, 4294967247,
      698372539,  4294967218, 3395352740, 106,        2152269726, 4294967286,
      2359756494, 4294967295, 3222984636, 4294967295, 2570538944, 4294967245,
      2411643755, 4294967276, 4043358382, 4294967261, 4264981908, 4294967272,
      662299845,  4294967078, 3210530367, 192,        1971883893, 0,
      2020124179, 4294967136, 983105078,  4294967295, 3311989110, 8,
      2511456412, 4294967255, 1677263138, 11,         3318898874, 4294967295,
      1483881405, 2,          2973119317, 4294967282, 2966272999, 4294967295,
      1471518795, 4294967230, 3013849442, 4294967271, 4169730343, 4294967047,
      1699640856, 4294967295, 585882382,  33,         737678639,  88,
      690861208,  4294967278, 4105075093, 210,        3900026316, 4294967295,
      3810198214, 27,         2860945220, 28,         3364954953, 57,
      4100920550, 37,         4084500906, 4294967276, 2737771432, 4294967295,
      2151001976, 4294967295, 686011099,  4294967239, 1785475370, 4294967282,
      920691631,  4294967095, 388093492,  4294967220, 3663884186, 4294967169,
      3068873770, 4294967183, 3207326344, 4294967286, 130114105,  4294967229,
      4284222559, 33,         1567781986, 4294967252, 1581952807, 138,
      2766711131, 45,         2073503591, 4,          3168782766, 4294967257,
      890011847,  4294967295, 10937415,   156,        812259678,  4294967284,
      2921135717, 4294967206, 2844969698, 4294967295, 980188902,  4294967289,
      3203237587, 4294967295, 4116419057, 4294967243, 947636291,  4294967282,
      3783665953, 4294967162, 2053851711, 10,         1583376832, 4294967257,
      3623346739, 74,         2926778901, 4294967295, 937111275,  4294967152,
      883799989,  52,         1059906727, 39,         2511360073, 4294967160,
      337157921,  3,          4062722415, 4294967059, 978794260,  4294967290,
      3803319003, 4294967245, 2638021051, 4294967231, 16323305,   4294967258,
      1839169672, 4294967288, 1271481082, 4294967246, 2883291215, 14,
      929505945,  4294967295, 2264318101, 29,         471995243,  4294967263,
      2743096216, 4294967275, 819042550,  4294967267, 2691937414, 4294967251,
      486607846,  4294967295, 2838155103, 14,         1640082479, 4294967275,
      356271680,  52,         2093281193, 4294967213, 863734540,  4294967239,
      478017933,  4294967287, 2362197995, 64,         215439881,  4294967095,
      1820595204, 4294967273, 2576974033, 4294967295, 1374939544, 4294967277,
      4246945561, 4294967202, 1117875444, 4294966988, 2402071599, 4294967295,
      1177754561, 4294967295, 669591455,  4294967182, 719037215,  4294967098,
      1547716537, 4294967143, 2975148419, 25,         3370024783, 4294967228,
      3967730484, 4294967295, 3676850703, 68,         2733524060, 4294967294,
      2477509159, 4294967249, 3692665270, 4294967205, 2679163575, 4294967246,
      3034423629, 205,        658846718,  4294967216, 1210086281, 81,
      2412056154, 6,          1113603369, 223,        732416131,  23,
      771561276,  1,          473804624,  170,        3044247229, 24,
      1888077311, 4294967259, 3050998378, 5,          2388757299, 4294967167,
      4215123749, 34,         3713808131, 4294967293, 4202349910, 29,
      1460457998, 37,         181588967,  4294967184, 857048007,  55,
      3418266239, 4294967157, 3322955908, 4294967284, 3607146816, 4294967149,
      3128052641, 4294967268, 992328281,  4294967254, 1551362342, 4294967195,
      153000561,  42,         542996263,  4294967293, 185331111,  35,
      4277536026, 145,        3120857370, 4294967259, 10332338,   4294967236,
      3826521519, 4294967285, 4099207508, 4294967225, 653015536,  4294967293,
      1553075384, 7,          1804298942, 4294966957, 2151001976, 4294967295,
      167803502,  86,         915400910,  4294967295, 292339039,  23,
      437256085,  0,          2653268114, 7,          1254269533, 4294967228,
      1038636974, 4294967239, 2616558620, 4294966945, 830199686,  4294967259,
      1089506759, 4294966978, 1375255604, 4294967208, 1668895286, 17,
      3719224574, 4294967012, 3040917484, 4294967199, 1809660129, 4294967295,
      566641731,  4294967272, 4099303847, 24,         1836313432, 4294967019,
      789153501,  43,         4132616640, 4294967286, 2036671885, 4294967290,
      2101996828, 4294967140, 4050934329, 4294967294, 846206931,  4294967290,
      1255982575, 40,         1613581941, 4294967116, 67674785,   176,
      3423528747, 4294967222, 2374875495, 4294967270, 3423403025, 7,
      546576282,  4294967163, 2986115217, 5,          940757080,  4294967204,
      1590572103, 4294967266, 3990775555, 118,        3478649414, 4294967229,
      4141966735, 4294967253, 3726516184, 4294967116, 2655864720, 4294967111,
      3286535431, 4294967211, 1266411252, 75,         904464665,  4294967228,
      1063359854, 4294967197, 472629703,  7,          4127672532, 34,
      920691631,  4294967095, 2003897213, 4294967269, 852989803,  4294967273,
      2428887027, 0,          3452502509, 152,        2305048226, 18,
      2534658928, 4294967295, 1699512794, 4294967198, 1668451164, 4294967285,
      1952422351, 18,         3197371172, 4294967103, 3286535431, 4294967211,
      3206691884, 4294967246, 3808774189, 204,        2120225853, 104,
      2333732971, 4294967255, 707088174,  4294967145, 3420488019, 90,
      1002949636, 183,        2899836581, 4294967286, 4113378329, 111,
      3522013718, 177,        3678879805, 107,        3502360668, 94,
      274811430,  74,         1934386004, 4294967244, 1186375027, 4294967216,
      3128052641, 4294967268, 3009095672, 31,         2465147719, 4294967270,
      1830579759, 4294967280, 3094105388, 4294967262, 2527367318, 4294967191,
      2391990705, 4294967193, 2066309490, 84,         1163076172, 4294967081,
      2699229024, 59,         2702874829, 111,        3472278941, 4294967272,
      2047829021, 4294967282, 2572155647, 4294967258, 3733395395, 4294967194,
      4181936678, 4294967283, 2011408544, 4294967209, 2718689396, 4294967248,
      1576277900, 47,         2194394493, 4294967274, 696439776,  4294967274,
      2710380310, 4294966902, 2369296927, 4294967274, 265587057,  26,
      2808613837, 19,         946431987,  4294967295, 4284538619, 4294967260,
      2483500126, 4294967271, 1482263532, 4294967196, 1249547486, 4294967286,
      3393227299, 4294967268, 1277375710, 4294967173, 3564194911, 4294966931,
      1765378198, 4294967171, 3131698446, 24,         1430472610, 14,
      45896294,   4294967255, 1598084604, 4294967295, 2884051397, 4294967269,
      710733979,  4294967197, 2538620793, 4294967278, 3137180675, 4294967221,
      1018983924, 4294967156, 1369583037, 4294967295, 3801798639, 31,
      1579700474, 4294967292, 1310814225, 4294967058, 1838979334, 4294967276,
      248878396,  4294966980, 4196232051, 4294967295, 2435923683, 4294967295,
      2855682712, 4294967259, 1453482448, 4294967160, 1875619532, 4294967185,
      2077466626, 76,         1791466337, 8,          3310150346, 4294967277,
      3673617297, 42,         1843605042, 4294967123, 2458364847, 4294967287,
      2606006561, 4294967169, 2985480757, 4294967261, 194046746,  4294967258,
      1145741241, 26,         2456432084, 47,         2299978396, 143,
      2406793646, 4294967237, 279565200,  18,         2686455185, 54,
      1783510884, 40,         1594978090, 4294967277, 3087322516, 4294967279,
      1629115681, 4294967295, 2898441939, 4294967287, 1116863818, 4294967191,
      4193537936, 7,          4138004870, 4294967270, 2211986718, 20,
      755522308,  4294967264, 870582028,  19,         3949633031, 103,
      514976531,  9,          1790293756, 23,         3691241245, 86,
      1081588879, 161,        1487526040, 4294967261, 788960823,  4294967149,
      3787697114, 298,        2926617946, 4294967107, 3999078791, 19,
      1856571559, 22,         2433070953, 4294967293, 3991883520, 10,
      1493961129, 15,         51887261,   4294967277, 2121333818, 4294967292,
      2746234453, 4294967295, 267519820,  4294967266, 455576769,  4294967295,
      1238668837, 4294966778, 4173376148, 4294967099, 2848583780, 49,
      965672638,  56,         283114666,  4294967271, 4178445978, 4294966974,
      1525119098, 23,         3011250496, 4294967285, 1581760129, 4294967244,
      29792710,   4294967129, 3682113211, 133,        1724235674, 4294967156,
      2769944537, 71,         2340515843, 4294967238, 3995845385, 4294967289,
      3489806550, 4294967221, 3537828285, 18,         1877744973, 23,
      2702462430, 85,         1304699876, 4294967295, 562775035,  4294967295,
      1111381589, 4294967290, 274811430,  74,         3623566460, 4294967206,
      1968967717, 4294967290, 1526639462, 4294967237, 938824317,  4294967260,
      1471518795, 4294967230, 2695392881, 4294967291, 4266820672, 3,
      2401118739, 4294967146, 3167166063, 4294967244, 2476939315, 6,
      495197759,  7,          2969094006, 4294967295, 3571679199, 4294967225,
      2894603456, 4294967045, 3350179055, 4294966955, 4206121437, 0,
      2849217070, 0,          3940724718, 4294967282, 2173317418, 72,
      2395636510, 4294967245, 4025830773, 16,         3676658025, 4294967174,
      2921136887, 4294967295, 1840151915, 4294967261, 2053948050, 105,
      2364034419, 4294967209, 231639804,  20,         2974639681, 4294967200,
      1107735784, 4294967238, 2126912386, 4294967288, 915718140,  19,
      1590283086, 4294966981, 821387712,  4294967237, 864781399,  9,
      3382798622, 4294967233, 616280169,  82,         3249451111, 4294967274,
      2637009425, 138,        3109004668, 105,        4229514291, 52,
      2328725417, 4294967295, 3973086991, 4294967277};
};

} // namespace vaip_vaiml_custom_op