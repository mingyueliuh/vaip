#include "custom_op_gt_1_3.hpp"
#include "../../common/bf16_utils.h"
#include "constants_gt_1_3.hpp"
#include "txn_pkg_gt_1_3.hpp"
#define GT_1_3_TF_NUM 36
// #define GT_FRONT_SZ 16480 + 552960
#define GT_FRONT_SZ 0

#include <vaip/util.hpp>
#include <vaip/vaip.hpp>

namespace vaip_vaiml_custom_op {
constexpr unsigned conv_ifm_size = 16480;
constexpr unsigned conv_ofm_size = 552960;
constexpr unsigned conv_wts_size = 2501056;
constexpr unsigned linear_out_tmp_size = 2129920;
constexpr unsigned linear_out_ofm_size = 25600;
constexpr unsigned linear_out_wts_size = 5418432;
constexpr unsigned transformer_io_size =
    4300800;  // unified size for 3 in 1 transformer block
constexpr unsigned transformer_wts_size =
    11608064; // this is the same as gt1.2, but there are holes internally
constexpr unsigned transformer_tmp_size =
    102400 + 576000 + 499200; // same as 1.2
constexpr unsigned rtp_skip_size = 1472;
extern size_t GT_SUB_MUL_WTS_convert(
    const std::unordered_map<std::string, flexmlrt::client::ErtIoTypeNew>& wts_,
    int8_t*& wts_ptr, const std::string& sub_i_scale_name,
    const std::string& sub_i_zp_name, const std::string& sub_w_scale_name,
    const std::string& sub_w_zp_name, const std::string& sub_o_scale_name,
    const std::string& sub_o_zp_name, const std::string& sub_w_name,
    const std::string& mul_i_scale_name, const std::string& mul_i_zp_name,
    const std::string& mul_w_scale_name, const std::string& mul_w_zp_name,
    const std::string& mul_o_scale_name, const std::string& mul_o_zp_name,
    const std::string& mul_w_name, std::string model_name);

extern size_t GT_CONV_WTS_convert(
    const std::unordered_map<std::string, flexmlrt::client::ErtIoTypeNew>& wts_,
    int8_t*& wts_ptr, const std::string& conv_i_scale_name,
    const std::string& conv_i_zp_name, const std::string& conv_w_scale_name,
    const std::string& conv_w_zp_name, const std::string& conv_b_scale_name,
    const std::string& conv_b_zp_name, const std::string& conv_o_scale_name,
    const std::string& conv_o_zp_name, const std::string& conv_w_name,
    const std::string& conv_b_name, int32_t oc, int32_t ic, int32_t h,
    int32_t w, int32_t sv_ic, uint32_t* c0, uint32_t* lp,
    std::string model_name);

size_t GT_MMB_WTS_convert_ptr(
    const std::unordered_map<std::string, flexmlrt::client::ErtIoTypeNew>& wts_,
    int8_t*& wts_ptr, int8_t*& rtp_ptr, const std::string& mmb_i_scale_name,
    const std::string& mmb_i_zp_name, const std::string& mmb_w_scale_name,
    const std::string& mmb_w_zp_name, const std::string& mmb_b_scale_name,
    const std::string& mmb_b_zp_name, const std::string& mmb_o_scale_name,
    const std::string& mmb_o_zp_name, const std::string& mmb_w_name,
    const std::string& bias_name, uint32_t gemm_m, uint32_t gemm_k,
    uint32_t gemm_n, uint32_t bias_broadcast_num, uint32_t sv_M, uint32_t sv_K,
    uint32_t sv_N, bool pad_w, std::string model_name);

size_t GT_LN_WTS_convert(
    const std::unordered_map<std::string, flexmlrt::client::ErtIoTypeNew>& wts_,
    int8_t*& wts_ptr, const std::string& ln_i_s_name,
    const std::string& ln_i_zp_name, const std::string& ln_scale_s_name,
    const std::string& ln_scale_zp_name, const std::string& ln_bias_s_name,
    const std::string& ln_bias_zp_name, const std::string& ln_o_s_name,
    const std::string& ln_o_zp_name, const std::string& ln_scale_name,
    const std::string& ln_bias_name, std::string model_name);

size_t GT_MMB_WTS_convert_raw_ptr(
    const std::unordered_map<std::string, flexmlrt::client::ErtIoTypeNew>& wts_,
    int8_t*& wts_ptr, int8_t*& rtp_ptr, int8_t* mmb_w_ptr,
    const std::string& mmb_i_scale_name, const std::string& mmb_i_zp_name,
    const std::string& mmb_w_scale_name, const std::string& mmb_w_zp_name,
    const std::string& mmb_b_scale_name, const std::string& mmb_b_zp_name,
    const std::string& mmb_o_scale_name, const std::string& mmb_o_zp_name,
    const std::string& bias_name, uint32_t gemm_m, uint32_t gemm_k,
    uint32_t gemm_n, uint32_t bias_broadcast_num, uint32_t sv_M, uint32_t sv_K,
    uint32_t sv_N, const std::string model_version);

size_t GT_MUL_WTS_convert(
    const std::unordered_map<std::string, flexmlrt::client::ErtIoTypeNew>& wts_,
    int8_t*& wts_ptr, int8_t*& rtp_ptr, const std::string& mul_i_scale_name,
    const std::string& mul_i_zp_name, const std::string& mul_w_scale_name,
    const std::string& mul_w_zp_name, const std::string& mul_o_scale_name,
    const std::string& mul_o_zp_name, const std::string& mul_w_name,
    const std::string model_version);

size_t GT_BMM_WTS_convert(
    const std::unordered_map<std::string, flexmlrt::client::ErtIoTypeNew>& wts_,
    int8_t*& wts_ptr, int8_t*& rtp_ptr, const std::string& mm_i_scale_name,
    const std::string& mm_i_zp_name, const std::string& mm_w_scale_name,
    const std::string& mm_w_zp_name, const std::string& mm_o_scale_name,
    const std::string& mm_o_zp_name, const std::string& mm_w_name,
    bool transpose_b, uint32_t gemm_M, uint32_t gemm_K, uint32_t gemm_N,
    uint32_t sv_M, uint32_t sv_K, uint32_t sv_N,
    const std::string model_version);

size_t GT_QDQ_convert(
    const std::unordered_map<std::string, flexmlrt::client::ErtIoTypeNew>& wts_,
    int8_t*& wts_ptr, int8_t*& rtp_ptr, const std::string& i_s_name,
    const std::string& i_zp_name, const std::string& o_s_name,
    const std::string& o_zp_name, const std::string model_version);

size_t GT_ADD_WTS_QDQ_convert(
    const std::unordered_map<std::string, flexmlrt::client::ErtIoTypeNew>& wts_,
    int8_t*& wts_ptr, int8_t*& rtp_ptr, const std::string& add_i_scale_name,
    const std::string& add_i_zp_name, const std::string& add_w_scale_name,
    const std::string& add_w_zp_name, const std::string& add_o_scale_name,
    const std::string& add_o_zp_name, const std::string& add_w_name,
    const int32_t tensor_size, const std::string model_version);

size_t GT_MUL_WTS_QDQ_convert(
    const std::unordered_map<std::string, flexmlrt::client::ErtIoTypeNew>& wts_,
    int8_t*& wts_ptr, int8_t*& rtp_ptr, const std::string& mul_i_scale_name,
    const std::string& mul_i_zp_name, const std::string& mul_w_scale_name,
    const std::string& mul_w_zp_name, const std::string& mul_o_scale_name,
    const std::string& mul_o_zp_name, const std::string& mul_w_name,
    const int32_t tensor_size, const std::string model_version);

size_t GT_SOFTMAX_WTS_convert(
    const std::unordered_map<std::string, flexmlrt::client::ErtIoTypeNew>& wts_,
    int8_t*& wts_ptr, const std::string& sm_i_scale_name,
    const std::string& sm_i_zp_name, const std::string& sm_o_scale_name,
    const std::string& sm_o_zp_name, uint32_t K, uint32_t K_valid,
    const std::string model_version);

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
std::map<std::string, std::vector<char>> MyCustomOpGT1_3::node_cache;

SUBGRAPH_ID
MyCustomOpGT1_3::IdentifySubgraphVector(
    const std::shared_ptr<MetaDefProto>& meta_def) {
  // EPContext based subgraphs
  if (meta_def->nodes().size() == 1) {
    // All nodes in a subgraph becomes a EPContext node
    VAIML_DEBUG_PRINT("    EPContext model_version_=", model_version_,
                      " sg_name_=", sg_name_);
    std::map<std::string, SUBGRAPH_ID> pattern_to_id = {
        {"vaiml_par_0", GT_FRONT},
        {"vaiml_par_1", GT_FRONT_MM},
        {"vaiml_par_2", GT_TRANSFORMER_BLOCK},
        {"vaiml_par_3", GT_LN_MATMUL_ADD_LN},
        {"vaiml_par_4", GT_CACHE_FRAMES_SLICE}};
    return pattern_to_id.at(sg_name_);
  } else {
    std::vector<std::pair<std::string, SUBGRAPH_ID>> v_pattern_to_id = {
        {"/conv/conv.0/Conv", GT_FRONT},
        {"/Reshape_2_output_0_DequantizeLinear", GT_FRONT_MM},
        {"/lin_enc/fc/MatMul", GT_LN_MATMUL_ADD_LN},
        {"/Unsqueeze_26_output_0_QuantizeLinear", GT_TRANSFORMER_BLOCK},
        {"oup_cache_frames", GT_CACHE_FRAMES_SLICE}};
    std::vector<SUBGRAPH_ID> v_res;
    for (const auto& node : meta_def->nodes()) {
      for (const auto& pattern_to_id : v_pattern_to_id) {
        if (node.rfind(pattern_to_id.first, 0) != std::string::npos) {
          return pattern_to_id.second;
        }
      }
    }
  }

  return SUBGRAPH_ID::UNKNOWN;
}

std::vector<std::vector<uint8_t>> SplitTransformerHeadMMWts(
    const std::string& mm_wts_name,
    const std::unordered_map<std::string, flexmlrt::client::ErtIoTypeNew>&
        wts_) {
  // split 512x1536 to 3 512x512 (i.e.
  // 512x1536--reshape-->512x3x512--transpose(1, 0, 2)-->3x512x512)
  uint8_t* w_ptr_orig = (uint8_t*)(wts_.at(mm_wts_name).data);
  std::vector<std::vector<uint8_t>> res(3);
  for (int i = 0; i < 3; ++i) {
    res[i].resize(512 * 512);
    for (int j = 0; j < 512; ++j) {
      memcpy(res[i].data() + j * 512, w_ptr_orig + j * 1536 + i * 512, 512);
    }
  }
  return res;
}

MyCustomOpGT1_3::MyCustomOpGT1_3(std::shared_ptr<const PassContext> context,
                                 const std::shared_ptr<MetaDefProto>& meta_def,
                                 onnxruntime::Model* model)
    : CustomOpImp(context, meta_def, model),
      model_version_(
          context->get_config_proto().provider_options().at("model_name")) {
  auto& session_option = context->get_config_proto().provider_options();
  sg_name_ = meta_def->vaiml_param().vaiml_model_path();
  LoadConstantsToWts(context, meta_def);
  subgraph_id_ = IdentifySubgraphVector(meta_def);
  // transformer_block_id_ = subgraph_id_ == SUBGRAPH_ID::GT_TRANSFORMER_BLOCK ?
  // GetTransformerBlockId(meta_def) : -1;

  std::vector<std::string> v_txn_bins;
  std::vector<std::string> v_ctrl_pkt_bins;
  std::vector<XRTRunOffset> xrt_offset;
  std::vector<KERNEL_NM> v_kernel_indices;
  std::vector<BO_ORDER> v_bo_order;
  size_t wts_size, ifm_size, ofm_size, tmp_size;
  // setting output_shapes to be used by getting output from onnx
  for (auto& vaiml_shapes : meta_def->vaiml_param().output_shapes()) {
    ort_output_shapes_.emplace_back(vaiml_shapes.shapes().begin(),
                                    vaiml_shapes.shapes().end());
  }
  PrepareHwRunner(v_txn_bins, v_ctrl_pkt_bins, xrt_offset, v_kernel_indices,
                  v_bo_order, ifm_size, ofm_size, wts_size, tmp_size);
  g.set_bo_order_vec(v_bo_order);

  if (subgraph_id_ < GT_CPU_OR_CONSTANT) {
    std::string xclbinFileName =
        get_xclbin_fullpath(context, session_option.at("xclbin"));
    auto read_xclbin = context->read_xclbin(xclbinFileName);
    auto xclbin = std::vector<char>(read_xclbin.value().begin(),
                                    read_xclbin.value().end());
    g.load_xclbin(xclbin);
    VAIML_DEBUG_PRINT("load xclbin done at ", xclbinFileName);
    g.load_txn_bin(v_txn_bins);
    VAIML_DEBUG_PRINT("load txn done");
    g.load_ctrl_pkt_bin(v_ctrl_pkt_bins);
    VAIML_DEBUG_PRINT("load ctrl pkt done");
    g.hw_runner_init(ifm_size, wts_size, ofm_size, tmp_size, true /*gt_mode*/,
                     xrt_offset, v_kernel_indices);
    VAIML_DEBUG_PRINT("hw runner init done");
    g.get_bo_ptrs(ifm_ptr_, wts_ptr_, ofm_ptr_);
  }

  VAIML_DEBUG_PRINT("Begin wts format for ", model_version_);
  InitWeights();
}

void MyCustomOpGT1_3::PrepareHwRunner(std::vector<std::string>& v_txn_bins,
                                      std::vector<std::string>& v_ctrl_pkt_bins,
                                      std::vector<XRTRunOffset>& xrt_offset,
                                      std::vector<KERNEL_NM>& v_kernel_indices,
                                      std::vector<BO_ORDER>& v_bo_order,
                                      size_t& ifm_size, size_t& ofm_size,
                                      size_t& wts_size, size_t& tmp_size) {
  if (subgraph_id_ == GT_FRONT) {
    ifm_size = 16480 /*conv input*/ + 552960 /*conv output*/;
    wts_size = 2501056 /*conv weights*/;
    ofm_size = 0;
    tmp_size = 2129920;
    /// xrt_offset.push_back({.ifm_offset = 0, .ofm_offset = 16480, .wts_offset
    /// = 0, .tmp_offset=0});
    xrt_offset.emplace_back(0, 0, 16480, 0);
    v_txn_bins.push_back(getBins_front_sub_ml_txn_1_3(model_version_));
    v_ctrl_pkt_bins.push_back(
        getBins_out_matmul_bias_ctrl_pkt_1_3(model_version_));
    v_kernel_indices.push_back(KERNEL_NM::GT_CONV);
    v_bo_order.push_back(BO_ORDER::ODR_GT_CONV);
  } else if (subgraph_id_ == GT_FRONT_MM) {
    ifm_size = 16480 /*conv input*/ + 552960 /*conv transpose reshape output*/ +
               25600 /*mm and add output*/;
    wts_size = 2501056 /*conv weights*/ + 5418432 /*mm and add weight */;
    ofm_size = 0;
    tmp_size = 2129920;
    xrt_offset.emplace_back(16480 /*input offset*/, 2501056 /*wts offset*/,
                            16480 + 552960 /*ofm offset*/, 0);
    v_txn_bins.push_back(getBins_out_matmul_bias_ml_txn_1_3(model_version_));
    v_ctrl_pkt_bins.push_back(
        getBins_out_matmul_bias_ctrl_pkt_1_3(model_version_));
    v_kernel_indices.push_back(KERNEL_NM::GT_MM);
    v_bo_order.push_back(BO_ORDER::ODR_GT_HEAD);
  } else if (subgraph_id_ == GT_LN_MATMUL_ADD_LN) {
    ifm_size = 25600 /*first ln input*/ + 25600 /*last ln output*/;
    wts_size = 590400;
    ofm_size = 0;
    tmp_size = 51200;
    xrt_offset.emplace_back(0 /*input offset*/, 0 /*wts offset*/,
                            25600 /*ofm offset*/, 0);
    v_txn_bins.push_back(getBins_ln_matmul_bias_ln_ml_txn_1_3(model_version_));
    v_ctrl_pkt_bins.push_back(
        getBins_ln_matmul_bias_ln_ctrl_pkt_1_3(model_version_));
    v_kernel_indices.push_back(KERNEL_NM::GT_MM);
    v_bo_order.push_back(BO_ORDER::ODR_GT_TAIL);
  } else if (subgraph_id_ == GT_TRANSFORMER_BLOCK) {
    ifm_size = 4300800 * GT_1_3_TF_NUM + 25600;
    wts_size = 11608064 * GT_1_3_TF_NUM;
    ofm_size = 0;
    tmp_size = 1536000;
    for (int i = 0; i < GT_1_3_TF_NUM; i++) {
      size_t inout_base_ddr = i * 4300800 + GT_FRONT_SZ;
      size_t wts_base_ddr = i * 11608064;
      xrt_offset.emplace_back(inout_base_ddr /*input offset*/,
                              wts_base_ddr /*wts offset*/,
                              inout_base_ddr /*ofm offset*/, 0);
      v_txn_bins.push_back(
          getBins_transformer_layers_ml_txn_1_3(model_version_));
      v_ctrl_pkt_bins.push_back(
          getBins_transformer_layers_ctrl_pkt_1_3(model_version_));
      v_kernel_indices.push_back(KERNEL_NM::GT_MM);
      v_bo_order.push_back(BO_ORDER::ODR_GT_TRANSFORMER);
    }
  }
}

void MyCustomOpGT1_3::LoadConstantsToWts(
    std::shared_ptr<const PassContext> context,
    const std::shared_ptr<MetaDefProto>& meta_def) {
  std::vector<char> wts_file;
  auto wts_file_opt = context->read_file_c8("wts.bin");
  wts_file = wts_file_opt.value();
  auto const_info = meta_def->vaiml_param().const_data_info();
  wts_buffers_.resize(const_info.size());
  VAIML_DEBUG_PRINT("const_info.size(): ", const_info.size());
  int wts_vec_idx = 0;
  for (auto it = const_info.begin(); it != const_info.end(); ++it) {
    flexmlrt::client::ErtIoTypeNew wts_vec;
    auto const_name = it->first;
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

    for (auto dim : const_data_info.shape()) {
      wts_vec.shape.push_back(dim);
    }
    size_t rd_offset = const_data_info.offset();
    if (rd_offset >= wts_file.size()) {
      throw std::runtime_error(
          "Failed to seek to the specified position in the wts file.");
    }
    wts_buffers_[wts_vec_idx].resize(wts_vec.size);
    if (rd_offset + wts_vec.size > wts_file.size()) {
      throw std::runtime_error("Failed to read the specified amount of data.");
    }
    std::memcpy(wts_buffers_[wts_vec_idx].data(), wts_file.data() + rd_offset,
                wts_vec.size);
    wts_vec.data = wts_buffers_[wts_vec_idx].data();
    wts_[wts_vec.name] = wts_vec;
    wts_vec_idx++;
    VAIML_DEBUG_PRINT("const name loaded: ", wts_vec.name);
  }
  VAIML_DEBUG_PRINT("Total constants loaded: ", wts_vec_idx);
}

void to_file(std::string f_name, size_t size, int8_t* ptr) {
  std::ofstream fs(f_name, std::ios::out | std::ios::binary);
  fs.write(reinterpret_cast<char*>(ptr), size);
  fs.close();
}

void MyCustomOpGT1_3::InitWeights() {
  // init conv part
  std::string ifm_s_name, ifm_z_name, wts_s_name, wts_z_name, ofm_s_name,
      ofm_z_name, wts_w_name;
  size_t total_wts_bytes = 0;

  if (subgraph_id_ == SUBGRAPH_ID::GT_FRONT) {
    int8_t* wts_ptr_front = wts_ptr_;
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
        wts_s_name1, wts_z_name1, ofm_s_name1, ofm_z_name1, wts_w_name1,
        model_version_);

    std::string bias_s_name, bias_z_name, bias_w_name;
    ifm_s_name = "/encoder_embedding/Mul_output_0_scale";
    ifm_z_name = "/encoder_embedding/Mul_output_0_zero_point";
    wts_s_name = "encoder.embed.conv.0.weight_scale";
    wts_z_name = "encoder.embed.conv.0.weight_zero_point";
    bias_s_name = "encoder.embed.conv.0.bias_quantized_scale";
    bias_z_name = "encoder.embed.conv.0.bias_quantized_zero_point";
    ofm_s_name = "/conv/conv.1/Relu_output_0_scale";
    ofm_z_name = "/conv/conv.1/Relu_output_0_zero_point";
    wts_w_name = "encoder.embed.conv.0.weight_quantized";
    bias_w_name = "encoder.embed.conv.0.bias_quantized";
    total_wts_bytes += GT_CONV_WTS_convert(
        wts_, wts_ptr_front, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        bias_s_name, bias_z_name, ofm_s_name, ofm_z_name, wts_w_name,
        bias_w_name, 512, 1, 3, 3, 8, const_cast<uint32_t*>(conv0_c0),
        const_cast<uint32_t*>(conv0_lp), model_version_);

    ifm_s_name = "/conv/conv.1/Relu_output_0_scale";
    ifm_z_name = "/conv/conv.1/Relu_output_0_zero_point";
    wts_s_name = "encoder.embed.conv.2.weight_scale";
    wts_z_name = "encoder.embed.conv.2.weight_zero_point";
    bias_s_name = "encoder.embed.conv.2.bias_quantized_scale";
    bias_z_name = "encoder.embed.conv.2.bias_quantized_zero_point";
    ofm_s_name = "/conv/conv.3/Relu_output_0_scale";
    ofm_z_name = "/conv/conv.3/Relu_output_0_zero_point";
    wts_w_name = "encoder.embed.conv.2.weight_quantized";
    bias_w_name = "encoder.embed.conv.2.bias_quantized";
    total_wts_bytes += GT_CONV_WTS_convert(
        wts_, wts_ptr_front, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        bias_s_name, bias_z_name, ofm_s_name, ofm_z_name, wts_w_name,
        bias_w_name, 512, 512, 3, 3, 32, const_cast<uint32_t*>(conv1_c0),
        const_cast<uint32_t*>(conv1_lp), model_version_);
    VAIML_DEBUG_PRINT(
        "Finish to format WTS for front subgraph CONV, total bytes: ",
        total_wts_bytes);
  } else if (subgraph_id_ == SUBGRAPH_ID::GT_FRONT_MM) {
    std::string i_s_name = "/conv/conv.3/Relu_output_0_scale";
    std::string i_z_name = "/conv/conv.3/Relu_output_0_zero_point";
    std::string w_s_name = "/out/Transpose_output_0_scale";
    std::string w_z_name = "/out/Transpose_output_0_zero_point";
    std::string b_s_name = "encoder.embed.out.bias_scale";
    std::string b_z_name = "encoder.embed.out.bias_zero_point";
    std::string o_s_name = "/out/Add_output_0_scale";
    std::string o_z_name = "/out/Add_output_0_zero_point";
    std::string w_name = "/out/Transpose_output_0_quantized";
    std::string b_name = "encoder.embed.out.bias_quantized";
    int8_t* wts_ptr_linear_out =
        (int8_t*)(wts_ptr_ + 2501056 + 1472); // leave space for rtp
    int8_t* rtp_ptr =
        wts_ptr_ + 2501056 + gt_global_rtp_offset_.at(subgraph_id_);
    total_wts_bytes += GT_MMB_WTS_convert_ptr(
        wts_, wts_ptr_linear_out, rtp_ptr, i_s_name, i_z_name, w_s_name,
        w_z_name, b_s_name, b_z_name, o_s_name, o_z_name, w_name, b_name, 25,
        10240 /*gemm k, padded from 19*512=9728*/, 512, 25 /*bias broadcast*/,
        16, 256, 16 /*sv_N changed to 16*/, true /*performs padding*/,
        model_version_);
    VAIML_DEBUG_PRINT("Finish formatting wts for linear out, total bytes: ",
                      total_wts_bytes);
  } else if (subgraph_id_ == SUBGRAPH_ID::GT_LN_MATMUL_ADD_LN) {
    int8_t* wts_ptr = wts_ptr_ + 1472; // leave space for rtp
    int8_t* rtp_ptr = wts_ptr_ + gt_global_rtp_offset_.at(subgraph_id_);
    total_wts_bytes += GT_LN_WTS_convert(
        wts_, wts_ptr, "/Add_179_output_0_scale",
        "/Add_179_output_0_zero_point",         // in scale, in zp
        "encoder.after_norm.weight_scale",
        "encoder.after_norm.weight_zero_point", // scale scale, scale zp
        "encoder.after_norm.bias_quantized_scale",
        "encoder.encoders.6.norm2.bias_quantized_zero_point", // bias scale,
                                                              // bias zp
        "/after_norm/Add_1_output_0_scale",
        "/after_norm/Add_1_output_0_zero_point", // out scale, out zp
        "encoder.after_norm.weight_quantized",
        "encoder.after_norm.bias_quantized",     // scale, bias
        model_version_);
    total_wts_bytes += GT_MMB_WTS_convert_ptr(
        wts_, wts_ptr, rtp_ptr, "/after_norm/Add_1_output_0_scale",
        "/after_norm/Add_1_output_0_zero_point",     // in scale, in zp
        "/lin_enc/fc/Transpose_output_0_scale",
        "/lin_enc/fc/Transpose_output_0_zero_point", // wts scale , wts zp
        "joint_network.lin_enc.fc.bias_scale",
        "joint_network.lin_enc.fc.bias_zero_point",  // add bias scale, add bias
                                                     // zp
        "/lin_enc/fc/Add_output_0_scale",
        "/lin_enc/fc/Add_output_0_zero_point",       // out scale, out zp
        "/lin_enc/fc/Transpose_output_0_quantized",
        "joint_network.lin_enc.fc.bias_quantized",   // wts, bias
        25, 512, 512, 25 /*bias broadcast*/, 32, 64, 16 /*sv_N changed to 16*/,
        false /*performs padding*/, model_version_);
    wts_ptr = wts_ptr_ + 588288;
    total_wts_bytes += GT_LN_WTS_convert(
        wts_, wts_ptr, "/lin_enc/fc/Add_output_0_scale",
        "/lin_enc/fc/Add_output_0_zero_point",           // in scale, in zp
        "joint_network.lin_enc.Lnorm.weight_scale",
        "joint_network.lin_enc.Lnorm.weight_zero_point", // scale scale, scale
                                                         // zp
        "joint_network.lin_enc.Lnorm.bias_quantized_scale",
        "encoder.encoders.6.norm2.bias_quantized_zero_point", // bias scale,
                                                              // bias zp
        "hidden_state_scale", "hidden_state_zero_point", // out scale, out zp
        "joint_network.lin_enc.Lnorm.weight_quantized",
        "joint_network.lin_enc.Lnorm.bias_quantized",    // scale, bias
        model_version_);
    VAIML_DEBUG_PRINT("Finish formatting wts for tail, total bytes: ",
                      total_wts_bytes);
    // std::string out_filename("gt_tail_dump_wts32.txt");
    // to_file(out_filename, 590400, wts_ptr_ );
  } else if (subgraph_id_ == SUBGRAPH_ID::GT_TRANSFORMER_BLOCK) {
    // set wts from onnx model
    for (int i = 0; i < GT_1_3_TF_NUM; i++) {
      size_t wts_base = 11608064 * i;
      subgraph_id_ = SUBGRAPH_ID::GT_QKV;
      InitTransformerBlockWeights(wts_, wts_ptr_ + wts_base);
      subgraph_id_ = SUBGRAPH_ID::GT_MATMUL_REDUCE;
      InitTransformerBlockWeights(wts_, wts_ptr_ + wts_base + 1783296);
      subgraph_id_ = SUBGRAPH_ID::GT_SM_LINEAR_OUT_FEED_FORWARD;
      InitTransformerBlockWeights(wts_, wts_ptr_ + wts_base + 1843712);
    }
    subgraph_id_ = SUBGRAPH_ID::GT_TRANSFORMER_BLOCK;

    // to_file("wts_tf_gen.bin", 11608064, wts_ptr_);

    {
        // TODO(zhonnian): wts for 1st tf block, to be removed for code merge
        //   std::ifstream ifs("C:\\tf_wts.bin", std::ios::in |
        //   std::ios::binary);
        //   printf("finish reading wts from file\n");
        //   ifs.read(reinterpret_cast<char*>(wts_ptr_), 11608064);
        //   ifs.close();
    }

    { // q-bmm wts
      uint8_t* bmm_wts =
          (uint8_t*)wts_.at("/Transpose_6_output_0_quantized").data;
      auto cache_iter = node_cache.find("BMM_IFM2");
      if (cache_iter == node_cache.end()) {
        std::vector<char> bmm_ifm2_cache(25 * 64 * 480 * sizeof(uint16_t), 0);
        uint16_t* bmm_ifm2 = (uint16_t*)bmm_ifm2_cache.data();
        // 25x64x475 -> 25x64x480 plus unpack to uint16
        for (int i = 0; i < 25 * 64; i++) {
          for (int j = 0; j < 475; j++) {
            bmm_ifm2[i * 480 + j] = bmm_wts[i * 475 + j];
          }
          for (int j = 0; j < 5; j++) {
            bmm_ifm2[i * 480 + 475 + j] = 128; // zp padding
          }
        }
        node_cache.emplace("BMM_IFM2", std::move(bmm_ifm2_cache));
      }
    }
  }
}

void MyCustomOpGT1_3::InitTransformerBlockWeights(
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
  const int8_t* rtp_ptr_start = rtp_ptr;
  std::string ifm_s_name, ifm_z_name, wts_s_name, wts_z_name, ofm_s_name,
      ofm_z_name, wts_w_name;
  std::string scale_s_name, scale_z_name, bias_s_name, bias_z_name, scale_name,
      bias_name;
  size_t total_wts_bytes = 0, wts_sz_loaded = 0;
  if (subgraph_id_ == SUBGRAPH_ID::GT_SM_LINEAR_OUT_FEED_FORWARD) {
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
      wts_w_name = "/linear_out_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Transpose_output_0_quantized";
    } else {
      ifm_s_name = "/MatMul_2_output_0_scale";
      ifm_z_name = "/MatMul_2_output_0_zero_point";
      wts_s_name = "/linear_out/Transpose_output_0_scale";
      wts_z_name = "/linear_out/Transpose_output_0_zero_point";
      wts_w_name = "/linear_out/Transpose_output_0_quantized";
    }
    // we will be using bias name, scale factor and zp for MMB, also ofm name
    // shall using that from add
    if (subgraph_index_linear_out_feed_forward > 0) {
      bias_s_name = "encoder.encoders." +
                    std::to_string(subgraph_index_linear_out_feed_forward) +
                    ".self_attn.linear_out.bias_scale";
      bias_z_name = "encoder.encoders." +
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
      bias_name = "encoder.encoders." +
                  std::to_string(subgraph_index_linear_out_feed_forward) +
                  ".self_attn.linear_out.bias_quantized";
    } else {
      bias_s_name = "encoder.encoders.0.self_attn.linear_out.bias_scale";
      bias_z_name = "encoder.encoders.0.self_attn.linear_out.bias_zero_point";
      ofm_s_name = "/linear_out/Add_output_0_scale";
      ofm_z_name = "/linear_out/Add_output_0_zero_point";
      bias_name = "encoder.encoders.0.self_attn.linear_out.bias_quantized";
    }

    wts_sz_loaded = GT_MMB_WTS_convert_ptr(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        bias_s_name, bias_z_name, ofm_s_name, ofm_z_name, wts_w_name, bias_name,
        25, 512, 512, 25, 32, 64, 16, false, model_version_);
    VAIML_DEBUG_PRINT(
        "Finish to format WTS for linear_out_feed_forward subgraph-",
        subgraph_index_linear_out_feed_forward, "-> MM ", wts_sz_loaded,
        "loaded", " , rtp offset: ", rtp_ptr - rtp_ptr_start);
    // update wts ptr position
    wts_ptr += 584704 - wts_sz_loaded + 64;
    rtp_ptr += 64;

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
    // no wts name change for add in middle
    wts_sz_loaded = GT_ADD_WTS_QDQ_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, 25 * 512, model_version_);
    VAIML_DEBUG_PRINT(
        "Finish to format WTS for linear_out_feed_forward subgraph-",
        subgraph_index_linear_out_feed_forward, "-> ADD3", wts_sz_loaded,
        "loaded, rtp offset: ", rtp_ptr - rtp_ptr_start);

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
      scale_s_name = "encoder.encoders." +
                     std::to_string(subgraph_index_linear_out_feed_forward) +
                     ".norm2.weight_scale";
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
    wts_sz_loaded =
        GT_LN_WTS_convert(wts_, wts_ptr, ifm_s_name, ifm_z_name, scale_s_name,
                          scale_z_name, bias_s_name, bias_z_name, ofm_s_name,
                          ofm_z_name, scale_name, bias_name, model_version_);
    VAIML_DEBUG_PRINT(
        "Finish to format WTS for linear_out_feed_forward subgraph-",
        subgraph_index_linear_out_feed_forward, "-> LN", wts_sz_loaded);

    // feed forward, first part from matmul, second from add
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
      wts_w_name = "/feed_forward/w_1_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Transpose_output_0_quantized";
    } else {
      ifm_s_name = "/norm2/Add_1_output_0_scale";
      ifm_z_name = "/norm2/Add_1_output_0_zero_point";
      wts_s_name = "/feed_forward/w_1/Transpose_output_0_scale";
      wts_z_name = "/feed_forward/w_1/Transpose_output_0_zero_point";
      wts_w_name = "/feed_forward/w_1/Transpose_output_0_quantized";
    }

    if (subgraph_index_linear_out_feed_forward > 0) {
      bias_s_name = "encoder.encoders." +
                    std::to_string(subgraph_index_linear_out_feed_forward) +
                    ".feed_forward.w_1.bias_scale";
      bias_z_name = "encoder.encoders." +
                    std::to_string(subgraph_index_linear_out_feed_forward) +
                    ".feed_forward.w_1.bias_zero_point";
      ofm_s_name = "/feed_forward/w_1_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Add_output_0_scale";
      ofm_z_name = "/feed_forward/w_1_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Add_output_0_zero_point";
      bias_name = "encoder.encoders." +
                  std::to_string(subgraph_index_linear_out_feed_forward) +
                  ".feed_forward.w_1.bias_quantized";
    } else {
      bias_s_name = "encoder.encoders.0.feed_forward.w_1.bias_scale";
      bias_z_name = "encoder.encoders.0.feed_forward.w_1.bias_zero_point";
      ofm_s_name = "/feed_forward/w_1/Add_output_0_scale";
      ofm_z_name = "/feed_forward/w_1/Add_output_0_zero_point";
      bias_name = "encoder.encoders.0.feed_forward.w_1.bias_quantized";
    }
    wts_sz_loaded = GT_MMB_WTS_convert_ptr(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        bias_s_name, bias_z_name, ofm_s_name, ofm_z_name, wts_w_name, bias_name,
        25, 512, 4096, 25, 32, 64, 16, false, model_version_);
    VAIML_DEBUG_PRINT(
        "Finish to format WTS for linear_out_feed_forward subgraph-",
        subgraph_index_linear_out_feed_forward, "-> MM_W1", wts_sz_loaded,
        "loaded, rtp offset: ", rtp_ptr - rtp_ptr_start);
    wts_ptr += 4677632 - wts_sz_loaded + 64;
    rtp_ptr += 64;

    // first come from MM second from Add
    if (subgraph_index_linear_out_feed_forward > 0) {
      ifm_s_name = "/feed_forward/w_1_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Add_output_0_scale";
      ifm_z_name = "/feed_forward/act_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Relu_output_0_zero_point";
      wts_s_name = "/feed_forward/w_2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Transpose_output_0_scale";
      wts_z_name = "/feed_forward/w_2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Transpose_output_0_zero_point";
      wts_w_name = "/feed_forward/w_2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Transpose_output_0_quantized";
    } else {
      ifm_s_name = "/feed_forward/w_1/Add_output_0_scale";
      ifm_z_name = "/feed_forward/act/Relu_output_0_zero_point";
      wts_s_name = "/feed_forward/w_2/Transpose_output_0_scale";
      wts_z_name = "/feed_forward/w_2/Transpose_output_0_zero_point";
      wts_w_name = "/feed_forward/w_2/Transpose_output_0_quantized";
    }
    if (subgraph_index_linear_out_feed_forward > 0) {
      bias_s_name = "encoder.encoders." +
                    std::to_string(subgraph_index_linear_out_feed_forward) +
                    ".feed_forward.w_2.bias_scale";
      bias_z_name = "encoder.encoders." +
                    std::to_string(subgraph_index_linear_out_feed_forward) +
                    ".feed_forward.w_2.bias_zero_point";
      ofm_s_name = "/feed_forward/w_2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Add_output_0_scale";
      ofm_z_name = "/feed_forward/w_2_" +
                   std::to_string(subgraph_index_linear_out_feed_forward) +
                   "/Add_output_0_zero_point";
      bias_name = "encoder.encoders." +
                  std::to_string(subgraph_index_linear_out_feed_forward) +
                  ".feed_forward.w_2.bias_quantized";
    } else {
      bias_s_name = "encoder.encoders.0.feed_forward.w_2.bias_scale";
      bias_z_name = "encoder.encoders.0.feed_forward.w_2.bias_zero_point";
      ofm_s_name = "/feed_forward/w_2/Add_output_0_scale";
      ofm_z_name = "/feed_forward/w_2/Add_output_0_zero_point";
      bias_name = "encoder.encoders.0.feed_forward.w_2.bias_quantized";
    }
    wts_sz_loaded = GT_MMB_WTS_convert_ptr(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        bias_s_name, bias_z_name, ofm_s_name, ofm_z_name, wts_w_name, bias_name,
        25, 4096, 512, 25, 32, 64, 16, false, model_version_);
    VAIML_DEBUG_PRINT(
        "Finish to format WTS for linear_out_feed_forward subgraph-",
        subgraph_index_linear_out_feed_forward, "-> MM_W2 ", wts_sz_loaded,
        " loaded, rtp offset: ", rtp_ptr - rtp_ptr_start);
    wts_ptr += 4498432 - wts_sz_loaded + 64;
    rtp_ptr += 64;

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
    wts_sz_loaded = GT_ADD_WTS_QDQ_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, 25 * 512, model_version_);
    VAIML_DEBUG_PRINT(
        "Finish to format WTS for linear_out_feed_forward subgraph-",
        subgraph_index_linear_out_feed_forward, "-> ADD4", wts_sz_loaded,
        " loaded, rtp offset: ", rtp_ptr - rtp_ptr_start);
    subgraph_index_linear_out_feed_forward++;
  } else if (subgraph_id_ == SUBGRAPH_ID::GT_QKV) {
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
    wts_sz_loaded =
        GT_LN_WTS_convert(wts_, wts_ptr, ifm_s_name, ifm_z_name, scale_s_name,
                          scale_z_name, bias_s_name, bias_z_name, ofm_s_name,
                          ofm_z_name, scale_name, bias_name, model_version_);
    VAIML_DEBUG_PRINT("Finish to format WTS for qkv subgraph-",
                      subgraph_index_qkv, "-> LN ", wts_sz_loaded, " loaded");
    std::string qkv_mm_wts_name =
        subgraph_index_qkv == 0
            ? "_v_3392_quantized"
            : str_fmt("_v_%d_quantized", subgraph_index_qkv * 5 + 3392);
    auto wts_mmb_splitted = SplitTransformerHeadMMWts(qkv_mm_wts_name, wts_);
    int fan_id = 0;
    for (std::string fanout : std::vector<std::string>({"q", "k", "v"})) {
      if (subgraph_index_qkv > 0) {
        ifm_s_name =
            str_fmt("/norm1_%d/Add_1_output_0_scale", subgraph_index_qkv);
        ifm_z_name =
            str_fmt("/norm1_%d/Add_1_output_0_zero_point", subgraph_index_qkv);
        wts_s_name = str_fmt("_v_%d_scale", subgraph_index_qkv * 5 + 3392);
        wts_z_name = str_fmt("_v_%d_zero_point", subgraph_index_qkv * 5 + 3392);
        bias_s_name =
            str_fmt("encoder.encoders.%d.self_attn.linear_%s.bias_scale",
                    subgraph_index_qkv, fanout.c_str());
        bias_z_name =
            str_fmt("encoder.encoders.%d.self_attn.linear_%s.bias_zero_point",
                    subgraph_index_qkv, fanout.c_str());
        ofm_s_name = str_fmt("/linear_%s_%d/Add_output_0_scale", fanout.c_str(),
                             subgraph_index_qkv);
        ;
        ofm_z_name = str_fmt("/linear_%s_%d/Add_output_0_zero_point",
                             fanout.c_str(), subgraph_index_qkv);
        bias_name =
            str_fmt("encoder.encoders.%d.self_attn.linear_%s.bias_quantized",
                    subgraph_index_qkv, fanout.c_str());
      } else {
        ifm_s_name = "/norm1/Add_1_output_0_scale";
        ifm_z_name = "/norm1/Add_1_output_0_zero_point";
        wts_s_name = "_v_3392_scale";
        wts_z_name = "_v_3392_zero_point";
        bias_s_name =
            str_fmt("encoder.encoders.0.self_attn.linear_%s.bias_scale",
                    fanout.c_str());
        bias_z_name =
            str_fmt("encoder.encoders.0.self_attn.linear_%s.bias_zero_point",
                    fanout.c_str());
        ofm_s_name = str_fmt("/linear_%s/Add_output_0_scale", fanout.c_str());
        ofm_z_name =
            str_fmt("/linear_%s/Add_output_0_zero_point", fanout.c_str());
        bias_name =
            str_fmt("encoder.encoders.0.self_attn.linear_%s.bias_quantized",
                    fanout.c_str());
      }
      wts_sz_loaded = GT_MMB_WTS_convert_raw_ptr(
          wts_, wts_ptr, rtp_ptr, (int8_t*)wts_mmb_splitted[fan_id].data(),
          ifm_s_name, ifm_z_name, wts_s_name, wts_z_name, bias_s_name,
          bias_z_name, ofm_s_name, ofm_z_name, bias_name, 25, 512, 512, 25,
          32 /*sv_M*/, 64 /*sv_K*/, 16 /*sv_N*/, model_version_);
      ++fan_id;
      wts_ptr += 584704 - wts_sz_loaded + 64;
      rtp_ptr += 64;
      VAIML_DEBUG_PRINT("Finish to format WTS for qkv subgraph-",
                        subgraph_index_qkv, "-> MMB ", fanout, ", ",
                        wts_sz_loaded, "loaded, rtp offset",
                        rtp_ptr - rtp_ptr_start);
    } // end of qkv matmul + add

    // skip linear_v reshape + transpose + concat don't have weights

    VAIML_DEBUG_PRINT("Finish loading wts for qkv MMB");
    // start of linear_q reshape + transpose + mul + reshape + transpose + bmm
    if (subgraph_index_qkv > 0) {
      ifm_s_name =
          str_fmt("/linear_q_%d/Add_output_0_scale", subgraph_index_qkv);
      ifm_z_name =
          str_fmt("/linear_q_%d/Add_output_0_zero_point", subgraph_index_qkv);
      wts_s_name = "/Constant_15_output_0_scale";
      wts_z_name = "/Constant_15_output_0_zero_point";
      ofm_s_name = str_fmt("/Mul_%d_output_0_scale", subgraph_index_qkv * 5);
      ofm_z_name =
          str_fmt("/Mul_%d_output_0_zero_point", subgraph_index_qkv * 5);
      wts_w_name = "/Constant_15_output_0_quantized";
    } else {
      ifm_s_name = "/linear_q/Add_output_0_scale";
      ifm_z_name = "/linear_q/Add_output_0_zero_point";
      wts_s_name = "/Constant_15_output_0_scale";
      wts_z_name = "/Constant_15_output_0_zero_point";
      ofm_s_name = "/Mul_output_0_scale";
      ofm_z_name = "/Mul_output_0_zero_point";
      wts_w_name = "/Constant_15_output_0_quantized";
    }
    wts_sz_loaded = GT_MUL_WTS_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, model_version_);
    VAIML_DEBUG_PRINT2("Finish to format WTS for qkv subgraph-",
                       subgraph_index_qkv, "-> Mul ", wts_sz_loaded,
                       "loaded, rtp offset: ", rtp_ptr - rtp_ptr_start);
    rtp_ptr += 64; // shall skip 64 bytes to continue concat related param

    // end of linear_q reshape + transpose + mul + reshape + transpose + bmm

    // skip linear_k reshape + transpose + concat don't have weights

    // qdq rtps for k,v gather-concat
    wts_sz_loaded = GT_QDQ_convert(
        wts_, wts_ptr, rtp_ptr, v_unsqueeze_scale_[subgraph_index_qkv],
        v_unsqueeze_zp_[subgraph_index_qkv],
        v_concat_slice_scale_[subgraph_index_qkv],
        v_concat_slice_zp_[subgraph_index_qkv], model_version_);
    VAIML_DEBUG_PRINT2("Finish to format WTS for qkv subgraph-",
                       subgraph_index_qkv, "-> v concat QDQ ", wts_sz_loaded,
                       "loaded, rtp offset: ", rtp_ptr - rtp_ptr_start);

    wts_sz_loaded = GT_QDQ_convert(
        wts_, wts_ptr, rtp_ptr, k_unsqueeze_scale_[subgraph_index_qkv],
        k_unsqueeze_zp_[subgraph_index_qkv],
        k_concat_slice_scale_[subgraph_index_qkv],
        k_concat_slice_zp_[subgraph_index_qkv], model_version_);
    VAIML_DEBUG_PRINT2("Finish to format WTS for qkv subgraph-",
                       subgraph_index_qkv, "-> k concat QDQ ", wts_sz_loaded,
                       "loaded, rtp offset: ", rtp_ptr - rtp_ptr_start);
    subgraph_index_qkv++;
  } else if (subgraph_id_ == SUBGRAPH_ID::GT_MATMUL_REDUCE) {
    // now bmm_1 moved to matmul_reduce
    if (subgraph_index_matmul_reduce > 0) {
      ifm_s_name =
          str_fmt("/Mul_%d_output_0_scale", subgraph_index_matmul_reduce * 5);
      ifm_z_name = str_fmt("/Mul_%d_output_0_zero_point",
                           subgraph_index_matmul_reduce * 5);
      wts_s_name = "/Transpose_6_output_0_scale";
      wts_z_name = "/Transpose_6_output_0_zero_point";
      ofm_s_name = str_fmt("/MatMul_%d_output_0_scale",
                           subgraph_index_matmul_reduce * 3 + 1);
      ofm_z_name = str_fmt("/MatMul_%d_output_0_zero_point",
                           subgraph_index_matmul_reduce * 3 + 1);
    } else {
      ifm_s_name = "/Mul_output_0_scale";
      ifm_z_name = "/Mul_output_0_zero_point";
      wts_s_name = "/Transpose_6_output_0_scale";
      wts_z_name = "/Transpose_6_output_0_zero_point";
      ofm_s_name = "/MatMul_1_output_0_scale";
      ofm_z_name = "/MatMul_1_output_0_zero_point";
    }
    wts_sz_loaded = GT_BMM_WTS_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, false, 8, 64, 512, 16, 64, 32,
        model_version_); // 8 64 475,  N / row / col
    VAIML_DEBUG_PRINT2("Finish to format WTS for matmul_reduce subgraph-",
                       subgraph_index_matmul_reduce, "-> BMM ", wts_sz_loaded,
                       "loaded, rtp_offset: ", rtp_ptr - rtp_ptr_start);
#define NAME_CONCAT(name1, beta, scale, name2)                                 \
  name1 + std::to_string(beta + subgraph_index_matmul_reduce * scale) + name2;
    if (subgraph_index_matmul_reduce > 0) {
      ifm_s_name = NAME_CONCAT("/Mul_", 0, 5, "_output_0_scale");
      ifm_z_name = NAME_CONCAT("/Mul_", 0, 5, "_output_0_zero_point");
      wts_s_name = bmm_scale_wts_prefix_.at(subgraph_index_matmul_reduce);
      wts_z_name = NAME_CONCAT("/Concat_", 114, 8, "_output_0_zero_point");
      ofm_s_name = NAME_CONCAT("/MatMul_", 0, 3, "_output_0_scale");
      ofm_z_name = NAME_CONCAT("/MatMul_", 0, 3, "_output_0_zero_point");
    } else {
      ifm_s_name = "/Mul_output_0_scale";
      ifm_z_name = "/Mul_output_0_zero_point";
      wts_s_name = "/linear_k/Add_output_0_scale";
      wts_z_name = "/Concat_6_output_0_zero_point";
      ofm_s_name = "/MatMul_output_0_scale";
      ofm_z_name = "/MatMul_output_0_zero_point";
    }
    wts_sz_loaded = GT_BMM_WTS_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, "", true, 25, 64, 512, 32, 64, 32,
        model_version_); // 25 64 475, N / row / col
    VAIML_DEBUG_PRINT2("Finish to format WTS for Matmul-Reduce subgraph-",
                       subgraph_index_matmul_reduce, "-> BMM", wts_sz_loaded,
                       "loaded, rtp_offset: ", rtp_ptr - rtp_ptr_start);

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
    wts_sz_loaded = GT_ADD_WTS_QDQ_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, 8 * 25 * 475, model_version_);
    VAIML_DEBUG_PRINT2("Finish to format WTS for Matmul-Reduce subgraph-",
                       subgraph_index_matmul_reduce, "-> ADD", wts_sz_loaded,
                       " loaded, rtp offset: ", rtp_ptr - rtp_ptr_start,
                       "rtp_ptr absolute offset: ", rtp_ptr - wts_ptr_);

    if (subgraph_index_matmul_reduce > 0) {
      ifm_s_name = mul_ifm_scale.at(subgraph_index_matmul_reduce);
      ifm_z_name = NAME_CONCAT("/Add_", 0, 5, "_output_0_zero_point");
      wts_s_name = "/Softmax_13_output_0_scale";
      wts_z_name = "/Slice_1_output_0_zero_point";
      ofm_s_name = mul_output_scale.at(subgraph_index_matmul_reduce);
      ofm_z_name = NAME_CONCAT("/Mul_", 3, 5, "_output_0_zero_point");
      wts_w_name = "/Slice_1_output_0_QuantizeLinear_Output";
    } else {
      ifm_s_name = "/Add_output_0_scale";
      ifm_z_name = "/Add_output_0_zero_point";
      wts_s_name = "/Softmax_13_output_0_scale";
      wts_z_name = "/Slice_1_output_0_zero_point";
      ofm_s_name = "/Mul_3_output_0_scale";
      ofm_z_name = "/Mul_3_output_0_zero_point";
      wts_w_name = "";
    }
    wts_sz_loaded = GT_MUL_WTS_QDQ_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, 8 * 25 * 480, model_version_);
    VAIML_DEBUG_PRINT2("Finish to format WTS for Matmul-Reduce subgraph-",
                       subgraph_index_matmul_reduce, "-> Mul ", wts_sz_loaded,
                       " loaded, rtp offset: ", rtp_ptr - rtp_ptr_start);
    // Mul-4
    if (subgraph_index_matmul_reduce > 0) {
      ifm_s_name = NAME_CONCAT("/Add_", 0, 5, "_output_0_scale");
      // mul2_scale.at(subgraph_index_matmul_reduce);
      ifm_z_name = NAME_CONCAT("/Mul_", 3, 5, "_output_0_zero_point");
      wts_s_name = "/Softmax_13_output_0_scale";
      wts_z_name = "/Slice_output_0_zero_point";
      ofm_s_name = mul2_scale.at(subgraph_index_matmul_reduce);
      ofm_z_name = NAME_CONCAT("/Mul_", 4, 5, "_output_0_zero_point");
      wts_w_name = "";
    } else {
      ifm_s_name = "/Add_output_0_scale";
      ifm_z_name = "/Mul_3_output_0_zero_point";
      wts_s_name = "/Softmax_13_output_0_scale";
      wts_z_name = "/Slice_output_0_zero_point";
      ofm_s_name = "/Mul_4_output_0_scale";
      ofm_z_name = "/Mul_4_output_0_zero_point";
      wts_w_name = "";
    }
    wts_sz_loaded = GT_MUL_WTS_QDQ_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, 8 * 25 * 480, model_version_);
    VAIML_DEBUG_PRINT2("Finish to format WTS for Matmul-Reduce subgraph-",
                       subgraph_index_matmul_reduce, "-> Mul ", wts_sz_loaded,
                       "loaded, rtp offset: ", rtp_ptr - rtp_ptr_start);

    if (subgraph_index_matmul_reduce > 0) {
      ifm_s_name = add_scale.at(subgraph_index_matmul_reduce);
      ifm_z_name = NAME_CONCAT("/Mul_", 3, 5, "_output_0_zero_point");
      wts_s_name = mul2_scale.at(subgraph_index_matmul_reduce);
      wts_z_name = NAME_CONCAT("/Mul_", 4, 5, "_output_0_zero_point");
      ofm_s_name = add_scale.at(subgraph_index_matmul_reduce);
      ofm_z_name = NAME_CONCAT("/Add_", 2, 5, "_output_0_zero_point");
      wts_w_name = "";
    } else {
      ifm_s_name = "/Mul_3_output_0_scale";
      ifm_z_name = "/Mul_3_output_0_zero_point";
      wts_s_name = "/Mul_4_output_0_scale";
      wts_z_name = "/Mul_4_output_0_zero_point";
      ofm_s_name = "/Mul_3_output_0_scale";
      ofm_z_name = "/Add_2_output_0_zero_point";
      wts_w_name = "";
    }
    wts_sz_loaded = GT_ADD_WTS_QDQ_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, 8 * 25 * 480, model_version_);
    VAIML_DEBUG_PRINT2("Finish to format WTS for Matmul-Reduce subgraph-",
                       subgraph_index_matmul_reduce, "-> ADD", wts_sz_loaded,
                       "loaded, rtp offset: ", rtp_ptr - rtp_ptr_start);
#undef NAME_CONCAT
#define NAME_CONCAT(name1, beta, scale, name2)                                 \
  name1 + std::to_string(beta + subgraph_index_matmul_reduce * scale) + name2;
    // softmax and bmm2 is also in matmul reduce now
    if (subgraph_index_matmul_reduce > 0) {
      ifm_s_name = softmax_ifm_prefix_.at(subgraph_index_matmul_reduce);
      ifm_z_name = NAME_CONCAT("/Add_", 2, 5, "_output_0_zero_point");
      ofm_s_name = "/Softmax_13_output_0_scale";
      ofm_z_name = "/Softmax_" + std::to_string(subgraph_index_matmul_reduce) +
                   "_output_0_zero_point";
    } else {
      ifm_s_name = "/Mul_3_output_0_scale";
      ifm_z_name = "/Add_2_output_0_zero_point";
      ofm_s_name = "/Softmax_13_output_0_scale";
      ofm_z_name = "/Softmax_output_0_zero_point";
    }
    // softmax shall take only wts size and no rtp will be used
    wts_sz_loaded = GT_SOFTMAX_WTS_convert(
        wts_, wts_ptr, ifm_s_name, ifm_z_name, ofm_s_name, ofm_z_name,
        480 /*K*/, 475 /*K_valid*/, model_version_);
    VAIML_DEBUG_PRINT2("Finish to format WTS for matmul_reduce subgraph-",
                       subgraph_index_matmul_reduce, "-> Softmax",
                       wts_sz_loaded, "loaded, rtp offset",
                       rtp_ptr - rtp_ptr_start);

    if (subgraph_index_matmul_reduce > 0) {
      std::set<int> set_slice = {
          3, 5, 7, 8, 9, 14, 20, 21, 23, 28, 34,
      };
      std::set<int> set_concat = {1,  2,  4,  12, 13, 16, 17, 18,
                                  19, 25, 26, 29, 31, 32, 33, 35};
      std::set<int> set_linear_v = {
          10, 11, 15, 22, 24, 30,
      };
      std::set<int> set_unsqueeze = {
          6,
          27,
      };
      if (set_slice.find(subgraph_index_matmul_reduce) != set_slice.end()) {
        // 3, 7, 11, 15, 18, ..., 143
        wts_s_name = "/Slice_" +
                     std::to_string(4 * subgraph_index_matmul_reduce + 3) +
                     "_output_0_scale";
      } else if (set_concat.find(subgraph_index_matmul_reduce) !=
                 set_concat.end()) {
        wts_s_name = "/Concat_" +
                     std::to_string(8 * subgraph_index_matmul_reduce + 115) +
                     "_output_0_scale";
      } else if (set_linear_v.find(subgraph_index_matmul_reduce) !=
                 set_linear_v.end()) {
        wts_s_name = "/linear_v_" +
                     std::to_string(subgraph_index_matmul_reduce) +
                     "/Add_output_0_scale";
      } else if (set_unsqueeze.find(subgraph_index_matmul_reduce) !=
                 set_unsqueeze.end()) {
        wts_s_name = "/Unsqueeze_" +
                     std::to_string(25 * subgraph_index_matmul_reduce + 385) +
                     "_output_0_scale";
      }
      // 7, 123, 131, 139, 147, ..., 395
      wts_z_name = "/Concat_" +
                   std::to_string(8 * subgraph_index_matmul_reduce + 115) +
                   "_output_0_zero_point";
      // 2, 5, 8, 11, 14, 107
      ofm_s_name = "/MatMul_" +
                   std::to_string(3 * subgraph_index_matmul_reduce + 2) +
                   "_output_0_scale";
      ofm_z_name = "/MatMul_" +
                   std::to_string(3 * subgraph_index_matmul_reduce + 2) +
                   "_output_0_zero_point";
      wts_w_name = "/Concat_" +
                   std::to_string(8 * subgraph_index_matmul_reduce + 115) +
                   "_output_0_QuantizeLinear_Output";

      ifm_s_name = "/Softmax_13_output_0_scale";
      ifm_z_name = "/Softmax_" + std::to_string(subgraph_index_matmul_reduce) +
                   "_output_0_zero_point";
    } else {
      ifm_s_name = "/Softmax_13_output_0_scale";
      ifm_z_name = "/Softmax_output_0_zero_point";
      wts_s_name = "/linear_v/Add_output_0_scale";
      wts_z_name = "/Concat_7_output_0_zero_point";
      ofm_s_name = "/MatMul_2_output_0_scale";
      ofm_z_name = "/MatMul_2_output_0_zero_point";
      wts_w_name = "/Concat_7_output_0_QuantizeLinear_Output";
    }
    wts_sz_loaded = GT_BMM_WTS_convert(
        wts_, wts_ptr, rtp_ptr, ifm_s_name, ifm_z_name, wts_s_name, wts_z_name,
        ofm_s_name, ofm_z_name, wts_w_name, false, 25, 512, 64, 16, 128, 16,
        model_version_); // this will be the last bmm in our, padding occurs!
    VAIML_DEBUG_PRINT(
        "Finish to format WTS for linear_out_feed_forward subgraph-",
        subgraph_index_matmul_reduce, "-> MM2 ", wts_sz_loaded,
        ", loaded, rtp offset: ", rtp_ptr - rtp_ptr_start);
    // note we are not using up all available wts buffer, the pointer is updated
    // upon next enter with subgraph_id_ == sm_feed_forward
    subgraph_index_matmul_reduce++;
#undef NAME_CONCAT
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
}

int32_t MyCustomOpGT1_3::Slice144Compute_GT(Ort::KernelContext& ctx) const {
  auto inputvalue = ctx.GetInput(0);
  const void* input = inputvalue.GetTensorRawData();
  auto output_shapes = ort_output_shapes_;
  auto& output_shape = output_shapes[0];
  auto ortvalue = ctx.GetOutput(0, output_shape.data(), output_shape.size());
  void* output = ortvalue.GetTensorMutableRawData();
  int zp = 0;
  float scale = 0.00042341125663369894;
  for (int i = 0; i < 3 * 80; i++) {
    ((float*)output)[i] = (((const uint16_t*)input)[100 * 80 + i] - zp) * scale;
  }
  return 3 * 80 * sizeof(float);
}

int32_t MyCustomOpGT1_3::MainBlockInputs(Ort::KernelContext& ctx) const {
  /* onnx order:
        /Reshape_1_output_0 inp_cache_v (v-gather)
        /Reshape_output_0 inp_cache_k (k-gather)
        /Slice_1_output_0_QuantizeLinear_Output mul before reduce min
        /Slice_output_0_QuantizeLinear_Output mul after reduce min
        /out/Add_output_0_QuantizeLinear_Output add-ln

      "/out/Add_output_0_QuantizeLinear_Output",
      "inp_cache_k",
      "inp_cache_v",
      "mask_DequantizeLinear_Output",
      "mask_DequantizeLinear_Output/duplicated"
    txn order:
      add-ln, bmm, v gather, k gather,
      |mul after reducemin(0:475 of mask), mul before reducemin(475:950 of
    mask)|
  */
  auto get_ort_pointer = [&ctx](size_t idx) {
    return ctx.GetInput(idx).GetTensorRawData();
  };
  // { // conv
  //   const void* input = get_ort_pointer(0);
  //   memcpy(ifm_ptr_, input, 103 * 80 * sizeof(uint16_t));
  // }
  { // TODO(zhonnian): add-ln, will be removed for super cmdlist
    const void* input = get_ort_pointer(0);
    memcpy(ifm_ptr_, input, 25 * 512 * sizeof(uint16_t));
  }

  for (int tf_iter = 0; tf_iter < GT_1_3_TF_NUM; tf_iter++) {
    size_t base_ddr = tf_iter * 4300800;
    { // bmm
      int8_t* bmm_input = ifm_ptr_ + GT_FRONT_SZ + base_ddr + 25600;
      const std::vector<char>& bmm_ifm2_cache = node_cache.at("BMM_IFM2");
      memcpy(bmm_input, bmm_ifm2_cache.data(), bmm_ifm2_cache.size());
    }
    // v k gather
    // slice and pad
    // 36x8x450x64 -> 1x8x475x64
    for (auto concat_input_idx : {2, 1}) {
      bool is_v_gather = concat_input_idx == 2;
      size_t word_offset = is_v_gather ? 1561600 : 2048000;
      int8_t* concat_input_ptr =
          ifm_ptr_ + GT_FRONT_SZ + base_ddr + word_offset;
      const void* input = get_ort_pointer(concat_input_idx);
      for (int i = 0; i < 8; i++) {
        float_to_bfloat16_avx512_unrolled(
            (const float*)input + tf_iter * 8 * 450 * 64 + i * 450 * 64,
            (uint16_t*)concat_input_ptr + i * 475 * 64, 450 * 64);
      }
    }

    {
      // Mul before & after ReduceMin
      // need to perform slicing ?? not for single tf block
      // 8x25x950 -> 8x25x475(0-475, padded to 480) after reducemin
      // 8x25x950 -> 8x25x475(475-950, padded to 480) before reducemin
      int8_t* mul_input_ptr = ifm_ptr_ + GT_FRONT_SZ + base_ddr + 2534400;
      const void* input = get_ort_pointer(3);
      auto cache_iter = node_cache.find("Slice");
      if (cache_iter == node_cache.end()) {
        for (int i = 0; i < 8 * 25; i++) {
          memcpy((uint16_t*)mul_input_ptr + i * 480,
                 (const uint16_t*)input + i * 950, 475 * sizeof(uint16_t));
          memcpy((uint16_t*)mul_input_ptr + 8 * 25 * 480 + i * 480,
                 (const uint16_t*)input + i * 950 + 475,
                 475 * sizeof(uint16_t));
        }
        // const void* slice_input = get_ort_pointer(3);
        // const void* slice_1_input = get_ort_pointer(2);
        // for (int i = 0; i < 8 * 25; i++) {
        //   memcpy((uint16_t*)mul_input_ptr + i * 480,
        //          (const uint16_t*)slice_input + i * 475,
        //          475 * sizeof(uint16_t));
        //   memcpy((uint16_t*)mul_input_ptr + 8 * 25 * 480 + i * 480,
        //          (const uint16_t*)slice_1_input + i * 475,
        //          475 * sizeof(uint16_t));
        // }
        // std::vector<char> slice_cache(8 * 25 * 480 * sizeof(uint16_t), 0);
        // memcpy(slice_cache.data(), mul_input_ptr, slice_cache.size());
        // node_cache.emplace("Slice", std::move(slice_cache));

        // std::vector<char> slice_1_cache(8 * 25 * 480 * sizeof(uint16_t), 0);
        // memcpy(slice_1_cache.data(),
        //        mul_input_ptr + 8 * 25 * 480 * sizeof(uint16_t),
        //        slice_1_cache.size());
        // node_cache.emplace("Slice_1", std::move(slice_1_cache));
      } else {
        const std::vector<char>& slice_cache = cache_iter->second;
        memcpy(mul_input_ptr, slice_cache.data(), slice_cache.size());
        const std::vector<char>& slice_1_cache = node_cache.at("Slice_1");
        memcpy(mul_input_ptr + 8 * 25 * 480 * sizeof(uint16_t),
               slice_1_cache.data(), slice_1_cache.size());
      }
    }
  }
  if (node_cache.find("Slice") != node_cache.end()) {
    node_cache.erase("Slice");
    node_cache.erase("Slice_1");
  }
  return 0;
}

int32_t MyCustomOpGT1_3::MainBlockOutputs(Ort::KernelContext& ctx) const {
  /*
   onnx order:
        oup_lid_QuantizeLinear_Output
        oup_cache_v
        oup_cache_k
        /Add_179_output_0_QuantizeLinear_Output
   */
  auto get_ort_pointer = [&ctx, this](size_t idx) {
    auto output_shapes = this->ort_output_shapes_;
    auto& output_shape = output_shapes[idx];
    auto ortvalue =
        ctx.GetOutput(idx, output_shape.data(), output_shape.size());
    return ortvalue.GetTensorMutableRawData();
  };

  // for (auto split_idx : {1,2}) {
  //   void* data = get_ort_pointer(split_idx);
  //   bool is_v_split = (split_idx == 1);
  for (auto split_idx : {1, 2}) {
    void* data = get_ort_pointer(split_idx);
    bool is_v_split = (split_idx == 1);
    size_t xrt_offset_kv = is_v_split ? 1561600 : 2048000;
    for (int tf_iter = 0; tf_iter < GT_1_3_TF_NUM; tf_iter++) {
      size_t xrt_base_ddr = tf_iter * 4300800 + xrt_offset_kv + GT_FRONT_SZ;
      size_t ort_base_ddr = tf_iter * 8 * 450 * 64;
      for (int i = 0; i < 8; i++) {
        bfloat16_to_float_avx512_unrolled(
            (uint16_t*)(ofm_ptr_ + xrt_base_ddr) + i * 475 * 64 + 25 * 64,
            (float*)data + ort_base_ddr + i * 450 * 64, 450 * 64);
      }
    }
  }

  {
    // Add_179_output_0_QuantizeLinear_Output
    void* data = get_ort_pointer(3);
    memcpy(data, ofm_ptr_ + GT_FRONT_SZ + GT_1_3_TF_NUM * 4300800,
           25 * 512 * sizeof(uint16_t));
  }
  {
    // oup_lid
    void* data = get_ort_pointer(0);
    memcpy(data, ofm_ptr_ + GT_FRONT_SZ + 15 * 4300800,
           25 * 512 * sizeof(uint16_t));
  }
  return 0;
}

void MyCustomOpGT1_3::Compute(const OrtApi* api,
                              OrtKernelContext* context) const {
  if (Ort::Global<void>::api_ == nullptr) {
    Ort::Global<void>::api_ = api;
  }
  Ort::KernelContext ctx(context);
  // CPU graph, skip hw runner
  if (subgraph_id_ == SUBGRAPH_ID::GT_CACHE_FRAMES_SLICE) {
    Slice144Compute_GT(ctx);
    return;
  }
  const void* input_deq_dup = nullptr;
  // prepare for input
  if (subgraph_id_ == SUBGRAPH_ID::GT_FRONT) {
    {
      auto num_inputs = ctx.GetInputCount();
      VAIML_DEBUG_PRINT("inputs number: ", num_inputs);
      auto inputvalue = ctx.GetInput(0); // 0 is not used
      const void* input = inputvalue.GetTensorRawData();
      auto type_and_shape = inputvalue.GetTensorTypeAndShapeInfo();
      auto num_elements = type_and_shape.GetElementCount();
      auto input_type = type_and_shape.GetElementType();
      memcpy(ifm_ptr_, input, num_elements * sizeof(uint16_t));
    }
  } else if (subgraph_id_ == SUBGRAPH_ID::GT_FRONT_MM) {
    {
      auto num_inputs = ctx.GetInputCount();
      VAIML_DEBUG_PRINT("inputs number: ", num_inputs);
      auto inputvalue = ctx.GetInput(0);
      const void* input = inputvalue.GetTensorRawData();
      auto type_and_shape = inputvalue.GetTensorTypeAndShapeInfo();
      // printf("shape of linear out input");
      // for (auto d : type_and_shape.GetShape()) {
      //   printf("%lld, ", d);
      // }
      // printf("type: %d\n", type_and_shape.GetElementType());
      // 25x512x19->25x512x20
      for (int i = 0; i < 25; ++i) {
        for (int j = 0; j < 512; ++j) {
          memcpy((uint16_t*)(ifm_ptr_ + 16480) + i * 10240 + j * 20,
                 (const uint16_t*)input + i * 9728 + j * 19,
                 19 * sizeof(uint16_t));
        }
      }
      // for (int i = 0; i < 128; ++i) {
      //   std::cout << std::setw(8) << std::hex << std::setfill('0')
      //             << *((uint32_t*)(ifm_ptr_ + 16480) + i) << "    "
      //             << *((uint32_t*)input + i) << std::endl;
      // }
    }
  } else if (subgraph_id_ == SUBGRAPH_ID::GT_LN_MATMUL_ADD_LN) {
    auto num_inputs = ctx.GetInputCount();
    auto inputvalue = ctx.GetInput(0);
    const void* input = inputvalue.GetTensorRawData();
    auto type_and_shape = inputvalue.GetTensorTypeAndShapeInfo();
    memcpy(ifm_ptr_, input, 25 * 512 * sizeof(uint16_t));
  } else if (subgraph_id_ == SUBGRAPH_ID::GT_TRANSFORMER_BLOCK) {
    MainBlockInputs(ctx);
    // to_file("ifm.bin", 4300800 * 2 + 25600, ifm_ptr_);
  }

  // hardware run
  auto output_shapes = ort_output_shapes_;
  VAIML_DEBUG_PRINT("    outputs number: ", output_shapes.size());
  auto err_status = const_cast<hw_runner&>(g).run(
      (void*)ifm_ptr_, (void*)wts_ptr_, (void*)ofm_ptr_);

  if (subgraph_id_ == GT_FRONT) {
    {
      int index = 0; // /Reshape_2_output_0_QuantizeLinear_Output
      auto& output_shape = output_shapes[index];
      auto ortvalue =
          ctx.GetOutput(index, output_shape.data(), output_shape.size());
      auto type_and_shape = ortvalue.GetTensorTypeAndShapeInfo();
      auto tensor_type = type_and_shape.GetElementType();
      auto num_elements = type_and_shape.GetElementCount();
      void* data = ortvalue.GetTensorMutableRawData();
      VAIML_DEBUG_PRINT("conv output num_elements ", num_elements);
      for (int i = 0; i < 25; ++i) {
        for (int j = 0; j < 512; ++j) {
          memcpy((uint16_t*)data + i * 9728 + j * 19,
                 (uint16_t*)(ofm_ptr_ + 16480) + i * 10240 + j * 20,
                 19 * sizeof(uint16_t));
          memset((uint16_t*)(ifm_ptr_ + 16480) + i * 10240 + j * 20 + 19, 0,
                 sizeof(uint16_t));
        }
      }
    }
  } else if (subgraph_id_ == GT_FRONT_MM) {
    int index = 0;
    auto& output_shape = output_shapes[index];
    auto ortvalue =
        ctx.GetOutput(index, output_shape.data(), output_shape.size());
    void* data = ortvalue.GetTensorMutableRawData();
    memcpy(data, ofm_ptr_ + 16480 + 552960, 25600);
  } else if (subgraph_id_ == SUBGRAPH_ID::GT_LN_MATMUL_ADD_LN) {
    int index = 0;
    auto& output_shape = output_shapes[index];
    auto ortvalue =
        ctx.GetOutput(index, output_shape.data(), output_shape.size());
    void* data = ortvalue.GetTensorMutableRawData();
    memcpy(data, ofm_ptr_, 25600);
  } else if (subgraph_id_ == SUBGRAPH_ID::GT_TRANSFORMER_BLOCK) {
    MainBlockOutputs(ctx);
    // to_file("ofm.bin", 4300800*2+25600, ofm_ptr_);
  }

  if (err_status == -2) {
    printf("BOARD CRASHED\n");
  }
}

MyCustomOpGT1_3::~MyCustomOpGT1_3() {}
} // namespace vaip_vaiml_custom_op
