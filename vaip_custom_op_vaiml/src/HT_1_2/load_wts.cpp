/*
 *     The Xilinx Vitis AI Vaip in this distribution are provided under the
 * following free and permissive binary-only license, but are not provided in
 * source code form.  While the following free and permissive license is similar
 * to the BSD open source license, it is NOT the BSD open source license nor
 * other OSI-approved open source license.
 *
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
#include "load_wts.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>

// #define VAIML_CUSTOM_OP_LOAD_WTS_PROFILING
namespace vaip_vaiml_custom_op {
std::vector<uint16_t>
wts_gen_layernorm(int32_t* bias_data, int8_t* wts_data, uint32_t dim,
                  float s_bias, int64_t zp_bias, float s_weight,
                  int64_t zp_weight, float s_ifm, int64_t zp_ifm, float s_ofm,
                  int64_t zp_ofm, float ifm_scale_refactor,
                  float ofm_scale_refactor) {
  std::vector<uint16_t> res;
  fp32_uint32 a;

  // push back 16 uint32_t RTPs
  a.UI32 = dim;
  res.push_back(a.UI16[0]);
  res.push_back(a.UI16[1]);
  a.UI32 = dim / 8;
  res.push_back(a.UI16[0]);
  res.push_back(a.UI16[1]);
  a.UI32 = (uint32_t)((double)(zp_ifm + 128) / ifm_scale_refactor);
  res.push_back(a.UI16[0]);
  res.push_back(a.UI16[1]);
  a.FP32 = s_ifm * ifm_scale_refactor;
  res.push_back(a.UI16[1]);
  res.push_back((uint16_t)0);
  a.UI32 = (uint32_t)((double)(zp_ofm + 128) / ofm_scale_refactor);
  res.push_back(a.UI16[0]);
  res.push_back(a.UI16[1]);
  a.FP32 = 1.0 / (s_ofm * ofm_scale_refactor);
  res.push_back(a.UI16[1]);
  res.push_back((uint16_t)0);
  a.FP32 = 1.0 / dim;
  res.push_back(a.UI16[0]);
  res.push_back(a.UI16[1]);
  for (int i = 0; i < 9; i++) {
    res.push_back((uint16_t)0);
    res.push_back((uint16_t)0);
  }

  uint32_t dim_padded_len = (dim + 511) / 512 * 512 - dim;

  // push back padded weight
  for (int i = 0; i < dim; i++) {
    int64_t tmp = (int64_t)wts_data[i] + 128;
    a.FP32 = bfloat16_rnd_even(dequant(tmp, (zp_weight + 128), s_weight));
    res.push_back(a.UI16[1]);
  }
  for (int i = 0; i < dim_padded_len; i++) {
    res.push_back((uint16_t)0);
  }

  // push back padded bias
  for (int i = 0; i < dim; i++) {
    int64_t tmp =
        (int64_t)bias_data[i] - std::numeric_limits<std::int32_t>::min();
    a.FP32 = bfloat16_rnd_even(dequant(
        tmp, (zp_bias - std::numeric_limits<std::int32_t>::min()), s_bias));
    res.push_back(a.UI16[1]);
  }
  for (int i = 0; i < dim_padded_len; i++) {
    res.push_back((uint16_t)0);
  }

  return res;
}

std::vector<uint16_t> wts_gen_sigmoid(float s_ifm, int64_t zp_ifm, float s_ofm,
                                      int64_t zp_ofm) {
  std::vector<uint16_t> res;
  fp32_uint32 a;

  // push back 16 uint32_t RTPs
  a.UI32 = 64;
  res.push_back(a.UI16[0]);
  res.push_back(a.UI16[1]);
  a.UI32 = zp_ifm + 128;
  res.push_back(a.UI16[0]);
  res.push_back(a.UI16[1]);
  a.FP32 = s_ifm * 0.5;
  res.push_back(a.UI16[1]);
  res.push_back((uint16_t)0);
  a.FP32 = zp_ofm + 128;
  res.push_back(a.UI16[0]);
  res.push_back(a.UI16[1]);
  a.FP32 = 1.0 / s_ofm;
  res.push_back(a.UI16[1]);
  res.push_back((uint16_t)0);
  for (int i = 0; i < 11; i++) {
    res.push_back((uint16_t)0);
    res.push_back((uint16_t)0);
  }

  return res;
}

void loadAdd128(std::vector<uint8_t>& dst, int8_t* src, int size) {
  for (int i = 0; i < size; i++) {
    dst.push_back(static_cast<uint8_t>(src[i] + 128));
  }
  return;
}

int saveMemoryToCache(const char* mem, size_t mem_size,
                      const vaip_core::PassContext& context,
                      std::string filename) {
  auto filepath = context.get_log_dir() / (filename + ".bin");
  const_cast<vaip_core::PassContext&>(context).write_file(
      filename + ".bin", gsl::span<const char>(mem, mem_size));
  std::cout << mem_size << " bytes of memory saved to cache " << filepath
            << std::endl;
  return 0;
}

int htGenerateLstmInput(const LstmSettings& s,
                        const struct lstm_init_wts& lstm_in, uint8_t* result,
                        const vaip_core::PassContext& context) {
  auto filename = s.layer_name + ".bin";
  auto wts_v = context.read_file_c8(filename).value();
  memcpy(result, wts_v.data(), wts_v.size());
  return 0;

  // printf("### count: %d\n", count);
  // printf("####: %s: wrote %d bytes of data\n", s.layer_name.c_str(), p -
  // wts);
  // std::string wtsfile = s.layer_name + ".wts";
  // dumpMemoryToFile(wts, 2162688 * 4, wtsfile);
  double Sx, Sw, Sr, Sb, Sh, Sc, Sy1, Sy2;
  int Zx, Zw, Zr, Zb, Zh, Zc, Zy1, Zy2;
  int Sg = 1;
  int Zg = 0;
  if (s.layer_id == 320) {
    Sx = lstm_in.scale[9];
    Zx = lstm_in.zp[9];
    Sw = lstm_in.scale[10];
    Zw = lstm_in.zp[10];
    Sr = lstm_in.scale[11];
    Zr = lstm_in.zp[11];
    Sb = lstm_in.scale[12];
    Zb = lstm_in.zp[12];
    Sh = lstm_in.scale[4];
    Zh = lstm_in.zp[4];
    Sc = lstm_in.scale[1];
    Zc = lstm_in.zp[1];
    Sy1 = lstm_in.scale[14];
    Zy1 = lstm_in.zp[14];
    Sy2 = lstm_in.scale[15];
    Zy2 = lstm_in.zp[15];
  } else if (s.layer_id == 1024) {
    Sx = lstm_in.scale[13];
    Zx = lstm_in.zp[13];
    Sw = lstm_in.scale[16];
    Zw = lstm_in.zp[16];
    Sr = lstm_in.scale[17];
    Zr = lstm_in.zp[17];
    Sb = lstm_in.scale[18];
    Zb = lstm_in.zp[18];
    Sh = lstm_in.scale[5];
    Zh = lstm_in.zp[5];
    Sc = lstm_in.scale[2];
    Zc = lstm_in.zp[2];
    Sy1 = lstm_in.scale[20];
    Zy1 = lstm_in.zp[20];
    Sy2 = lstm_in.scale[21];
    Zy2 = lstm_in.zp[21];
  }

  int8_t* x_wts_p;
  int8_t* h_wts_p;
  int8_t* b_wts_p;
  if (s.layer_id == 320) {
    x_wts_p = (lstm_in.lstm0_x_wts);
    h_wts_p = (lstm_in.lstm0_h_wts);
    b_wts_p = (lstm_in.lstm0_bias);
  } else {
    x_wts_p = (lstm_in.lstm1_x_wts);
    h_wts_p = (lstm_in.lstm1_h_wts);
    b_wts_p = (lstm_in.lstm1_bias);
  }
  std::vector<uint8_t> w_u8; // (x_wts_p, x_wts_p + 4096 * s.len_x);
  std::vector<uint8_t> r_u8; // (h_wts_p, h_wts_p + 4096 * s.len_h);
  std::vector<uint8_t> b_u8; // (b_wts_p, b_wts_p + 8192);
  loadAdd128(w_u8, x_wts_p, 4096 * s.len_x);
  loadAdd128(r_u8, h_wts_p, 4096 * s.len_h);
  loadAdd128(b_u8, b_wts_p, 8192);

  WTensor<int32_t> w_i32 = WTensor<int32_t>::createFromVector(w_u8);
  WTensor<int32_t> r_i32 = WTensor<int32_t>::createFromVector(r_u8);
  WTensor<int32_t> b_i32 = WTensor<int32_t>::createFromVector(b_u8);
  // w_i32.print("### wi32: ", PARTIAL_DATA);
  // r_i32.print("### ri32: ", PARTIAL_DATA);
  // b_i32.print("### bi32: ", PARTIAL_DATA);

  // b_i32.print("b_i32: ", PARTIAL_DATA);
  auto bw_i32 = b_i32.slice({0}, {4096}).reshape({4096});
  // bw_i32.print("bw_i32: ", PARTIAL_DATA);
  auto br_i32 = b_i32.slice({4096}, {8192}).reshape({4096});
  // br_i32.print("br_i32: ", PARTIAL_DATA);

  // printf("###--- len_x:%d, len_h:%d\n", s.len_x, s.len_h);
  auto SWk_i32 = w_i32.reshape({4096, s.len_x})
                     .transpose({1, 0})
                     .reduceSum(0)
                     .reshape({4096});
  // SWk_i32.print("SWk_i32: ", PARTIAL_DATA);
  auto SRk_i32 = r_i32.reshape({4096, s.len_h})
                     .transpose({1, 0})
                     .reduceSum(0)
                     .reshape({4096});
  // SRk_i32.print("SRk_i32: ", PARTIAL_DATA);

  // Calculate QDQ values
  double Qx = Sx * Sw / Sg;
  double Qh = Sh * Sr / Sg;
  double Qa = Qx * Zw;
  double Qb = Qh * Zr;
  // printf("--- Qx:%.6f, Qh:%.6f, Qa:%.6f, Qb:%.6f\n", Qx, Qh, Qa, Qb);
  WTensor<double> bw_dbl = WTensor<double>::createFromVector(bw_i32.data());
  WTensor<double> br_dbl = WTensor<double>::createFromVector(br_i32.data());
  WTensor<double> SWk_dbl = WTensor<double>::createFromVector(SWk_i32.data());
  WTensor<double> SRk_dbl = WTensor<double>::createFromVector(SRk_i32.data());
  // SWk_dbl.print("SWk_dbl: ", PARTIAL_DATA);
  // SRk_dbl.print("SRk_dbl: ", PARTIAL_DATA);
  double Qc_0 = Qx * s.len_x * Zx * Zw + Qh * s.len_h * Zh * Zr + Zg;
  auto Qc_1 = SWk_dbl.mul((-1) * Qx * Zx);
  auto Qc_2 = SRk_dbl.mul((-1) * Qh * Zh);
  // Qc_2.print("Qc_2: ", PARTIAL_DATA);
  // bw_dbl.print("bw_dbl: ", PARTIAL_DATA);
  // br_dbl.print("br_dbl: ", PARTIAL_DATA);
  auto Qc_3 = bw_dbl.add(br_dbl);
  // Qc_3.print("Qc_3: ", PARTIAL_DATA);
  auto Qc_4 = Qc_3.sub(2 * Zb);
  auto Qc_5 = Qc_4.mul(Sb / Sg);
  // Qc_5.print("Qc_5: ", PARTIAL_DATA);
  auto Qc_6 = Qc_1.add(Qc_2);
  // Qc_6.print("Qc_6: ", PARTIAL_DATA);
  auto Qc_7 = Qc_6.add(Qc_5);
  // Qc_7.print("Qc_7: ", PARTIAL_DATA);
  auto Qc_8 = Qc_7.add(Qc_0);
  // Qc_8.print("Qc_8: ", PARTIAL_DATA);
  auto Qc_9 = Qc_8.mul(std::pow(2, s.nonlinear_in_shift));
  // Qc_9.print("Qc_9: ", PARTIAL_DATA);
  WTensor<int32_t> NB_i32 = WTensor<int32_t>::createFromVector(Qc_9.data());
  // NB_i32.print("--- Qc int(NB_i32): ", PARTIAL_DATA);

  dims_t order = {2, 3, 0, 1};
  WTensor<uint8_t> w0_u8 = WTensor<uint8_t>::createFromVector(w_u8);
  WTensor<uint8_t> r0_u8 = WTensor<uint8_t>::createFromVector(r_u8);
  WTensor<uint8_t> w1_u8 =
      w0_u8.reshape({4, 1024, s.len_x}).reorder(order).reshape({4096, s.len_x});
  WTensor<uint8_t> r1_u8 =
      r0_u8.reshape({4, 1024, s.len_h}).reorder(order).reshape({4096, s.len_h});
  WTensor<int32_t> nb_i32 =
      NB_i32.reshape({4, 1024}).reorder(order).reshape({1, 4096});

  // w1_u8.print("--- post reorder w1_u8: ", PARTIAL_DATA);
  // r1_u8.print("--- post reorder r1_u8: ", PARTIAL_DATA);
  // nb_i32.print("--- post reorder nb_i32: ", PARTIAL_DATA);

  // ----------------- write_tvs -----------------
  int max_K = std::max(s.len_x, s.len_h);
  auto w1_u8_t = w1_u8.reshape({4096, s.len_x}).transpose({1, 0});
  auto r1_u8_t = r1_u8.reshape({4096, s.len_h}).transpose({1, 0});
  auto W_pad_u8 = w1_u8_t.pad(
      {{0, max_K - s.len_x},
       {0, 0}}); // .reshape({max_K / s.sv_K, s.sv_K, s.n_iter * s.sv_N});
  auto R_pad_u8 = r1_u8_t.pad(
      {{0, max_K - s.len_h},
       {0, 0}}); // .reshape({max_K / s.sv_K, s.sv_K, s.n_iter * s.sv_N});
  // W_pad_u16.print("W_pad_u16: ", PARTIAL_DATA);
  // R_pad_u16.print("R_pad_u16: ", PARTIAL_DATA);
  // ----------------- write_wts -----------------
  int ssv_N = s.sv_N / 4;
  int K = s.len_kp;
  int sv_K = s.sv_K;

  // auto W_pad_u8 = WTensor<uint8_t>::createFromVector(W_pad_u16.data());
  // auto R_pad_u8 = WTensor<uint8_t>::createFromVector(R_pad_u16.data());

  auto W_reshaped_ = W_pad_u8.reshape(
      {K / sv_K, sv_K, 4, s.n_iter / s.num_row, s.num_row, ssv_N});
  auto W_transposed_u8 = W_reshaped_.transpose({3, 0, 4, 1, 2, 5});
  auto R_reshaped_ = R_pad_u8.reshape(
      {K / sv_K, sv_K, 4, s.n_iter / s.num_row, s.num_row, ssv_N});
  auto R_transposed_u8 = R_reshaped_.transpose({3, 0, 4, 1, 2, 5});
  auto B_reshaped_i32 = nb_i32.reshape({4, s.len_h});
  // W_transposed_u8.print("### W_transposed_u8: ", TYPE_SHAPE);
  // R_transposed_u8.print("### R_transposed_u8: ", TYPE_SHAPE);

  dims_t sv_W_start = {0, 0, 0, 0, 0, 0};
  dims_t sv_W_end = W_transposed_u8.shape();
  unsigned char* wts = result;
  unsigned char* p = wts;
  for (int idx_n_col = 0; idx_n_col < (s.n_iter / s.num_row); idx_n_col++) {
    for (int idx_K = 0; idx_K < (max_K / s.sv_K); idx_K++) {
      for (int idx_row = 0; idx_row < s.num_row; idx_row++) {
        sv_W_start[0] = idx_n_col;
        sv_W_start[1] = idx_K;
        sv_W_start[2] = idx_row;
        sv_W_end[0] = idx_n_col + 1;
        sv_W_end[1] = idx_K + 1;
        sv_W_end[2] = idx_row + 1;
        auto sv_W = W_transposed_u8.slice(sv_W_start, sv_W_end)
                        .reshape({sv_K, 4 * ssv_N});
        auto sv_R = R_transposed_u8.slice(sv_W_start, sv_W_end)
                        .reshape({sv_K, 4 * ssv_N});
        sv_W.reshape({s.sv_K / 8, 8, s.sv_N});
        sv_R.reshape({s.sv_K / 8, 8, s.sv_N});
        dims_t svm_s0 = {0, 0, 0};
        dims_t svm_e0 = sv_W.shape();
        svm_e0[1] = 4;
        dims_t svm_s1 = {0, 0, 0};
        dims_t svm_e1 = sv_W.shape();
        svm_s1[1] = 4;
        svm_e1[1] = 8;
        auto svm0_W = sv_W.slice(svm_s0, svm_e0)
                          .reshape({s.sv_K / 8, 4, s.sv_N / 8, 8})
                          .transpose({2, 0, 1, 3});
        auto svm1_W = sv_W.slice(svm_s1, svm_e1)
                          .reshape({s.sv_K / 8, 4, s.sv_N / 8, 8})
                          .transpose({2, 0, 1, 3});
        auto svm0_R = sv_R.slice(svm_s0, svm_e0)
                          .reshape({s.sv_K / 8, 4, s.sv_N / 8, 8})
                          .transpose({2, 0, 1, 3});
        auto svm1_R = sv_R.slice(svm_s1, svm_e1)
                          .reshape({s.sv_K / 8, 4, s.sv_N / 8, 8})
                          .transpose({2, 0, 1, 3});
        int idx_n_real = idx_n_col * s.num_row + idx_row;
        dims_t svb_s = {0, 0};
        dims_t svb_e = B_reshaped_i32.shape();
        svb_s[1] = idx_n_real * ssv_N;
        svb_e[1] = (idx_n_real + 1) * ssv_N;
        auto sv_B = B_reshaped_i32.slice(svb_s, svb_e);
        memcpy(p, svm0_W.data().data(), svm0_W.size());
        p += svm0_W.size();
        memcpy(p, svm0_R.data().data(), svm0_R.size());
        p += svm0_R.size();
        memcpy(p, svm1_W.data().data(), svm1_W.size());
        p += svm1_W.size();
        memcpy(p, svm1_R.data().data(), svm1_R.size());
        p += svm1_R.size();
        memcpy(p, sv_B.data().data(), sv_B.size() * sizeof(int32_t));
        p += sv_B.size() * sizeof(int32_t);
      }
    }
  }
  printf("####: %s: wrote %d bytes of data\n", s.layer_name.c_str(), p - wts);
  size_t wts_size = p - wts;
  saveMemoryToCache((const char*)wts, wts_size, context, s.layer_name);
  return 0;
}

std::vector<uint8_t>
ht_wts_gen_lstm_b2b(const lstm_init_wts& param,
                    const vaip_core::PassContext& context) {
  uint8_t* result = new uint8_t[8448 * 1024 * 2 + 64 * 2];

#ifdef VAIML_CUSTOM_OP_LOAD_WTS_PROFILING
  auto start2 = std::chrono::steady_clock::now();
#endif
  LstmSettings lstm_s320 = LstmSettings(320);
  htGenerateLstmInput(lstm_s320, param, result + 64 * 2, context);

#ifdef VAIML_CUSTOM_OP_LOAD_WTS_PROFILING
  auto end2 = std::chrono::steady_clock::now();
  double time_sec2 = std::chrono::duration<double>(end2 - start2).count();
  std::cout << "htGenerateLstmInput lstm_s320 time (sec): " << time_sec2
            << std::endl;
#endif

#ifdef VAIML_CUSTOM_OP_LOAD_WTS_PROFILING
  auto start3 = std::chrono::steady_clock::now();
#endif

  LstmSettings lstm_s1024 = LstmSettings(1024);
  htGenerateLstmInput(lstm_s1024, param, result + 8448 * 1024 + 64 * 2,
                      context);

#ifdef VAIML_CUSTOM_OP_LOAD_WTS_PROFILING
  auto end3 = std::chrono::steady_clock::now();
  double time_sec3 = std::chrono::duration<double>(end3 - start3).count();
  std::cout << "htGenerateLstmInput lstm_s1024 time (sec): " << time_sec3
            << std::endl;
#endif

  std::vector<uint8_t> wts(result, result + 8448 * 1024 * 2 + 64 * 2);

  delete[] result;
  return wts;
}

std::vector<uint8_t>
wts_gen_matmul(int8_t* mat_B, uint32_t M, uint32_t K, uint32_t N, uint32_t sv_M,
               uint32_t sv_K, uint32_t sv_N, float s_matA, int64_t zp_matA,
               float s_matB, int64_t zp_matB, float s_matC, int64_t zp_matC,
               float ifm_scale_refactor, float ofm_scale_refactor) {
  // compute QDQ coefficients
  int64_t a_dq_xzero_pt = ((double)(zp_matA + 128) / ifm_scale_refactor);
  int64_t w_dq_xzero_pt = zp_matB + 128;
  int64_t a_q_yzero_pt = ((double)(zp_matC + 128) / ofm_scale_refactor);
  std::vector<int64_t> weights;
  for (int i = 0; i < K * N; i++) {
    weights.push_back((int64_t)mat_B[i] + 128);
  }
  int64_t weights_in_ch = K;
  int32_t matmul_shift = 0;
  float c2_coeff =
      ifm_scale_refactor / ofm_scale_refactor * (s_matA * s_matB / s_matC);
  // find closest shifted int32
  float prev_rel_err = 1e9;
  float float_val = c2_coeff;
  float curr_float_val = c2_coeff;
  float best_float_val = 0.0;
  int16_t shift_val = 0;
  int32_t best_int;
  int32_t closest_curr_int;
  int16_t best_shift_val;
  while (curr_float_val <= 8388607) {
    closest_curr_int = round(curr_float_val);
    float cur_rel_err =
        fabs(float_val - closest_curr_int / pow(2, shift_val)) / float_val;
    if (cur_rel_err < prev_rel_err) {
      prev_rel_err = cur_rel_err;
      best_float_val = (float)(closest_curr_int >> shift_val);
      best_shift_val = shift_val;
      best_int = closest_curr_int;
    }
    curr_float_val *= 2;
    shift_val += 1;
  }
  int64_t c2_coeff_prime = best_int;
  int16_t shft_c2 = best_shift_val;
  // calculate c0, c1, c2
  std::vector<int64_t> c1_coeff;
  for (int i = 0; i < N; i++) {
    int64_t sum_weight = 0;
    for (int j = 0; j < K; j++) {
      sum_weight += weights[i + j * N];
    }
    int64_t tmp = (-a_dq_xzero_pt) * c2_coeff_prime * sum_weight +
                  (int64_t)(a_q_yzero_pt << shft_c2);
    c1_coeff.push_back(tmp);
  }
  int64_t num_weights_unrolled = weights_in_ch;
  int32_t c3_coeff_offset = -a_dq_xzero_pt * num_weights_unrolled;
  int64_t c3_coeff_scale = -c2_coeff_prime * w_dq_xzero_pt;
  int64_t c3_coeff_scale_shift = 0;
  if (abs(c3_coeff_scale) > 2147483647)
    c3_coeff_scale_shift = (int64_t)ceil(log2(fabs(c3_coeff_scale)) - 31);
  else
    c3_coeff_scale_shift = 0;
  c3_coeff_scale = (c3_coeff_scale >> c3_coeff_scale_shift);
  int32_t C2 = c2_coeff_prime << matmul_shift;
  int32_t C1 = (int32_t)c3_coeff_scale;
  std::vector<int64_t> C0;
  for (int i = 0; i < c1_coeff.size(); i++) {
    int64_t tmp = c3_coeff_scale * (c3_coeff_offset << c3_coeff_scale_shift) +
                  c1_coeff[i];
    C0.push_back(tmp);
  }

  // push back the WTS, subvolues of matB + QDQ_params
  std::vector<uint8_t> res;
  for (int idx_half = 0; idx_half < 2; idx_half++) {
    for (int idx_K = 0; idx_K < K / sv_K; idx_K++) {
      for (int idx_N_half = 0; idx_N_half < N / 2 / sv_N; idx_N_half++) {
        // actual index of sub-volume sv_N
        int idx_N = idx_N_half + idx_half * (N / 2 / sv_N);
        int sv_row_offt = idx_K * sv_K;
        int sv_col_offt = idx_N * sv_N;
        // formatted sub-volumes of matB, to (N/8)K(N8)
        // original format is KN
        for (int n = 0; n < sv_N / 8; n++) {
          for (int k = 0; k < sv_K; k++) {
            for (int l = 0; l < 8; l++) {
              // avoid computing address in each iteration for better
              // performance
              int row = sv_row_offt + k;
              int col = sv_col_offt + l + n * 8;
              res.push_back((uint8_t)weights[row * N + col]);
            }
          }
        }
        // qdq_params: sv_N int64_t from c0 + int32_t(c1) + int32_t(c2)
        uint8_t* sv_c0 = (uint8_t*)(&C0[sv_col_offt]);
        for (int q = 0; q < sv_N * sizeof(int64_t); q++) {
          res.push_back(sv_c0[q]);
        }
        res.push_back((uint8_t)(C1 & 0x000000ff));
        res.push_back((uint8_t)((C1 & 0x0000ff00) >> 8));
        res.push_back((uint8_t)((C1 & 0x00ff0000) >> 16));
        res.push_back((uint8_t)((C1 & 0xff000000) >> 24));
        res.push_back((uint8_t)(C2 & 0x000000ff));
        res.push_back((uint8_t)((C2 & 0x0000ff00) >> 8));
        res.push_back((uint8_t)((C2 & 0x00ff0000) >> 16));
        res.push_back((uint8_t)((C2 & 0xff000000) >> 24));
      }
    }
  }

  return res;
}

} // namespace vaip_vaiml_custom_op
