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
#ifndef HW_RUNNER_H
#  define HW_RUNNER_H
#  pragma once
#  include <cstdlib>
#  include <filesystem>
#  include <fstream>

// AIE Driver headers
#  include "xaiengine.h"

#  include "experimental/xrt_kernel.h"
#  include "xrt/xrt_bo.h"
#  include "xrt/xrt_device.h"
#  include "xrt/xrt_kernel.h"
enum KERNEL_NM { GT_CONV = 0, GT_MM = 1, HT = 2 };
#  include "op_buf.hpp"
#  define HW_RUNNER_USE_CMDLIST
constexpr std::uint64_t DDR_AIE_ADDR_OFFSET = std::uint64_t{0x80000000};
constexpr std::uint64_t OPCODE = std::uint64_t{2};
// namespace vaip_vaiml_custom_op {
enum BO_ORDER {
  WTS_IFM_OFM,
  WTS_IFM_TMP_OFM,
  WTS_OFM_TMP_IFM,
  ODR_GT_CONV,
  ODR_GT_HEAD,
  ODR_GT_TRANSFORMER,
  ODR_GT_TAIL,
  ODR_HT
};
class xrt_context {
protected:
  xrt::device device_;
  xrt::xclbin xclbin_;
  xrt::hw_context context_;
  // xrt::kernel kernel_;
  // xrt::kernel kernel0_;
  std::vector<xrt::kernel> kernel_vec_;

protected:
  xrt_context(const std::vector<char>& xclbin) {
    unsigned int device_index = 0;
    device_ = xrt::device(device_index);
    xclbin_ = xrt::xclbin(xclbin);

    device_.register_xclbin(xclbin_);
    context_ = xrt::hw_context(device_, xclbin_.get_uuid());
    kernel_vec_.emplace_back(xrt::kernel(context_, "GT_CONV"));
    kernel_vec_.emplace_back(xrt::kernel(context_, "GT_MM"));
    kernel_vec_.emplace_back(xrt::kernel(context_, "HT"));
  }

public:
  static xrt_context& get_instance(const std::vector<char>& xclbin) {
    static xrt_context ctx_(xclbin);
    return ctx_;
  }

  xrt_context(const xrt_context&) = delete;
  xrt_context(const xrt_context&&) = delete;
  xrt_context& operator=(const xrt_context&) = delete;
  xrt_context& operator=(const xrt_context&&) = delete;

  xrt::device& get_device() { return device_; }
  xrt::hw_context& get_context() { return context_; }
  xrt::kernel& get_kernel(uint32_t idx = 0) { return kernel_vec_[idx]; }
};

struct XRTRunOffset {
  size_t ifm_offset = 0;
  size_t wts_offset = 0;
  size_t ofm_offset = 0;
  size_t tmp_offset = 0;
  XRTRunOffset(size_t i_off, size_t w_off, size_t o_off, size_t t_off)
      : ifm_offset(i_off), wts_offset(w_off), ofm_offset(o_off),
        tmp_offset(t_off) {}
  XRTRunOffset() {}
};
class hw_runner {
public:
  hw_runner(){};
  // Constructor for hw_runner

  // Destructor
  ~hw_runner(){};
  void set_bo_order(BO_ORDER order);
  void set_bo_order_vec(vector<BO_ORDER> order);
  //  void hw_runner_init(const std::string& xclbin_filename, uint32_t ifm_size,
  //                      uint32_t wts_size, uint32_t ofm_size, uint32_t
  //                      tmp_size, bool gt_mode);
  void hw_runner_init(uint32_t ifm_size, uint32_t wts_size, uint32_t ofm_size,
                      uint32_t tmp_size, bool gt_mode,
                      const std::vector<XRTRunOffset>& run_offets,
                      const std::vector<KERNEL_NM>& kernel_index);

  void load_xclbin(const std::vector<char>& xclbin);
  void load_txn_bin(const std::string& txnbin_filename);
  void load_txn_bin(const std::vector<std::string>& txnbin_filenames);

  void load_ctrl_pkt_bin(const std::vector<std::string>& ctrlpkt_filenames);

  // Performs the run on hardware
  int run(void* ifm, void* wts, void* ofm);

  void get_bo_ptrs(int8_t*& ifm_ptr, int8_t*& wts_ptr, int8_t*& ofm_ptr);
  void pre_run_bo_sync();

private:
  BO_ORDER bo_order_ = BO_ORDER::WTS_IFM_TMP_OFM;
  std::vector<BO_ORDER> bo_order_vec_ = {};
  std::vector<xrt::run> run_obj_vec_;
  xrt::bo* logical_ofm_bo_ = nullptr;
#  ifdef HW_RUNNER_USE_CMDLIST
  std::vector<xrt::runlist> runlist_wrapped_;
#  endif
  xrt_context* context_;
  uint32_t ifm_size_;
  uint32_t wts_size_;
  uint32_t ofm_size_;
  uint32_t tmp_size_;

  std::string ofm_filename_;

  std::vector<xrt::bo> instr_bo_vec_;
  std::vector<xrt::bo> ctrl_pkt_bo_vec_;

  int8_t* ifm_ptr_;
  int8_t* wts_ptr_;
  int8_t* ofm_ptr_;
  int8_t* tmp_ptr_;
  xrt::bo ifm_bo_;
  xrt::bo wts_bo_;
  xrt::bo ofm_bo_;
  xrt::bo tmp_bo_;
};

#endif // HW_RUNNER_H

       // } // namespace