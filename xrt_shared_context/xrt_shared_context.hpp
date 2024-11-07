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
#include "vitis/ai/env_config.hpp"
#include "vitis/ai/weak.hpp"
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>
#include <xir/graph/graph.hpp>
#ifdef __GNUC__
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wpedantic"
#  pragma GCC diagnostic ignored "-Wconversion"
#endif
#include <xrt/xrt_bo.h>
#include <xrt/xrt_device.h>
#include <xrt/xrt_kernel.h>
#ifdef __GNUC__
#  pragma GCC diagnostic pop
#endif

// header only due to some complicated linkage
namespace vaip {
using vaip_pass_context_t = vaip_core::PassContext;
#ifdef __GNUC__
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wpedantic"
#endif
DEF_ENV_PARAM_2(XLNX_VART_FIRMWARE, "", std::string)
#ifdef __GNUC__
#  pragma GCC diagnostic pop
#endif
class Device {
public:
  Device(int device_id) : device_id_(device_id) {
    xrt_device_ = std::make_unique<xrt::device>(device_id);
  }

  void register_xclbin(const xrt::xclbin& xclbin) {
    // return value ignored
    xrt_device_->register_xclbin(xclbin);
  }

  xrt::device& xrt_device() { return *xrt_device_; }

private:
  std::unique_ptr<xrt::device> xrt_device_;
  int device_id_ = 0;
};

class Xclbin {
public:
  Xclbin(const vaip_pass_context_t& vaip_pass_context, int device_id,
         const std::string& xclbin_file = ENV_PARAM(XLNX_VART_FIRMWARE))
      : xclbin_file_(xclbin_file) {
    auto path_xclbin_file = std::filesystem::path(xclbin_file);
    auto path_xclbin_file_name = path_xclbin_file.filename();
    auto xclbin_context = vaip_pass_context.read_xclbin(path_xclbin_file_name);
    if (xclbin_context.has_value()) {
      // xclbin::xclbin(vector<char>) force we copy xclbin content.
      auto xclbin_context2 = std::vector<char>(xclbin_context.value().begin(),
                                               xclbin_context.value().end());
      xrt_xclbin_ = std::make_unique<xrt::xclbin>(xclbin_context2);
    } else {
      xrt_xclbin_ = std::make_unique<xrt::xclbin>(xclbin_file);
    }
    // device id uniquely determine device
    shared_xrt_device_ = vitis::ai::WeakStore<std::string, Device>::create(
        std::to_string(device_id), device_id);
    shared_xrt_device_->register_xclbin(*xrt_xclbin_);
  }

  xrt::xclbin& xrt_xclbin() { return *xrt_xclbin_; }

  std::shared_ptr<Device> get_shared_device() { return shared_xrt_device_; }

private:
  std::shared_ptr<Device> shared_xrt_device_;
  std::unique_ptr<xrt::xclbin> xrt_xclbin_;
  std::string xclbin_file_;
};

class Context {
public:
  static constexpr int MAX_NUM_OF_CONTEXTS = 8;
  static constexpr const char* CTX_SHARE_OPTION_KEY = "share_context";

  Context() {
    // create attribute store
    attrs_ = xir::Attrs::create();
    // reserve some space for kernels
    xrt_kernels_.reserve(512);
  }

  // note: parameter order has been adjusted, device_id first, context id second
  Context(const vaip_pass_context_t& vaip_pass_context, int device_id,
          int context_id, std::map<std::string, std::uint32_t>& qos,
          const std::string& xclbin_file = ENV_PARAM(XLNX_VART_FIRMWARE))
      : device_id_(device_id), context_id_(context_id),
        xclbin_file_(xclbin_file) {

    feature_qos_update_ = false;
    init_qos_ = true;
    support_eff_mode_ = true;
    XrtFeatureCheck();

    // create(or get) shared xclbin from weakstore
    // with base name of xclbin as key, without the path
    auto base_name = get_basename(xclbin_file);
    shared_xrt_xclbin_ = vitis::ai::WeakStore<std::string, Xclbin>::create(
        base_name, vaip_pass_context, device_id, xclbin_file);

    auto xrt_device = shared_xrt_xclbin_->get_shared_device();

    // create xrt context
    if (qos.empty()) {
      xrt_hw_context_ = std::make_unique<xrt::hw_context>(
          xrt_device->xrt_device(),
          shared_xrt_xclbin_->xrt_xclbin().get_uuid());
    } else {
      try {
        xrt_hw_context_ = std::make_unique<xrt::hw_context>(
            xrt_device->xrt_device(),
            shared_xrt_xclbin_->xrt_xclbin().get_uuid(), qos);
      } catch (std::exception& e) {

        if (std::string(e.what()).find("perf_pref") != std::string::npos) {
          LOG(WARNING) << "XRT device doesn't support efficient mode, will "
                          "ignore the QoS request.";
          qos.erase("perf_pref");
          xrt_hw_context_ = std::make_unique<xrt::hw_context>(
              xrt_device->xrt_device(),
              shared_xrt_xclbin_->xrt_xclbin().get_uuid(), qos);
          support_eff_mode_ = false;
        } else {
          throw;
        }
      }
    }

    // create attribute store
    attrs_ = xir::Attrs::create();
    // reserve some space for kernels
    xrt_kernels_.reserve(512);

    // set attributes
    attrs_->set_attr<int>("device_id", device_id_);
    attrs_->set_attr<xrt::device*>("xrt_device", &xrt_device->xrt_device());
    attrs_->set_attr<xrt::xclbin*>("xrt_xclbin",
                                   &shared_xrt_xclbin_->xrt_xclbin());
    attrs_->set_attr<xrt::hw_context*>("xrt_hw_context", xrt_hw_context_.get());
  }

  // note: parameter order has been adjusted, device_id first, context id second
  static std::shared_ptr<Context>
  create_shared_context(const vaip_pass_context_t& vaip_pass_context,
                        int device_id, int context_id,
                        const std::string& xclbin_file,
                        std::map<std::string, std::uint32_t> qos = {}) {
    // creating shared context with weakstore
    // using context id and xclbin file base name as key
    // not considering device id now since there's only 1 device for NPU
    auto base_name = get_basename(xclbin_file);
    auto key = std::to_string(context_id) + base_name;
    return vitis::ai::WeakStore<std::string, Context>::create(
        key, vaip_pass_context, device_id, context_id, qos, xclbin_file);
  }

  xir::Attrs* get_attrs() { return attrs_.get(); }

  void create_kernel(const std::string name, const std::string kname) {
    if (!attrs_->has_attr(name.c_str())) {
      try {
        // create kernel and save in vector
        xrt_kernels_.emplace_back(xrt_hw_context(), kname);
        // set attr for kernel
        attrs_->set_attr<xrt::kernel*>(name, &(xrt_kernels_.back()));
      } catch (std::exception& e) {
        LOG(FATAL) << "Error creating kernel: " << e.what();
      }
    }
  }

  xrt::kernel* get_kernel(int index = 0) { return &(xrt_kernels_[index]); }
  // This function originates from GE, as referenced in the following link.
  // https://gitenterprise.xilinx.com/VitisAI/graph-engine/blob/dev/src/graph-engine/graph_runner.cpp#L128
  std::string get_xclbin_kernelName() {
    auto xclbin = attrs_->get_attr<xrt::xclbin*>("xrt_xclbin");
    std::string kernel_name;
    bool find = false;
    auto xclbin_kernels = xclbin->get_kernels();
    for (auto kernel : xclbin_kernels) {
      if (kernel.get_name().find("DPU") != std::string::npos) {
        kernel_name = kernel.get_name();
        find = true;
        break;
      }
    }
    if (!find)
      throw std::runtime_error(
          "Couldn't find a correct kernel that contains DPU ");
    return kernel_name;
  }

  xrt::hw_context& xrt_hw_context() {
    CHECK(xrt_hw_context_) << "hw_context doesn't exist!";
    return *xrt_hw_context_;
  }
  // This function logic basically originates from GE, as referenced in the
  // following link.
  // https://gitenterprise.xilinx.com/VitisAI/graph-engine/blob/c4e1132b0c9d05a47dab3175d9dc9d6ed878522b/src/graph-engine/graph_runner.hpp#L96-L108
  void update_qos(std::map<std::string, std::uint32_t> qos) {

    merge_qos_ = qos;
    if (init_qos_) {
      latency_ = qos["latency"];
      gops_ = qos["gops"];
      init_qos_ = false;
      return;
    }

    if (qos.count("gops")) {
      gops_ += qos["gops"];
      merge_qos_["gops"] = gops_;
    }
    if (qos.count("latency")) {
      latency_ += qos["latency"];
      merge_qos_["latency"] = latency_;
    }
    if (feature_qos_update_) {
      xrt_hw_context_->update_qos(merge_qos_);
    }
    return;
  }

private:
  static std::string get_basename(const std::string& full_path) {
    std::filesystem::path xclbin_full_path(full_path);
    // enable cache in mem , cannot check this file is exist
    // CHECK(std::filesystem::exists(xclbin_full_path))
    //    << "specified xclbin doesn't exist: " << full_path;
    return xclbin_full_path.filename().string();
  }
  // This function originates from GE, as referenced in the following link.
  // https://gitenterprise.xilinx.com/VitisAI/graph-engine/blob/c4e1132b0c9d05a47dab3175d9dc9d6ed878522b/src/graph-engine/graph_runner.hpp#L251
  void XrtFeatureCheck() {
#ifdef _WIN32
    auto h = ::LoadLibraryExA("xrt_coreutil.dll", NULL,
                              LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);
    if (h != NULL) {
      auto p = ::GetProcAddress(h, "xrtVersionFeature");
      if (p != NULL) {
        feature_qos_update_ =
            true; // All XRT versions that support the xrtVersionFeature API
                  // also support the update_qos API
      }
      ::FreeLibrary(h);
    }
#endif
  }

  int device_id_;
  int context_id_;
  std::uint32_t latency_;
  std::uint32_t gops_;
  bool feature_qos_update_;
  bool init_qos_;
  bool support_eff_mode_;
  std::string xclbin_file_;
  std::shared_ptr<Xclbin> shared_xrt_xclbin_;
  std::unique_ptr<xrt::hw_context> xrt_hw_context_;
  std::vector<xrt::kernel> xrt_kernels_;
  std::unique_ptr<xir::Attrs> attrs_;
  std::map<std::string, std::uint32_t> merge_qos_;
};
} // namespace vaip