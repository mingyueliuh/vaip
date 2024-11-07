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

#pragma once
#include <iostream>
#include <memory>
#include <string>

#include "vart/runner_ext.hpp"
#include "xrt/xrt_device.h"
#include "xrt/xrt_kernel.h"
#include <glog/logging.h>
#include <xir/attrs/attrs.hpp>
#include <xrt/xrt_bo.h>

namespace vaip_core {
// XRTUpdateQosImpl class
// update efficient mode directly through xrt::hw_context
class XRTUpdateQosImpl : public QoSUpdateInterface {
public:
  explicit XRTUpdateQosImpl(xrt::hw_context* context)
      : hw_context_(context), support_eff_mode_(true) {}

  void update_qos(const std::string& perf_pref_value) override {
    if (hw_context_) {
      xrt::hw_context::qos_type qos_map_perf;
      qos_map_perf["perf_pref"] = (perf_pref_value == "Efficient") ? 1 : 0;
      std::lock_guard<std::mutex> lock(support_eff_mode_lock);
      try {

        if (support_eff_mode_) {
          hw_context_->update_qos(qos_map_perf);
        }
      } catch (std::exception& e) {
        if (std::string(e.what()).find("perf_pref") != std::string::npos) {
          LOG(WARNING) << "XRT device doesn't support efficient mode, will "
                          "ignore the QoS request.";
          support_eff_mode_ = false;
        } else {
          throw;
        }
      }
    } else {
      LOG(WARNING)
          << "Error: hw_context_ is null in XRTUpdateQosImpl::update_qos";
    }
  }

private:
  xrt::hw_context* hw_context_;
  bool support_eff_mode_;
  mutable std::mutex support_eff_mode_lock;
};

// GEUpdateQosImpl class
// update efficient mode through set_run_attrs api of vart runner
class GEUpdateQosImpl : public QoSUpdateInterface {
public:
  explicit GEUpdateQosImpl(vart::RunnerExt* runner)
      : runner_(runner), support_eff_mode_(true) {}

  void update_qos(const std::string& perf_pref_value) override {
    if (runner_) {
      std::lock_guard<std::mutex> lock(support_eff_mode_lock);
      try {
        if (support_eff_mode_) {
          std::shared_ptr<xir::Attrs> attrs = xir::Attrs::create();
          if (perf_pref_value == "Default") {
            attrs->set_attr<std::string>("performance_preference", "Default");
          } else {
            attrs->set_attr<std::string>("performance_preference",
                                         "HighEfficiencyMode");
          }
          auto unique_attrs = xir::Attrs::clone(attrs.get());
          runner_->set_run_attrs(unique_attrs);
        }
      } catch (std::exception& e) {
        if (std::string(e.what()).find("perf_pref") != std::string::npos) {
          LOG(WARNING) << "XRT device doesn't support efficient mode, will "
                          "ignore the update efficient mode through "
                          "set_run_attrs method in dpu custom op.";
          support_eff_mode_ = false;
        } else {
          LOG(FATAL) << "-- Error: Failed to update efficient mode through "
                        "set_run_attrs method in dpu custom op: "
                     << e.what();
        }
      }
    } else {
      LOG(WARNING) << "Error: runner_ is null in GEUpdateQosImpl::update_qos";
    }
  }

private:
  vart::RunnerExt* runner_;
  bool support_eff_mode_;
  mutable std::mutex support_eff_mode_lock;
};
} // namespace vaip_core
