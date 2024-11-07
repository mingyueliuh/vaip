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
#define _CRT_SECURE_NO_WARNINGS
#include <fstream>
#include <google/protobuf/util/json_util.h>

#include "vaip/mem_xclbin.hpp"
#include "vaip/util.hpp"

#include "pass_context_imp.hpp"
#include "profile_utils.hpp"
#include "tar_ball.hpp"

DEF_ENV_PARAM(DEBUG_TAR_CACHE, "0")

namespace vaip_core {

/// struct WithPass
PassContextImp::WithPass::WithPass(PassContextImp& context, IPass& pass)
    : _context(&context) {
  _context->current_pass_stack.push_back(&pass);
}
PassContextImp::WithPass::~WithPass() {
  _context->current_pass_stack.pop_back();
}

/// static
static MemUsageProto convert_to_chrome_event(const MemUsageProto& mem_usage) {
  auto ret = MemUsageProto();
  {
    std::stringstream stream;
    stream << std::hex << mem_usage.current_memory_in_bytes();
    ret.set_current_memory(stream.str());
  }
  {
    std::stringstream stream;
    stream << std::hex << mem_usage.peak_memory_in_bytes();
    ret.set_peak_memory(stream.str());
  }
  return ret;
}
static std::vector<char>
read_file_to_buffer(const std::filesystem::path& path) {
  std::ifstream is(path, std::ios::binary);
  CHECK(is.good());
  CHECK(is.seekg(0, std::ios_base::end).good());
  auto size = is.tellg();
  CHECK_NE(size, -1);
  CHECK(is.seekg(0, std::ios_base::beg).good());
  auto buffer = std::vector<char>((size_t)size);
  CHECK(is.read(buffer.data(), size).good());
  return buffer;
}
static FILE* write_to_tmp_file(gsl::span<const char> data) {
#if _WIN32
  FILE* tmp_file = nullptr;
  auto err = tmpfile_s(&tmp_file);
  CHECK_EQ(err, 0) << "tmpfile_s error";
#else
  FILE* tmp_file = tmpfile();
  CHECK(tmp_file != nullptr) << "cannot create tmp file";
#endif
  auto write_size = std::fwrite(data.data(), 1, data.size(), tmp_file);
  CHECK_EQ((size_t)write_size, data.size());
  return tmp_file;
}

static std::string msg_to_json_string(const google::protobuf::Message& msg) {
  google::protobuf::util::JsonPrintOptions options;
  options.add_whitespace = true;
  auto json_str = std::string();
  auto status =
      google::protobuf::util::MessageToJsonString(msg, &json_str, options);
  CHECK(status.ok()) << "cannot write json string:" << msg.DebugString();
  return json_str;
}

/// struct PassContextImp
int PassContextImp::allocate_suffix()
    const { // it is not a big deal to update suffix_counter
  suffix_counter = suffix_counter + 1;
  return suffix_counter;
}

PassContextImp::WithPass PassContextImp::with_current_pass(IPass& pass) {
  return WithPass(*this, pass);
}

std::filesystem::path PassContextImp::get_log_dir() const { return log_dir; }
std::optional<std::string>
PassContextImp::get_provider_option(const std::string& option_name) const {
  const auto& config = context_proto.config();
  auto it = config.provider_options().find(option_name);
  if (it != config.provider_options().end()) {
    return it->second;
  }
  return std::nullopt;
}

std::optional<std::string>
PassContextImp::get_session_config(const std::string& option_name) const {
  const auto& config = context_proto.config();
  auto it = config.session_configs().find(option_name);
  if (it != config.session_configs().end()) {
    return it->second;
  }
  return std::nullopt;
}
std::string
PassContextImp::get_provider_option(const std::string& option_name,
                                    const std::string& default_value) const {
  auto option_value = get_provider_option(option_name);
  if (option_value.has_value()) {
    return option_value.value();
  }
  return default_value;
}

bool PassContextImp::cache_in_mem() const {
  auto context_enable_option = this->get_session_config("ep.context_enable");
  bool cache_inside_model = context_enable_option.has_value() &&
                            (context_enable_option.value() == "1");
  bool use_cache_model = cache_inside_model || is_ep_context_model;
  if (!use_cache_model) {
#ifdef WIN24_BUILD
    // session option set
    if (cache_dir_set) {
      return false;
    }
    return get_config_proto().enable_cache_file_io_in_mem();
#else
    return false;
#endif
  } else {
    return get_config_proto().enable_cache_file_io_in_mem();
  }
}

void PassContextImp::set_is_ep_context_model(bool is_ep_context_model) {
  this->is_ep_context_model = is_ep_context_model;
}

bool PassContextImp::get_is_ep_context_model() {
  return this->is_ep_context_model;
}

int64_t PassContextImp::get_provider_option_i64(const std::string& option_name,
                                                int64_t default_value) const {
  auto config_value = get_provider_option(option_name);
  auto ret = default_value;
  if (config_value.has_value()) {
    ret = std::stoll(config_value.value());
  } else {
    ret = default_value;
  }
  return ret;
}

std::string
PassContextImp::get_session_config(const std::string& option_name,
                                   const std::string& default_value) const {
  auto option_value = get_session_config(option_name);
  if (option_value.has_value()) {
    return option_value.value();
  }
  return default_value;
}

extern thread_local const void* g_state;
extern thread_local vaip_core::DllSafe<std::string> (*g_get_config_entry)(
    const void* state, const char* entry_name);
std::string
PassContextImp::get_run_option(const std::string& option_name,
                               const std::string& default_value) const {
  // set the default value.
  std::string ret = default_value;
  if (g_state) {
    auto dll_string = g_get_config_entry(g_state, option_name.data());
    if (dll_string.get() != nullptr) {
      ret = std::string(*dll_string);
    }
    return ret;
  }
  return ret;
}

std::string
PassContextImp::get_ep_dynamic_option(const std::string& option_name,
                                      const std::string& default_value) const {
  std::lock_guard<std::mutex> lock(this->ep_dynamic_options_lock);
  auto it = ep_dynamic_options.find(option_name);
  if (it == ep_dynamic_options.end()) {
    return default_value;
  } else {
    return it->second;
  }
}

void PassContextImp::add_QosUpdater(
    const std::shared_ptr<QoSUpdateInterface>& updater) const {
  CHECK(updater) << "Null QoS updater cannot be added to PassContext";
  qos_updaters_.push_back(updater);
}

void PassContextImp::update_all_qos(const std::string& workload_type) const {
  if (workload_type == "Efficient" || workload_type == "Default") {
    for (const auto& updater : qos_updaters_) {
      CHECK(updater) << "Found null QoS updater in qos_updaters_";
      updater->update_qos(workload_type);
    }
  } else {
    throw std::runtime_error("Invalid workload type: " + workload_type);
  }
}

template <typename char_type> struct binary_io {
  static std::vector<char_type> slurp_binary(FILE* file) {
    CHECK(fseek64(file, 0, SEEK_SET) == 0);
    CHECK(fseek64(file, 0, SEEK_END) == 0);
    auto size = ftell64(file);
    CHECK(fseek64(file, 0, SEEK_SET) == 0);
    auto buffer = std::vector<char_type>((size_t)size / sizeof(char_type));
    if (size != 0) {
      CHECK(fread(buffer.data(), 1, size, file) == static_cast<size_t>(size));
    }
    return buffer;
  }
};

template <typename T>
std::optional<std::vector<T>>
PassContextImp::read_file_generic(const std::string& filename) const {
  std::optional<std::vector<T>> ret;
  auto stream = open_file_for_read(filename);
  if (stream == nullptr) {
    return std::nullopt;
  }
  constexpr size_t buffer_size = 8196;
  char tmp[buffer_size];
  ret = std::vector<T>();
  ret.value().reserve(buffer_size);
  size_t read_count = 0;
  do {
    read_count = stream->fread(&tmp, buffer_size);
    ret.value().insert(ret.value().end(), tmp, tmp + read_count);
  } while (read_count != 0);
  LOG_IF(FATAL, !ret.has_value())
      << "can't read " << filename << " in the cache object.";
  return ret;
}
std::optional<std::vector<char>>
PassContextImp::read_file_c8(const std::string& filename) const {
  return read_file_generic<char>(filename);
}

std::optional<std::vector<uint8_t>>
PassContextImp::read_file_u8(const std::string& filename) const {
  return read_file_generic<uint8_t>(filename);
}

std::unique_ptr<CacheFileReader>
PassContextImp::open_file_for_read(const std::string& filename) const {
  std::unique_ptr<CacheFileReader> ret = nullptr;
  auto in_mem = cache_in_mem();
  auto& cace_files =
      const_cast<std::remove_cv_t<decltype(cache_files_)&>>(cache_files_);
  auto it = cace_files.find(filename);
  if (it != cace_files.end()) {
    if (in_mem) {
      LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE))
          << "tmp file opened: " << filename;
      ret = std::unique_ptr<CacheFileReader>(
          new CacheFileReaderImp(in_mem, filename, it->second));
    } else {
      FILE* fp = std::freopen((get_log_dir() / filename).u8string().c_str(),
                              "rb+", it->second);
      if (fp == nullptr) {
        LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE))
            << " cannot freopen " << filename;
      } else {
        it->second = fp;
        ret = std::unique_ptr<CacheFileReader>(
            new CacheFileReaderImp(in_mem, filename, it->second));
      }
    }
  } else {
    if (!in_mem) {
      FILE* fp =
          std::fopen((get_log_dir() / filename).u8string().c_str(), "rb+");
      if (fp == nullptr) {
        LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE))
            << " cannot freopen " << filename;
      } else {
        cace_files[filename] = fp;
        ret = std::unique_ptr<CacheFileReader>(
            new CacheFileReaderImp(in_mem, filename, fp));
      }
    } else {
      LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE))
          << "tmp file open failed: cannot found " << filename
          << ". try to use write_file_for_write before reading.";
      ret = nullptr;
    }
  }
  return ret;
}

std::unique_ptr<CacheFileWriter>
PassContextImp::open_file_for_write(const std::string& filename) {
  std::unique_ptr<CacheFileWriter> ret = nullptr;
  auto it = cache_files_.find(filename);
  FILE* tmp_file = nullptr;
  auto in_mem = cache_in_mem();
  if (it != cache_files_.end()) {
    if (in_mem) {
      fclose(it->second);
      LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE))
          << "tmp file write: " << filename;
      tmp_file = tmpfile();
      if (tmp_file == nullptr) {
        LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE))
            << " cannot create tmp file " << filename;
      } else {
        it->second = tmp_file;
        return std::unique_ptr<CacheFileWriter>(
            new CacheFileWriterImp(in_mem, filename, it->second));
      }
    } else {
      FILE* fp = std::freopen((get_log_dir() / filename).u8string().c_str(),
                              "wb+", it->second);
      if (fp == nullptr) {
        LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE))
            << " cannot freopen " << filename;
      } else {
        it->second = fp;
        return std::unique_ptr<CacheFileWriter>(
            new CacheFileWriterImp(in_mem, filename, fp));
      }
    }
  } else {
    if (in_mem) {
      LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE))
          << "tmp file write: " << filename;
      tmp_file = tmpfile();
      if (tmp_file == nullptr) {
        LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE))
            << "cannot create tmp file" << filename;
      } else {
        cache_files_[filename] = tmp_file;
        ret = std::unique_ptr<CacheFileWriter>(
            new CacheFileWriterImp(in_mem, filename, tmp_file));
      }
    } else {
      tmp_file =
          std::fopen((get_log_dir() / filename).u8string().c_str(), "wb+");
      if (tmp_file == nullptr) {
        LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE))
            << " fopen failed. " << filename;
      } else {
        cache_files_[filename] = tmp_file;
        ret = std::unique_ptr<CacheFileWriter>(
            new CacheFileWriterImp(in_mem, filename, tmp_file));
      }
    }
  }
  return ret;
}

FILE* PassContextImp::open_file(const std::string& filename) const {
  auto it = cache_files_.find(filename);
  if (it != cache_files_.end()) {
    LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE)) << "tmp file opened: " << filename;
    return it->second;
  }
  LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE))
      << "tmp file open failed: " << filename;
  return nullptr;
}

bool write_to_cache_files(std::map<std::string, FILE*>& cache_files,
                          const std::string& filename,
                          gsl::span<const char> data) {
  auto iter = cache_files.find(filename);
  if (iter != cache_files.end()) {
    fclose(iter->second);
  }
  cache_files[filename] = write_to_tmp_file(data);
  return true;
}
bool PassContextImp::write_file(const std::string& filename,
                                gsl::span<const char> data) {
  bool ret = true;
  auto stream = open_file_for_write(filename);
  CHECK(stream != nullptr) << "cannot open " << filename << " for write";
  if (!data.empty()) {
    CHECK(stream->fwrite(data.data(), data.size()) == data.size())
        << "failed to write " << filename;
  }
  stream = nullptr; // close file
  LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE))
      << "write " << filename << " " << data.size()
      << " bytes to the cache files";
  return ret;
}

bool PassContextImp::has_cache_file(const std::string& filename) const {
  return cache_files_.find(filename) != cache_files_.end();
}

std::vector<char> PassContextImp::cache_files_to_tar_mem() {
  std::ostringstream buf(std::ios::binary);
  for (const auto& iter : cache_files_) {
    auto file_content = read_file_c8(iter.first).value();
    tarball_write_file(buf, iter.first, file_content);
  }
  tarball_end(buf);
  auto str = buf.str();
  std::vector<char> ret(str.begin(), str.end());
  return ret;
}

bool PassContextImp::cache_files_to_tar_file(
    const std::filesystem::path& tar_file) const {
  std::ofstream tar_stream(tar_file, std::ios::binary);
  CHECK(tar_stream.good()) << "cannot open tar ball " << tar_file;
  for (auto& [filename, file] : cache_files_) {
    auto data = read_file_c8(filename).value();
    LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE))
        << "write " << filename << " " << data.size() << " bytes";
    tarball_write_file(tar_stream, filename, data);
  }
  tarball_end(tar_stream);
  return true;
}
bool PassContextImp::tar_file_to_cache_files(
    const std::filesystem::path& tar_file) {
  auto tar_mem = vaip_core::slurp_binary_c8(tar_file);
  tar_mem_to_cache_files(tar_mem.data(), tar_mem.size());
  return true;
}
bool PassContextImp::tar_mem_to_cache_files(const char* buffer, size_t size) {
  auto p = buffer;
  for (;;) {
    auto [filename, data] = tarball_read_file_from_memory(p, size);
    if (filename.empty()) {
      break;
    }
    LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE))
        << "load " << filename << " " << data.size() << " bytes";
    gsl::span<char> data_span = gsl::span<char>(data);
    write_file(filename, data_span);
  }
  return true;
}

std::filesystem::path PassContextImp::xclbin_path_to_cache_files(
    const std::filesystem::path& path) const {
  auto filename = path.filename().u8string();
  auto ret = get_log_dir() / filename;

  bool in_mem = cache_in_mem();
  // already done
  if (in_mem && has_cache_file(filename)) {
    return ret;
  } else if ((!in_mem) && std::filesystem::exists(ret)) {
    return ret;
  }

  std::vector<char> buffer;
  if (has_mem_xclbin(filename)) {
    buffer = get_mem_xclbin(filename);
  } else if (std::filesystem::exists(path)) {
    buffer = read_file_to_buffer(path);
  } else {
    LOG(WARNING)
        << "Xclbin path doesn't exist, are you running with cpu runner?";
    return path;
  }
  const_cast<PassContextImp*>(this)->write_file(filename, buffer);
  return ret;
}

std::optional<std::vector<char>>
PassContextImp::read_xclbin(const std::filesystem::path& path) const {
  auto filename = path.filename().u8string();
  return read_file_c8(filename);
}

const ConfigProto& PassContextImp::get_config_proto() const {
  return context_proto.config();
}

void PassContextImp::save_context_json() const {
  ContextProto proto;
  proto.CopyFrom(this->context_proto);
  proto.mutable_config()->clear_encryption_key();
  try {
    auto json_str = msg_to_json_string(proto);
    const_cast<PassContextImp*>(this)->write_file("context.json", json_str);
  } catch (const std::exception& e) {
    std::cerr << "exception occurs : " << e.what() << "\n";
  }
}

void PassContextImp::add_context_resource(const std::string& name,
                                          std::shared_ptr<void> resource) {
  pass_resources[name] = resource;
}

std::shared_ptr<void>
PassContextImp::get_context_resource(const std::string& name) const {
  auto it = pass_resources.find(name);
  auto ret = std::shared_ptr<void>();
  if (it != pass_resources.end()) {
    ret = it->second;
  }
  return ret;
}

std::unique_ptr<PassContextTimer>
PassContextImp::measure(const std::string& label) {
  return std::unique_ptr<PassContextTimer>(
      new PassContextTimerImp(label, *this));
}

/// struct PassContextTimerImp
PassContextTimerImp::PassContextTimerImp(const std::string& label,
                                         PassContextImp& context)
    : PassContextTimer(), label_{label}, context_{context},
      start_{std::chrono::steady_clock::now()}, mem_usage_{GetMemUsage()} {}
PassContextTimerImp::~PassContextTimerImp() {
  auto end_tp = std::chrono::steady_clock::now();
  auto end_mem_usage = GetMemUsage();
  auto event = context_.context_proto.mutable_events()->Add();
  int64_t thead_id = vaip_core::get_tid();
  int64_t process_id = vaip_core::get_pid();
  event->set_name(label_);
  event->set_ph("X");
  event->set_pid(process_id);
  event->set_tid(thead_id);
  auto start = std::chrono::duration_cast<std::chrono::microseconds>(
                   start_ - context_.start_)
                   .count();
  event->set_ts(start);
  auto interval =
      std::chrono::duration_cast<std::chrono::microseconds>(end_tp - start_)
          .count();
  event->set_dur(interval);
  *event->mutable_args()->mutable_mem_usage() = mem_usage_;
  event->mutable_args()->mutable_mem_usage()->set_current_memory_in_bytes(
      end_mem_usage.current_memory_in_bytes() -
      event->args().mem_usage().current_memory_in_bytes());
  // memory usage at start
  event = context_.context_proto.mutable_events()->Add();
  event->set_id(label_ + "_mem_usage_1");
  event->set_ph("v");
  event->set_pid(process_id);
  event->set_ts(std::chrono::duration_cast<std::chrono::microseconds>(
                    start_ - context_.start_)
                    .count());
  *event->mutable_args()->mutable_dumps()->mutable_process_totals() =
      convert_to_chrome_event(mem_usage_);
  // memory usage at end
  event = context_.context_proto.mutable_events()->Add();
  event->set_id(label_ + "_mem_usage_2");
  event->set_ph("v");
  event->set_pid(process_id);
  event->set_ts(std::chrono::duration_cast<std::chrono::microseconds>(
                    end_tp - context_.start_)
                    .count());
  *event->mutable_args()->mutable_dumps()->mutable_process_totals() =
      convert_to_chrome_event(end_mem_usage);
}

/// struct PassContext
std::unique_ptr<PassContext> PassContext::create() {
  return std::make_unique<PassContextImp>();
}

CacheFileReaderImp::CacheFileReaderImp(bool in_mem, const std::string& filename,
                                       FILE* fp)
    : CacheFileReader(), in_mem_(in_mem), name_{filename}, fp_{fp} {
  std::rewind(fp);
  LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE))
      << "open " << filename << " for read";
}

CacheFileReaderImp::~CacheFileReaderImp() {
  LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE)) << "close " << name_ << " for read";
}

std::size_t CacheFileReaderImp::fread(void* buffer, std::size_t size) const {
  auto ret = std::fread(buffer, 1u, size, fp_);
  return ret;
}

CacheFileWriterImp::CacheFileWriterImp(bool in_mem, const std::string& filename,
                                       FILE* fp)
    : CacheFileWriter(), in_mem_(in_mem), name_{filename}, fp_{fp} {
  LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE))
      << "open " << filename << " for write";
}

CacheFileWriterImp::~CacheFileWriterImp() {
  LOG_IF(INFO, ENV_PARAM(DEBUG_TAR_CACHE)) << "close " << name_ << " for write";
  std::fflush(fp_);
}

std::size_t CacheFileWriterImp::fwrite(const void* buffer,
                                       std::size_t size) const {
  auto ret = std::fwrite(buffer, 1u, size, fp_);
  return ret;
}

} // namespace vaip_core
