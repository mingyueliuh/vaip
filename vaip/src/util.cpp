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
#include "vaip/util.hpp"
#include <cstdio>

#include <glog/logging.h>

#include "vaip/graph.hpp"
#include <vaip/vaip_ort_api.h>

#include "vitis/ai/env_config.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#ifdef ENABLE_PYTHON
#  include <pybind11/embed.h>
#  include <pybind11/pybind11.h>
#  ifdef Py_NO_ENABLE_SHARED
#    include "import_python_module.h"
#    include "py_zip.hpp"
#    include "static_python_init.hpp"
#  endif
namespace py = pybind11;
#  ifdef Py_NO_ENABLE_SHARED
DEFINE_EXTERNAL_SYMBOL();
EXTERN_PYTHON_MACRO(voe_cpp2py_export);
#  endif
#endif

DEF_ENV_PARAM(DEBUG_VAIP_UTIL, "0")
#define MY_LOG(n) LOG_IF(INFO, ENV_PARAM(DEBUG_VAIP_UTIL) >= n)

namespace vaip_core {
VAIP_DLL_SPEC void dump_graph(const Graph& graph, const std::string& filename) {
  std::ofstream out(filename);
  auto text = graph_as_string(graph);
  out << text;
  out.close();
}

VAIP_DLL_SPEC std::unique_ptr<int> scale_to_fix_point(float scale) {
  auto fix_point = (int)(std::log2f(1 / scale));
  if (std::exp2f((float)fix_point) * scale == 1) {
    return std::make_unique<int>(fix_point);
  } else
    return std::make_unique<int>();
}

std::string convert_to_xir_op_type(const std::string& domain,
                                   const std::string& op_type) {
  if (domain == "com.xilinx") {
    if (op_type.size() >= 4u && op_type.substr(op_type.size() - 4u) == "_fix") {
      return op_type.substr(0u, op_type.size() - 4u) + "-fix";
    } else if (op_type == "transposed_conv2d") {
      return "transposed-conv2d";
    } else if (op_type == "leaky_relu") {
      return "leaky-relu";
    } else if (op_type == "depthwise_conv2d") {
      return "depthwise-conv2d";
    } else if (op_type == "depthwise_conv1d") {
      return "depthwise-conv1d";
    } else if (op_type == "hard_sigmoid") {
      return "hard-sigmoid";
    } else if (op_type == "hard_softmax") {
      return "hard-softmax";
    } else if (op_type == "pixel_shuffle") {
      return "pixel-shuffle";
    } else if (op_type == "quantize_linear") {
      return "quantize-linear";
    } else if (op_type == "dequantize_linear") {
      return "dequantize-linear";
    } else if (op_type == "quantize_linear_int8") {
      return "quantize-linear-int8";
    } else if (op_type == "quantize_linear_uint8") {
      return "quantize-linear-uint8";
    } else if (op_type == "dequantize_linear_int8") {
      return "dequantize-linear-int8";
    } else if (op_type == "dequantize_linear_uint8") {
      return "dequantize_linear_uint8";
    }
    return op_type;
  }
  return domain + ":" + op_type;
}

static std::vector<std::string> split_path(const char* env_name) {
  std::string path;
#ifdef _WIN32
#  pragma warning(push)
#  pragma warning(disable : 4996)
#endif
  auto env_value = getenv(env_name);
  path = env_value != nullptr ? env_value : "";
#ifdef _WIN32
#  pragma warning(pop)
#endif
  auto ret = std::vector<std::string>();
#ifdef _MSC_VER
  char sep = ';';
#else
  char sep = ':';
#endif

  std::string::size_type pos0 = 0u;
  for (auto pos = path.find(sep, pos0); pos != std::string::npos;
       pos = path.find(sep, pos0)) {
    ret.push_back(path.substr(pos0, pos - pos0));
    pos0 = pos + 1;
  }
  if (pos0 != std::string::npos) {
    ret.push_back(path.substr(pos0, path.size() - pos0));
  }
  return ret;
}

std::string find_file_in_path(const std::string& file, const char* env_name,
                              bool required) {
  auto path = split_path(env_name);
  namespace fs = std::filesystem;
  for (auto& p : path) {
    auto dir_path = fs::path(p);
    auto file_path = dir_path / fs::path(file);
    MY_LOG(1) << "for vai_config.json trying " << file_path;
    if (fs::exists(file_path)) {
      return file_path.u8string();
    }
  }
  std::ostringstream str;
  if (required) {
    str << "cannot find file " << file << " after searching following path\n";
    for (auto& p : path) {
      auto dir_path = fs::path(p);
      auto file_path = dir_path / fs::path(file);
      str << "\t" << file_path << "\n";
    }
    str << "please check enviroment variable " << env_name;
    LOG(FATAL) << str.str();
  }
  return std::string();
}

std::string slurp(const char* filename) {
  std::ifstream in;
  in.open(filename, std::ifstream::in);
  std::stringstream sstr;
  sstr << in.rdbuf();
  in.close();
  return sstr.str();
}
VAIP_DLL_SPEC std::string slurp(const std::filesystem::path& path) {
  return slurp(path.u8string().c_str());
}
std::string slurp_if_exists(const std::filesystem::path& path) {
  if (std::filesystem::exists(path))
    return slurp(path.u8string().c_str());
  else
    return std::string("");
}
#ifdef ENABLE_PYTHON
std::shared_ptr<void> init_interpreter() {
  static std::mutex mtx;
  static std::weak_ptr<void> py_interpreter_holder;
  std::shared_ptr<void> ret;
  std::lock_guard<std::mutex> lock(mtx);
  if (!Py_IsInitialized()) {
#  ifdef Py_NO_ENABLE_SHARED
    PyImport_AppendInittab("voe.voe_cpp2py_export", PyInit_voe_cpp2py_export);
    IMPORT_EXTERNAL_SYMBOL();
    // basically same as the scoped_interpreter, excepts path
    static static_scoped_interpreter inter{};
    // fix imoporting submodule
    std::string s(get_vaip_lib_in_mem(), get_vaip_lib_in_mem_size());
    py::bytes py_bytes(s);
    auto global = py::globals();
    global["vaip_lib"] = py_bytes;
    PyRun_SimpleString(get_importer_py_str());
#  else
    static py::scoped_interpreter inter{};
#  endif
    auto p = static_cast<void*>(&inter);
    ret = std::shared_ptr<void>(p, [](void* p) {});
    py_interpreter_holder = ret;
  }
  if (!ret) {
    ret = py_interpreter_holder.lock();
  }
  return ret;
}

VAIP_DLL_SPEC void eval_python_code(const std::string& code) {
  auto inter = init_interpreter();
  py::gil_scoped_acquire acquire;
  py::eval(code);
}
#endif

std::string dos2unix(const gsl::span<const char> input) {
  std::string ret;
  ret.reserve(input.size());
  for (auto c : input) {
    if (c == '\r') {
      continue;
    }
    ret.push_back(c);
  }
  return ret;
}
template <typename T> struct binary_io {
  using char_type = T;
  static std::vector<char_type>
  slurp_binary(const std::filesystem::path& filename) {
    std::ifstream is(filename, std::ios::binary);
    CHECK(is.good()) << "cannot open file " << filename;
    CHECK(is.seekg(0, std::ios_base::end).good());
    auto size = is.tellg();
    CHECK_NE(size, -1);
    CHECK(is.seekg(0, std::ios_base::beg).good());
    auto buffer = std::vector<char_type>((size_t)size / sizeof(char_type));
    CHECK(is.read(reinterpret_cast<char*>(buffer.data()), size).good());
    return buffer;
  }

  static bool dump_binary(const std::filesystem::path& filename,
                          gsl::span<const char_type> data) {
    std::ofstream out(filename, std::ios::binary);
    CHECK(out.write(reinterpret_cast<const char*>(data.data()),
                    data.size() * sizeof(char_type))
              .good());
    return true;
  }
};

std::vector<uint8_t> slurp_binary_u8(const std::filesystem::path& filename) {
  return binary_io<uint8_t>::slurp_binary(filename);
}
std::vector<int8_t> slurp_binary_i8(const std::filesystem::path& filename) {
  return binary_io<int8_t>::slurp_binary(filename);
}
std::vector<char> slurp_binary_c8(const std::filesystem::path& filename) {
  return binary_io<char>::slurp_binary(filename);
}

bool dump_binary(const std::filesystem::path& filename,
                 gsl::span<const uint8_t> data) {
  return binary_io<uint8_t>::dump_binary(filename, data);
}
bool dump_binary(const std::filesystem::path& filename,
                 gsl::span<const int8_t> data) {
  return binary_io<int8_t>::dump_binary(filename, data);
}
bool dump_binary(const std::filesystem::path& filename,
                 gsl::span<const char> data) {
  return binary_io<char>::dump_binary(filename, data);
}

/**
 * Compresses the given memory data using the ZIP algorithm.
 *
 * @param data The memory data to be compressed.
 * @return A vector containing the compressed data.
 */
#include <zlib.h>
template <typename T> struct zlib {
  using char_type = T;

  /*
        Update a running crc with the bytes buf[0..len-1] and return
      the updated crc. The crc should be initialized to zero. Pre- and
      post-conditioning (one's complement) is performed within this
      function so it shouldn't be done by the caller. Usage example:

        unsigned long crc = 0L;

        while (read_buffer(buffer, length) != EOF) {
          crc = update_crc(crc, buffer, length);
        }
        if (crc != original_crc) error();
     */
  static unsigned long update_crc(unsigned long crc, const unsigned char* buf,
                                  int len) {
    /* Table of CRCs of all 8-bit messages. */
    static unsigned long crc_table[256];
    /* Flag: has the table been computed? Initially false. */
    static int crc_table_computed;
    /* Make the table for a fast CRC. */
    auto make_crc_table = [&] {
      unsigned long c;
      int n, k;
      for (n = 0; n < 256; n++) {
        c = (unsigned long)n;
        for (k = 0; k < 8; k++) {
          if (c & 1) {
            c = 0xedb88320L ^ (c >> 1);
          } else {
            c = c >> 1;
          }
        }
        crc_table[n] = c;
      }
      crc_table_computed = 1;
    };
    unsigned long c = crc ^ 0xffffffffL;
    int n;

    if (!crc_table_computed) {
      make_crc_table();
    }
    for (n = 0; n < len; n++) {
      c = crc_table[(c ^ buf[n]) & 0xff] ^ (c >> 8);
    }
    return c ^ 0xffffffffL;
  }

  /* Return the CRC of the bytes buf[0..len-1]. */
  static unsigned long calculate_crc(const unsigned char* buf, int len) {
    return update_crc(0L, buf, len);
  }
  static std::vector<char_type> compress(gsl::span<const char_type> data,
                                         int level) {
#if _WIN32
    FILE* tmp_file = nullptr;
    auto err = tmpfile_s(&tmp_file);
    CHECK_EQ(err, 0) << "tmpfile_s error";
    auto fd = _fileno(tmp_file);
#else
    FILE* tmp_file = tmpfile();
    CHECK(tmp_file != nullptr) << "cannot create tmp file";
    auto fd = fileno(tmp_file);
#endif
    auto gzfile = gzdopen(fd, "wb");
    CHECK(gzfile != nullptr) << "gzopen error";
    auto status = gzwrite(gzfile, data.data(), (unsigned)data.size());
    CHECK_EQ((size_t)status, data.size()) << "gzwrite error";
    status = gzflush(gzfile, Z_FINISH);
    CHECK_EQ(status, 0) << "gzflush error";
    status = fseek64(tmp_file, 0, SEEK_END);
    CHECK(status == 0) << "fseek error";
    auto size = ftell64(tmp_file);
    CHECK_NE(size, -1) << "ftell error";
    status = fseek64(tmp_file, 0, SEEK_SET);
    auto output_buffer = std::vector<char_type>((size_t)size);
    auto read_size =
        std::fread(output_buffer.data(), 1, output_buffer.size(), tmp_file);
    CHECK_EQ((size_t)read_size, output_buffer.size());
    fclose(tmp_file);
    return output_buffer;
  }

  static std::vector<char_type> uncompress(gsl::span<const char_type> data) {
#if _WIN32
    FILE* tmp_file = nullptr;
    auto err = tmpfile_s(&tmp_file);
    CHECK_EQ(err, 0) << "tmpfile_s error";
#else
    FILE* tmp_file = tmpfile();
    CHECK(tmp_file != nullptr) << "cannot create tmp file";
    int err = 0;
#endif
    auto write_size = std::fwrite(data.data(), 1, data.size(), tmp_file);
    CHECK_EQ((size_t)write_size, data.size());
    err = fflush(tmp_file);
    auto status = fseek64(tmp_file, 0, SEEK_SET);
    CHECK_EQ(status, 0) << "fseek error";
    CHECK_EQ(err, 0) << "fflush error";
#if _WIN32
    auto fd = _fileno(tmp_file);
#else
    auto fd = fileno(tmp_file);
#endif
    auto gzfile = gzdopen(fd, "rb");
    CHECK(gzfile != nullptr) << "gzopen error";
    auto output_buffer = std::vector<char_type>();
    auto tmpbuffer = std::vector<char_type>(8 * 1024u);
    do {
      auto read_size =
          (size_t)gzread(gzfile, tmpbuffer.data(),
                         (unsigned)(tmpbuffer.size() * sizeof(char_type)));
      int err_num = 0;
      auto err_msg = gzerror(gzfile, &err_num);
      CHECK_EQ(err_num, Z_OK) << "gzread error: " << err_msg;
      output_buffer.insert(output_buffer.end(), tmpbuffer.begin(),
                           tmpbuffer.begin() + read_size / sizeof(char_type));
    } while (!gzeof(gzfile));
    fclose(tmp_file);
    return output_buffer;
  }
};
std::vector<uint8_t> compress(gsl::span<const uint8_t> data, int level) {
  return zlib<uint8_t>::compress(data, level);
}
std::vector<int8_t> compress(gsl::span<const int8_t> data, int level) {
  return zlib<int8_t>::compress(data, level);
}
std::vector<char> compress(gsl::span<const char> data, int level) {
  return zlib<char>::compress(data, level);
}
std::vector<uint8_t> uncompress(gsl::span<const uint8_t> data) {
  return zlib<uint8_t>::uncompress(data);
}
std::vector<int8_t> uncompress(gsl::span<const int8_t> data) {
  return zlib<int8_t>::uncompress(data);
}
std::vector<char> uncompress(gsl::span<const char> data) {
  return zlib<char>::uncompress(data);
}
} // namespace vaip_core
