
#
#   The Xilinx Vitis AI Vaip in this distribution are provided under the following free
#   and permissive binary-only license, but are not provided in source code form.  While the following free
#   and permissive license is similar to the BSD open source license, it is NOT the BSD open source license
#   nor other OSI-approved open source license.
#
#    Copyright (C) 2022 Xilinx, Inc. All rights reserved.
#    Copyright (C) 2023 – 2024 Advanced Micro Devices, Inc. All rights reserved.
#
#    Redistribution and use in binary form only, without modification, is permitted provided that the following conditions are met:
#
#    1. Redistributions must reproduce the above copyright notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the distribution.
#
#    2. The name of Xilinx, Inc. may not be used to endorse or promote products redistributed with this software without specific
#    prior written permission.
#
#    THIS SOFTWARE IS PROVIDED BY XILINX, INC. "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
#    THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL XILINX, INC.
#    BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#    PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
#    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
#


cmake_minimum_required(VERSION 3.17)
project(
  vaip
  VERSION 1.0.0
  LANGUAGES C CXX)
cmake_policy(SET CMP0115 NEW)
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/vaip_common.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/VitisVersion.cmake)
find_package(XRT)
find_package(vart REQUIRED util trace) # used for ENV_PARAM
find_package(xir) # only used for md5sum
find_package(nlohmann_json REQUIRED)


set(vaip_lib_file)
if(USE_STATIC_PYTHON)
  execute_process(
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/static_py_lib.py ${CMAKE_CURRENT_BINARY_DIR}/vaip_lib.zip
      ${PY_LIB} ${CMAKE_CURRENT_SOURCE_DIR}/python/voe ${CMAKE_CURRENT_SOURCE_DIR}/python/onnx_tool
  )
  execute_process(
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/dump_py.py ${CMAKE_CURRENT_SOURCE_DIR}/importer.py
    OUTPUT_VARIABLE PY_IMPORTER_BYTE_ARRAY
  )
  execute_process(
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/dump_zip.py ${CMAKE_CURRENT_BINARY_DIR}/vaip_lib.zip
    OUTPUT_VARIABLE VAIP_LIB_ZIP_BYTE_ARRAY
  )
  configure_file(src/py_zip.hpp py_zip.hpp COPYONLY)
  configure_file(src/py_zip.cpp.in py_zip.cpp @ONLY)
  set(vaip_lib_file ${CMAKE_CURRENT_BINARY_DIR}/py_zip.cpp ${CMAKE_CURRENT_BINARY_DIR}/py_zip.hpp)
endif()

# let the cpp find the hpp
configure_file(include/vaip/config_reader.hpp config_reader.hpp COPYONLY)
configure_file(include/vaip/xclbin_file.hpp xclbin_file.hpp COPYONLY)
configure_file(src/xclbin/config_reader.cpp config_reader.cpp COPYONLY)


configure_file(src/xclbin/mem_xclbin.cpp mem_xclbin.cpp COPYONLY)

set(Protobuf_USE_STATIC_LIBS ON)
find_package(Protobuf)
find_package(target-factory)
find_package(Boost REQUIRED)

find_package(Eigen3) # used for fast transpose
# add the vaip library
set(PROTO_FILES src/config.proto src/anchor_point.proto src/capability.proto
  src/pass_context.proto src/version.proto src/pattern.proto
  ../vaip_pass_level1_dpu/src/dpu_sg_report.proto
  ../vaip_pass_level1_dd_cxx/src/dd_metadata.proto)
if(COMMAND protobuf_generate_cpp)
  protobuf_generate_cpp(
    PROTO_SRCS PROTO_HDRS
    EXPORT_MACRO
    VAIP_DLL_SPEC ${PROTO_FILES})
else()
  set(PROTO_SRCS "")
  set(PROTO_HDRS "")
  foreach(PROTO_FILE ${PROTO_FILES})
    get_filename_component(PROTO_FILE_NAME ${PROTO_FILE} NAME_WE)
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${PROTO_FILE_NAME}.pb.cc
             ${CMAKE_CURRENT_BINARY_DIR}/${PROTO_FILE_NAME}.pb.h
      COMMAND protobuf::protoc
      ARGS --cpp_out=dllexport_decl=VAIP_DLL_SPEC:${CMAKE_CURRENT_BINARY_DIR}
      -I ${CMAKE_CURRENT_SOURCE_DIR}/src
      -I ${CMAKE_CURRENT_SOURCE_DIR}/../vaip_pass_level1_dd_cxx/src
      -I ${CMAKE_CURRENT_SOURCE_DIR}/../vaip_pass_level1_dpu/src
           ${CMAKE_CURRENT_SOURCE_DIR}/${PROTO_FILE}
      DEPENDS ${PROTO_FILE})
    list(APPEND PROTO_SRCS ${CMAKE_CURRENT_BINARY_DIR}/${PROTO_FILE_NAME}.pb.cc)
    list(APPEND PROTO_HDRS ${CMAKE_CURRENT_BINARY_DIR}/${PROTO_FILE_NAME}.pb.h)
  endforeach(PROTO_FILE ${PROTO_FILES})
endif()

if(MSVC)
  find_package(XRT REQUIRED)
  if(BUILD_SHARED_LIBS)
    set(VAIP_DLL_SPEC "__declspec(dllexport)")
  else(BUILD_SHARED_LIBS)
    set(VAIP_DLL_SPEC "__declspec(dllexport)")
  endif(BUILD_SHARED_LIBS)
  set_source_files_properties(
    ${PROTO_SRCS}
    PROPERTIES COMPILE_DEFINITIONS "VAIP_DLL_SPEC=${VAIP_DLL_SPEC}"
               COMPILE_FLAGS "/wd4251")
else(MSVC)
  #find_package(XRT REQUIRED)
  set_source_files_properties(
    ${PROTO_SRCS}
    PROPERTIES COMPILE_DEFINITIONS
               "VAIP_DLL_SPEC=__attribute__((visibility(\"default\")))")
  set_source_files_properties(${PROTO_SRCS} PROPERTIES COMPILE_FLAGS
                                                       -Wno-unused-variable)
endif(MSVC)
configure_file(src/version_info.hpp
               ${CMAKE_CURRENT_BINARY_DIR}/version_info.hpp COPYONLY)
configure_file(src/version_info.cpp.in
               ${CMAKE_CURRENT_BINARY_DIR}/version_info.cpp @ONLY)
vai_add_library(
  NAME
  core
  INCLUDE_DIR
  include
  SRCS
  ${PROTO_SRCS}
  ${PROTO_HDRS}
  ${CMAKE_CURRENT_BINARY_DIR}/../vaip_deps.inc.h
  ${CMAKE_CURRENT_BINARY_DIR}/config_reader.cpp
  src/xclbin/xclbin_file.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/mem_xclbin.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../vaip_summary_report/reporter.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../vaip_summary_report/reporter.hpp
  include/vaip/vaip_plugin.hpp
  src/vaip_plugin.cpp
  include/vaip/plugin.hpp
  src/plugin.cpp
  src/vaip.cpp
  src/tensor_proto.cpp
  include/vaip/tensor_proto.hpp
  include/vaip/vaip.hpp
  include/vaip/_sanity_check.hpp
  include/vaip/custom_op_imp.hpp
  src/custom_op.cpp
  include/vaip/xir_headers.hpp
  include/vaip/util.hpp
  src/vitisai_compile_model.cpp
  src/pass_context_imp.hpp
  src/pass_context_imp.cpp
  src/pass_imp.hpp
  src/pass_imp.cpp
  src/tar_ball.cpp
  src/pass.cpp
  include/vaip/pass.hpp
  src/graph.cpp
  include/vaip/graph.hpp
  src/model.cpp
  include/vaip/model.hpp
  src/anchor_point.cpp
  src/anchor_point_imp.cpp
  src/anchor_point_imp.hpp
  include/vaip/anchor_point.hpp
  include/vaip/node_attr.hpp
  include/vaip/node_arg.hpp
  src/node_arg.cpp
  include/vaip/node_input.hpp
  src/node_input.cpp
  include/vaip/node.hpp
  src/node.cpp
  src/util.cpp
  src/file_lock.cpp
  include/vaip/pattern.hpp
  src/pattern/pattern.cpp
  src/pattern/pattern_node.cpp
  src/pattern/pattern_node.hpp
  src/pattern/pattern_commutable_node.cpp
  src/pattern/pattern_commutable_node.hpp
  src/pattern/pattern_or.cpp
  src/pattern/pattern_or.hpp
  src/pattern/pattern_wildcard.cpp
  src/pattern/pattern_wildcard.hpp
  src/pattern/pattern_sequence.cpp
  src/pattern/pattern_sequence.hpp
  src/pattern/pattern_constant.cpp
  src/pattern/pattern_constant.hpp
  src/pattern/pattern_graph_input.cpp
  src/pattern/pattern_graph_input.hpp
  src/pattern/pattern_where.cpp
  src/pattern/pattern_where.hpp
  include/vaip/rewrite_rule.hpp
  src/xir_ops/xir_ops_defs.hpp
  src/xir_ops/xir_ops_defs.cpp
  src/xir_ops/xir_ops.hpp
  src/rewrite_rule.cpp
  src/node_attrs.cpp
  src/config.hpp
  src/config.cpp
  src/cache_dir.hpp
  src/cache_dir.cpp
  src/stat.cpp
  src/stat.hpp
  src/file_lock.hpp
  src/profile_utils.hpp
  src/profile_utils.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/version_info.cpp
  src/version_info.cpp.in
  include/vaip/transpose.hpp
  src/transpose.cpp
  include/vaip/guess_reshape.hpp
  src/guess_reshape.cpp
  include/vaip/dd/coeffs.hpp
  src/dd/coeffs.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/version.c
  ${vaip_lib_file})

target_include_directories (core PRIVATE ${xir_SOURCE_DIR} ${zlib_SOURCE_DIR} ${zlib_BINARY_DIR})
check_include_file(dlfcn.h DLFCN_H_FOUND)
if(DLFCN_H_FOUND)
  target_sources(core PRIVATE src/util_linux.cpp)
elseif(MSVC)
  target_sources(core PRIVATE src/util_mswin.cpp)
else()
  target_sources(core PRIVATE src/util_unknown.cpp)
endif()
target_link_libraries(
  core
  PRIVATE xir::xir vart::util vart::trace Eigen3::Eigen target-factory::target-factory ZLIB::ZLIB
  Boost::interprocess nlohmann_json::nlohmann_json
  PUBLIC protobuf::libprotobuf vart::util)
if(WIN24_BUILD)
  target_compile_definitions(core PUBLIC WIN24_BUILD)
endif()
if(USE_STATIC_PYTHON)
  target_link_libraries(core PUBLIC static_python::header)
endif()
if(BUILD_PYTHON_EXT)
  target_link_libraries(core PRIVATE pybind11::embed)
  target_compile_definitions(core PUBLIC ENABLE_PYTHON)
endif()
if (TARGET Boost::filesystem)
  target_link_libraries(core PRIVATE Boost::filesystem)
  target_compile_definitions(core PRIVATE ENABLE_BOOST)
endif()


if(NOT MSVC)
  target_link_options(core PUBLIC -Wl,--exclude-libs,ALL)
endif(NOT MSVC)

target_compile_definitions(core PUBLIC -DVAIP_USE_DLL=1)

if(WITH_XCOMPILER)
  target_compile_definitions(core PUBLIC -DWITH_XCOMPILER=1)
else(WITH_XCOMPILER)
  target_compile_definitions(core PUBLIC -DWITH_XCOMPILER=0)
endif(WITH_XCOMPILER)

if(MSVC)
  set_source_files_properties(src/transpose.cpp PROPERTIES COMPILE_FLAGS
                                                           "/wd4251 ")
else(MSVC)
  set_source_files_properties(src/transpose.cpp PROPERTIES COMPILE_FLAGS -O3)
endif(MSVC)

if(XRT_FOUND)
	target_compile_definitions(core PRIVATE ENABLE_XRT)
    target_include_directories(core PRIVATE ${XRT_INCLUDE_DIRS})
endif(XRT_FOUND)

if(GRAPH_ENGINE_HASH)
  target_compile_definitions(core PRIVATE GRAPH_ENGINE_VERSION="${GRAPH_ENGINE_HASH}")
endif(GRAPH_ENGINE_HASH)
# <build>/vaip/config.pb.h is generated from
# ${CMAKE_CURRENT_SOURCE_DIR}/config.proto we expect internal use as
# "vaip/config.pb.h"
target_include_directories(
  core
  PRIVATE ${PYTHON_INCLUDE_DIRS}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/..>
         $<INSTALL_INTERFACE:include> $<INSTALL_INTERFACE:include/vaip>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/vaip>)

if(BUILD_SHARED_LIBS)
    target_compile_definitions(core PRIVATE "-DVAIP_EXPORT_DLL=1")
else()
    target_compile_definitions(core PUBLIC "-DVAIP_EXPORT_DLL=1")
endif(BUILD_SHARED_LIBS)
target_compile_definitions(core PUBLIC "-DONNX_NAMESPACE=onnx")
if(MSVC)
  set_source_files_properties(${PROTO_SRCS} PROPERTIES COMPILE_FLAGS "/wd4251")
else(MSVC)
  set_source_files_properties(
    ${PROTO_SRCS} PROPERTIES COMPILE_FLAGS
                             "-Wno-unused-variable -Wno-conversion")
endif(MSVC)

vai_add_debug_command(core
    COMMAND "${CMAKE_INSTALL_PREFIX}\\bin\\test_onnx_runner.exe"
    ARGUMENTS "${CMAKE_CURRENT_SOURCE_DIR}\\..\\..\\test_onnx_runner\\data\\pt_resnet50.onnx"
    ENVIRONMENT DEBUG_DD_MERGE_QMHAGRPB=1
  )
