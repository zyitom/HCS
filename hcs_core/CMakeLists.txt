cmake_minimum_required(VERSION 3.12)
project(hcs_demo)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# libhcs host SDK：板卡组件（CanRttProbe、HcsLinkProbe）要走真实 HCS 板卡。
# 放在下面 add_compile_options 之前，好让 -Wfunction-effects 之类只管我们自己的组件代码，
# 不去噪 SDK——CMake 的目录级编译选项是 add_subdirectory 那一刻的快照，之后父目录再加的
# 选项不会回头影响已经处理过的子目录。SDK 自己声明 STATIC 目标 libhcs-sdk，
# 这里只 add_subdirectory，下面 ament_auto_add_library 之后再 target_link_libraries。
#
# 源码树按顺序找：-DLIBHCS_SOURCE_DIR=... > src/libhcs（留给将来的子模块）>
# 和 HCS 同级的 libhcs 工作区（开发机上就是 ~/Desktop/libhcs）。
set(LIBHCS_SOURCE_DIR "" CACHE PATH "libhcs checkout (the directory holding core/ and host/)")
set(_libhcs_source_dir "${LIBHCS_SOURCE_DIR}")
if(NOT _libhcs_source_dir)
  foreach(_candidate
      "${CMAKE_CURRENT_SOURCE_DIR}/src/libhcs"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../libhcs")
    if(EXISTS "${_candidate}/host/CMakeLists.txt")
      get_filename_component(_libhcs_source_dir "${_candidate}" ABSOLUTE)
      break()
    endif()
  endforeach()
endif()
if(NOT EXISTS "${_libhcs_source_dir}/host/CMakeLists.txt")
  message(FATAL_ERROR "hcs_demo: libhcs not found; pass -DLIBHCS_SOURCE_DIR=<libhcs checkout>")
endif()
message(STATUS "hcs_demo: libhcs from ${_libhcs_source_dir}")

# libhcs 的版本号取自 git tag；工作区里没有 tag 时它自己的 CMake 会直接 FATAL。
# 这里先替它跑一次版本脚本，拿不到就退回 0.0.0（和本机固件构建用的覆盖值一致，
# 这样主机与固件的版本门禁比的是同一个号）。显式传 -Dlibhcs_PROJECT_VERSION 时不动。
foreach(_kind PROJECT DEBIAN)
  if(NOT DEFINED libhcs_${_kind}_VERSION OR "${libhcs_${_kind}_VERSION}" STREQUAL "")
    set(_format_args "")
    if(_kind STREQUAL "DEBIAN")
      set(_format_args --format debian)
    endif()
    execute_process(
      COMMAND ".scripts/generate_version" ${_format_args}
      WORKING_DIRECTORY "${_libhcs_source_dir}"
      OUTPUT_VARIABLE _version
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE _result
      ERROR_QUIET)
    if(NOT _result EQUAL 0 OR _version STREQUAL "")
      set(_version "0.0.0")
    endif()
    set(libhcs_${_kind}_VERSION "${_version}")
  endif()
endforeach()

add_subdirectory("${_libhcs_source_dir}/host" libhcs-sdk-build EXCLUDE_FROM_ALL)

include(CheckCXXCompilerFlag)

# 组件跑在 1 kHz 热循环里，和 hcs_executor 一样开满。两个包的 -march 必须一致：
# component.hpp 是 header-only 的，不一致会让同一个 inline 函数在两边生成不同的码。
set(HCS_MARCH "x86-64-v3" CACHE STRING "-march value; empty disables")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)

  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # 组件的 update() 才是 HCS_NONBLOCKING 真正要管的地方 —— 这个包比
    # hcs_executor 更需要这个告警组。理由与 -Wno-perf-constraint-implies-noexcept
    # 的取舍见 hcs_executor/CMakeLists.txt 里的注释。
    add_compile_options(-Wfunction-effects -Wno-perf-constraint-implies-noexcept)
  endif()

  if(HCS_MARCH)
    string(MAKE_C_IDENTIFIER "HCS_HAVE_MARCH_${HCS_MARCH}" HCS_MARCH_SUPPORTED_VAR)
    check_cxx_compiler_flag("-march=${HCS_MARCH}" ${HCS_MARCH_SUPPORTED_VAR})
    if(${HCS_MARCH_SUPPORTED_VAR})
      add_compile_options(-march=${HCS_MARCH})
    else()
      message(STATUS
        "hcs_demo: -march=${HCS_MARCH} not supported by ${CMAKE_CXX_COMPILER_ID}, skipping")
    endif()
  endif()
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ── 档 0：显式源文件列表，不用 GLOB ────────────────────────────────────────
# 新组件要在这里显式加一行。GLOB_RECURSE 会把以后放进 src/ 的子模块、缺外部依赖的文件
# 一起抓进来，colcon build 直接失败。板卡组件只依赖上面链接的 libhcs host SDK。
set(PROJECT_SOURCE
  src/hardware/demo_board.cpp
  src/hardware/can_rtt_probe.cpp
  src/hardware/hcs_link_probe.cpp
  src/controller/pid/pid_controller.cpp
)

# 库名必须和 plugins.xml 里的 <library path="..."> 一致，
# 否则 pluginlib 在运行时找不到 .so。
ament_auto_add_library(${PROJECT_NAME} SHARED ${PROJECT_SOURCE})
target_link_libraries(${PROJECT_NAME} libhcs-sdk)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

pluginlib_export_plugin_description_file(hcs_executor plugins.xml)

ament_auto_package()
