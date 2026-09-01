cmake_minimum_required(VERSION 3.12)
project(rmcs_demo)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CheckCXXCompilerFlag)

# 组件跑在 1 kHz 热循环里，和 rmcs_executor 一样开满。两个包的 -march 必须一致：
# component.hpp 是 header-only 的，不一致会让同一个 inline 函数在两边生成不同的码。
set(RMCS_MARCH "x86-64-v3" CACHE STRING "-march value; empty disables")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)

  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # 组件的 update() 才是 RMCS_NONBLOCKING 真正要管的地方 —— 这个包比
    # rmcs_executor 更需要这个告警组。理由与 -Wno-perf-constraint-implies-noexcept
    # 的取舍见 rmcs_executor/CMakeLists.txt 里的注释。
    add_compile_options(-Wfunction-effects -Wno-perf-constraint-implies-noexcept)
  endif()

  if(RMCS_MARCH)
    string(MAKE_C_IDENTIFIER "RMCS_HAVE_MARCH_${RMCS_MARCH}" RMCS_MARCH_SUPPORTED_VAR)
    check_cxx_compiler_flag("-march=${RMCS_MARCH}" ${RMCS_MARCH_SUPPORTED_VAR})
    if(${RMCS_MARCH_SUPPORTED_VAR})
      add_compile_options(-march=${RMCS_MARCH})
    else()
      message(STATUS
        "rmcs_demo: -march=${RMCS_MARCH} not supported by ${CMAKE_CXX_COMPILER_ID}, skipping")
    endif()
  endif()
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ── 档 0：显式源文件列表，不再 GLOB ────────────────────────────────────────
# 原来的 file(GLOB_RECURSE src/*.cpp) 会把整棵 src/librmcs/（core + host + 三块板
# 固件，约 300 个 .cpp）连同 broadcaster / referee / hardware/real_car.cpp 一起抓进来，
# 它们各自缺外部依赖 —— librmcs SDK、std_msgs、fast_tf、std_srvs、rmcs_msgs ——
# colcon build 必失败。本包里真正编得过的本来就只有下面这两个。
#
# 正解（设计文档 档 0）是把 src/librmcs/ 物理移出本包、按外部依赖链进来：
# 它自己已经带完整的 CMake 构建，产出 liblibrmcs-sdk.a。本次只在构建里排除，
# 目录原样保留，等集成层落地再动。
# real_car.cpp 同理 —— 它现在的价值只剩「集成层做完之后的验收清单」。
set(PROJECT_SOURCE
  src/hardware/demo_board.cpp
  src/controller/pid/pid_controller.cpp
)

# 库名必须和 plugins.xml 里的 <library path="..."> 一致，
# 否则 pluginlib 在运行时找不到 .so。
ament_auto_add_library(${PROJECT_NAME} SHARED ${PROJECT_SOURCE})

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

ament_auto_package()
