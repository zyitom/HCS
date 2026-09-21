cmake_minimum_required(VERSION 3.12)
project(hcs_executor)

include(CheckCXXCompilerFlag)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 控制线程是这个包的产物，值得开满：-O3 让热循环里的组件调度真正被内联掉。
# -march 默认 x86-64-v3（Haswell 以后都有 AVX2/FMA）；交叉编译或老机器上探测不过就跳过。
set(HCS_MARCH "x86-64-v3" CACHE STRING "-march value; empty disables")
option(HCS_ENABLE_RTSAN "Build with -fsanitize=realtime (clang 20+ only)" OFF)
option(HCS_EXECUTOR_COMPONENT_TIMING "Per-component timing in the RT loop" OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-g1 -rdynamic -O3 -Wall -Wextra -Wpedantic)

  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # HCS_NONBLOCKING 只有配上这个告警组才真的是抓手 —— 不开它，
    # 基类上那句 [[clang::nonblocking]] 就只是个装饰。实测它能顺着
    # RCLCPP_INFO 的宏展开一路指到 fwrite。
    add_compile_options(-Wfunction-effects)
    # 但不要 -Wperf-constraint-implies-noexcept：它要求 nonblocking 函数声明 noexcept，
    # 而 update() 必须允许抛 —— executor 的组件失效隔离就是靠 catch 它来实现
    # 「单点失效不拉整机」的。诚实地说：从 update() 抛异常本身就是一次 RT 违规
    # （异常对象要分配），所以失效隔离是给「已经破坏了 RT 契约」的代码兜底的网。
    # 但在真车上，一个组件抛了应当降级进 safe mode，而不是 terminate 掉整个进程、
    # 把电机停在最后一条指令上。这个取舍见设计文档 §7 的三层失效保护。
    add_compile_options(-Wno-perf-constraint-implies-noexcept)
  endif()

  if(HCS_MARCH)
    # 探测结果按 march 值分开缓存，改了 HCS_MARCH 不会读到上一次的结论。
    string(MAKE_C_IDENTIFIER "HCS_HAVE_MARCH_${HCS_MARCH}" HCS_MARCH_SUPPORTED_VAR)
    check_cxx_compiler_flag("-march=${HCS_MARCH}" ${HCS_MARCH_SUPPORTED_VAR})
    if(${HCS_MARCH_SUPPORTED_VAR})
      add_compile_options(-march=${HCS_MARCH})
    else()
      message(STATUS
        "hcs_executor: -march=${HCS_MARCH} not supported by ${CMAKE_CXX_COMPILER_ID}, skipping")
    endif()
  endif()
endif()

if(HCS_ENABLE_RTSAN)
  # RealtimeSanitizer 只有 clang 20+ 有；gcc 上打开它只会得到一堆看不懂的链接错误。
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(FATAL_ERROR
      "HCS_ENABLE_RTSAN requires clang 20+, but CMAKE_CXX_COMPILER_ID is ${CMAKE_CXX_COMPILER_ID}")
  endif()
  add_compile_options(-fsanitize=realtime)
  add_link_options(-fsanitize=realtime)
endif()

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()

ament_auto_add_library (
  ${PROJECT_NAME}_lib
  SHARED
  src/component.cpp
)

ament_auto_add_executable (
  ${PROJECT_NAME}_exe
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME}_exe ${PROJECT_NAME}_lib)

# executor.hpp 由 src/main.cpp 包含，所以开关必须落在 _exe 上；_lib 一并加，
# 免得以后有人把 executor.hpp 拉进 component.cpp 时两边 ODR 不一致。
if(HCS_EXECUTOR_COMPONENT_TIMING)
  target_compile_definitions(${PROJECT_NAME}_lib PRIVATE HCS_EXECUTOR_COMPONENT_TIMING=1)
  target_compile_definitions(${PROJECT_NAME}_exe PRIVATE HCS_EXECUTOR_COMPONENT_TIMING=1)
endif()

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

# ── 单测 ──────────────────────────────────────────────────────────────────
# test_graph 只依赖 graph.hpp（无 ROS、无 Component），test_wiring 依赖
# component.hpp + wiring.hpp 和 hcs_executor_lib 里的 pending_component_name()。
# 两个都不需要 rclcpp::init / pluginlib / 参数服务器 —— 那正是把建图和接线
# 从 Executor 里剥出来买到的东西。
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_graph test/test_graph.cpp)
  target_include_directories(test_graph PRIVATE ${PROJECT_SOURCE_DIR}/include)

  ament_add_gtest(test_wiring test/test_wiring.cpp)
  target_include_directories(test_wiring PRIVATE ${PROJECT_SOURCE_DIR}/include)
  target_link_libraries(test_wiring ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_wiring hcs_sync hcs_utility)
endif()

set_property(TARGET ${PROJECT_NAME}_lib PROPERTY OUTPUT_NAME ${PROJECT_NAME})
set_property(TARGET ${PROJECT_NAME}_exe PROPERTY OUTPUT_NAME ${PROJECT_NAME})

ament_auto_package()
