cmake_minimum_required(VERSION 3.12)
project(hcs_base)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# 纯头文件、不依赖 ROS（规则：ROS 从 hcs_executor 才开始出现）。由原来的 hcs_sync 与
# hcs_utility 合并而来；命名空间暂时保留原来的 hcs_sync:: / hcs_utility::。
#
#   thread/    线程配置、实时武装、精确睡眠、门铃、门铃线程、RealtimeScope、RTSan 属性
#   channel/   Snapshot、EventQueue、MpscQueue、ByteTunnel、时钟类型与时基
#   logging/   日志：Logger（前端）→ Backend（队列 + 日志线程）→ Sink（输出端）
#   protocol/  CRC、字节序、环形缓冲、拆包、Eigen 结构化绑定
#   check/     MachineGuard
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_23)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

# ── 单测 ──────────────────────────────────────────────────────────────────
# Doorbell 的丢唤醒、TickTimer 的触发语义、RingBuffer / Snapshot 的内存序，都属于
# "平时跑一万遍都不出、比赛时出一次"、读代码看不出来的那一类。
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name
      test_doorbell
      test_doorbell_worker
      test_tick_timer
      test_ring_buffer
      test_realtime_arm_options
      test_snapshot
      test_event_queue
      test_mpsc_queue
      test_log
      test_byte_tunnel
      test_time_base)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME} pthread)
    target_compile_options(${test_name} PRIVATE -O2 -Wall -Wextra -Wpedantic)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      # HCS_NONBLOCKING 只有配上这个告警组才是真检查（它不在 -Wall / -Wextra 里）。
      # 这个包是纯头文件，没有别的地方替它开：test_log 里"周期域入口必须真的不阻塞"
      # 那一段靠的就是它。-Wno-… 的取舍见 hcs_executor/CMakeLists.txt。
      target_compile_options(${test_name} PRIVATE
        -Wfunction-effects -Wno-perf-constraint-implies-noexcept)
    endif()
  endforeach()
endif()

ament_package()
