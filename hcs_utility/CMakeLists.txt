cmake_minimum_required(VERSION 3.12)
project(hcs_utility)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()

include_directories(${PROJECT_SOURCE_DIR}/include)

# ── 单测 ──────────────────────────────────────────────────────────────────
# 这个包本来一个测试都没有。Doorbell 是第一个**必须**有测试的:丢唤醒属于
# "平时跑一万遍都不出、比赛时出一次"那一类,靠肉眼读代码看不出来。
# TickTimer / RingBuffer 同理:前者的触发语义靠无符号下溢的奇偶性成立,
# 后者的正确性靠 acquire/release 内存序 —— 都是"读代码看不出来"的那类。
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name
      test_doorbell
      test_doorbell_worker
      test_tick_timer
      test_ring_buffer)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_include_directories(${test_name} PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_compile_options(${test_name} PRIVATE -O2 -Wall -Wextra -Wpedantic)
    target_link_libraries(${test_name} pthread)
  endforeach()
endif()

ament_auto_package()
