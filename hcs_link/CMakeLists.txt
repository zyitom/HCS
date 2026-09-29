cmake_minimum_required(VERSION 3.12)
project(hcs_link)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# 纯头文件，只依赖标准库和 POSIX：视觉那边不装 ROS 也能直接把 include/ 加进去用，
# 这里的 ament 只负责在本工作区里安装和导出。
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_23)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # -DHCS_LINK_SANITIZE=thread|address：给单测加消毒器，单独开一个 build base 跑。
  set(HCS_LINK_SANITIZE "" CACHE STRING "sanitizer for hcs_link tests (thread/address/empty)")

  foreach(test_name test_channel test_rendezvous test_quiescent_cell test_autoaim)
    ament_add_gtest(${test_name} test/${test_name}.cpp TIMEOUT 120)
    target_link_libraries(${test_name} ${PROJECT_NAME})
    target_compile_options(${test_name} PRIVATE -O2 -g -Wall -Wextra -Wpedantic)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      # 库里标了 [[clang::nonblocking]] 的函数由这一组告警负责核实。
      target_compile_options(${test_name} PRIVATE
        -Wfunction-effects -Wno-perf-constraint-implies-noexcept)
    endif()
    if(HCS_LINK_SANITIZE)
      target_compile_options(${test_name} PRIVATE -fsanitize=${HCS_LINK_SANITIZE})
      target_link_options(${test_name} PRIVATE -fsanitize=${HCS_LINK_SANITIZE})
    endif()
  endforeach()
endif()

ament_package()
