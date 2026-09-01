cmake_minimum_required(VERSION 3.12)
project(rmcs_sync)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

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
  foreach(test_name test_snapshot test_event_queue test_byte_tunnel test_time_base)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME})
    target_compile_options(${test_name} PRIVATE -O2 -Wall -Wextra -Wpedantic)
  endforeach()
endif()

ament_package()
