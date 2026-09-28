cmake_minimum_required(VERSION 3.12)
project(hcs_description)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
add_library(hcs_description::hcs_description ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(fast_tf)
ament_package()
