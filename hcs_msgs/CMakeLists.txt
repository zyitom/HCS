cmake_minimum_required(VERSION 3.12)
project(hcs_msgs)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
add_library(hcs_msgs::hcs_msgs ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_package()
