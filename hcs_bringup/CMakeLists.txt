cmake_minimum_required(VERSION 3.8)
project(hcs_bringup)

find_package(ament_cmake REQUIRED)
find_package(hcs_utility REQUIRED)

# 机器护栏检查工具（只读，无需 root）：ros2 run hcs_bringup hcs_machine_check
add_executable(hcs_machine_check tools/hcs_machine_check.cpp)
target_compile_features(hcs_machine_check PRIVATE cxx_std_23)
ament_target_dependencies(hcs_machine_check hcs_utility)
install(TARGETS hcs_machine_check DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
