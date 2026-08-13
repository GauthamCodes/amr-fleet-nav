cmake_minimum_required(VERSION 3.8)
project(amr_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(safety_gate src/safety_gate.cpp)
target_include_directories(safety_gate PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(safety_gate PRIVATE cxx_std_17)
ament_target_dependencies(safety_gate
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  rcl_interfaces
  diagnostic_msgs
)

install(TARGETS safety_gate DESTINATION lib/${PROJECT_NAME})

# The gate is C++ because it sits in the command path; the SIZING arithmetic that
# produces its constants is Python, so the numbers the gate runs on are the numbers
# tests/test_safety_distance.py checks and the launch file passes in. One package,
# both halves, no chance of them drifting apart.
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Evidence instruments, not runtime. They live here rather than in amr_bringup
# because each one measures this package's node and imports its sizing model.
install(PROGRAMS
  scripts/safety_run.py
  scripts/stopping_sweep.py
  scripts/failclosed_run.py
  scripts/priority_mux.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
