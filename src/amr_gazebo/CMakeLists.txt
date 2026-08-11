cmake_minimum_required(VERSION 3.8)
project(amr_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# World rendering and spawn composition, shared by every launch file here.
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY worlds config launch models
  DESTINATION share/${PROJECT_NAME}
)

# Phase 0 measurement instruments, not fleet runtime nodes. Listed explicitly
# rather than globbed: a glob is evaluated at configure time, so a newly added
# script silently fails to install until CMake happens to re-run.
install(
  PROGRAMS
    scripts/smoke1_check.py
    scripts/smoke2_drive.py
    scripts/smoke2_measure.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
