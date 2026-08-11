cmake_minimum_required(VERSION 3.8)
project(amr_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The fleet configuration and the single module that interprets it ship together,
# so every launch file in the workspace reads the typed robot list the same way.
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
