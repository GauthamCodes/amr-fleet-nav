cmake_minimum_required(VERSION 3.8)
project(amr_costmap_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Phase 0 is a skeleton: dependencies are declared in package.xml so the build
# graph is correct, but no library target exists until Phase 4 (RampCostLayer)
# and Phase 6 (FleetTrajectoryLayer). Sources land in src/, headers in
# include/amr_costmap_plugins/, and the pluginlib manifest in costmap_plugins.xml.

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
