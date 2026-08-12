cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The parameter templates and the module that specialises them per robot ship
# together, so a robot's limits reach Nav2 from the same fleet.yaml entry that
# shapes its URDF.
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Phase 1 mission and measurement nodes. Listed explicitly rather than globbed,
# for the reason amr_gazebo documents: a glob is evaluated at configure time, so a
# newly added script silently fails to install until CMake happens to re-run.
install(
  PROGRAMS
    scripts/survey_drive.py
    scripts/nav_goal_run.py
    scripts/plot_run.py
    scripts/map_report.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
