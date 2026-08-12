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

ament_package()
