cmake_minimum_required(VERSION 3.8)
project(weld_cell)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch rviz srdf urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/weld_program.py
    scripts/plot_weld_log.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
