cmake_minimum_required(VERSION 3.5)
project(aichallenge_submit_launch)

find_package(ament_cmake_auto REQUIRED)
install(PROGRAMS
  scripts/rviz.bash
  DESTINATION lib/${PROJECT_NAME}
)
ament_auto_find_build_dependencies()
ament_auto_package(INSTALL_TO_SHARE launch map config scripts)
