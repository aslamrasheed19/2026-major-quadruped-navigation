cmake_minimum_required(VERSION 3.8)
project(xterra_m2_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch rviz config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/svan_pid_controller.py
    scripts/svan_walk_controller.py
    scripts/svan_core.py
    scripts/svan_nav.py
    scripts/svan_log.py
    scripts/compact_urdf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
