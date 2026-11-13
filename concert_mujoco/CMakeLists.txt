cmake_minimum_required(VERSION 3.20)
project(concert_mujoco)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch mjcf world
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/robot_mjcf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
