cmake_minimum_required(VERSION 3.20)
project(concert_mujoco)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch mjcf world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
