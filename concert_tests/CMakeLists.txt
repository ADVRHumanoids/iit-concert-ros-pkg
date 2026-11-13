cmake_minimum_required(VERSION 3.8)
project(concert_tests)

find_package(ament_cmake REQUIRED)

install(DIRECTORY robot
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS src/repeatability.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
