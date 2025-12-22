cmake_minimum_required(VERSION 3.8)
project(concert_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Robot description publisher node
add_executable(robot_description_publisher src/robot_description_publisher.cpp)
ament_target_dependencies(robot_description_publisher rclcpp std_msgs)

install(TARGETS robot_description_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY gui hal joint_config
  DESTINATION share/${PROJECT_NAME}/
)
install(FILES 
  modular.yaml
  ModularBot_impedance_config.yaml
  setup.sh
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
