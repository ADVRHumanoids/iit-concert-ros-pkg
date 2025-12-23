cmake_minimum_required(VERSION 3.0)

set(ROBOT_NAME concert)
project(iit-concert-ros-pkg VERSION 1.0.0)


## add sub-packages
add_subdirectory(concert_gazebo)
add_subdirectory(concert_config)
add_subdirectory(concert_demo)
add_subdirectory(concert_examples)
add_subdirectory(concert_cartesio)
add_subdirectory(concert_odometry_ros2)

enable_testing()
