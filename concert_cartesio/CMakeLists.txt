cmake_minimum_required(VERSION 3.8)
project(concert_cartesio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install YAML configuration files
file(GLOB YAML_FILES "*.yaml")
if(YAML_FILES)
  install(FILES ${YAML_FILES}
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Install scripts
file(GLOB SCRIPTS "scripts/*")
if(SCRIPTS)
  install(PROGRAMS ${SCRIPTS}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_dependencies()

ament_package()