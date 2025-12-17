cmake_minimum_required(VERSION 3.8)
project(remo_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# Note: launch and launch_ros are Python packages, not needed in CMake
# They are only needed at runtime, not at build time

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz config files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Install Python scripts
install(PROGRAMS
  src/grid_map_costmap_converter.py
  scripts/activate_nav2_nodes.py
  scripts/yolo_location_saver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
