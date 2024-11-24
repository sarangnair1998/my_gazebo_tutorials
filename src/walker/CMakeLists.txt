cmake_minimum_required(VERSION 3.8)
project(walker)

# Set compile options for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Export compile_commands.json for clang-tidy and other tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++17)


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds)

# Create the executable
add_executable(walker_node src/walker.cpp)

# Add include directories
target_include_directories(walker_node PUBLIC include)


# Link against required dependencies
ament_target_dependencies(walker_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs  # Added for consistency with other dependencies
)

# Install the executable
install(TARGETS walker_node
  DESTINATION lib/${PROJECT_NAME})

# Add testing support
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # The following lines skip certain lint checks; comment them if necessary
  set(ament_cmake_copyright_FOUND TRUE)  # Skip copyright check
  set(ament_cmake_cpplint_FOUND TRUE)    # Skip cpplint check

  # Automatically find and include all test dependencies
  ament_lint_auto_find_test_dependencies()
endif()

# Enable ament_package for the package
ament_package()
