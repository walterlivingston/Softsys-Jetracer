cmake_minimum_required(VERSION 3.8)
project(rviz_support)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find needed packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(marvelmind_ros2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)
# Find src files to compile
add_executable(rviz_repub src/rviz_repub_node.cpp)
ament_target_dependencies(rviz_repub rclcpp std_msgs nav_msgs geometry_msgs marvelmind_ros2_msgs)

add_executable(loc_logger src/loc_logger.cpp)
ament_target_dependencies(loc_logger rclcpp std_msgs nav_msgs marvelmind_ros2_msgs)

install(
  TARGETS 
  rviz_repub
  loc_logger
  DESTINATION lib/${PROJECT_NAME}
)

# make sure we grab config folder
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
