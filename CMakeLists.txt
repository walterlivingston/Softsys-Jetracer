cmake_minimum_required(VERSION 3.8)
project(cam_control)

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
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(softsys_msgs REQUIRED)
find_package(sally_msgs REQUIRED)
find_package(marvelmind_ros2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)
# Find src files to compile
add_executable(cam_control src/cam_control.cpp src/image_proc.cpp src/controller.cpp src/path_finder.cpp src/lead_contr.cpp)
ament_target_dependencies(cam_control rclcpp std_msgs OpenCV cv_bridge softsys_msgs marvelmind_ros2_msgs sally_msgs sensor_msgs)

install( 
  TARGETS 
  cam_control
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
