cmake_minimum_required(VERSION 3.5)
project(cbf_controller)

# Export compile options for lsp
set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

# Find dependencies
# find_package(ament_cmake_python REQUIRED)
# find_package(rclpy REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(cbf_controller controller_ros2.cpp)
ament_target_dependencies(cbf_controller rclcpp nav_msgs geometry_msgs tf2 tf2_ros std_msgs)

# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

install(
  TARGETS cbf_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
