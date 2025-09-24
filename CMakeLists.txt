cmake_minimum_required(VERSION 3.8)
project(capra_joy_controls)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(joy_controls src/joy_controls_node.cpp)
ament_target_dependencies(joy_controls rclcpp std_msgs geometry_msgs sensor_msgs)
target_compile_features(joy_controls PRIVATE cxx_std_20)

install(TARGETS
  joy_controls
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config include
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
