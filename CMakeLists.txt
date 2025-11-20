cmake_minimum_required(VERSION 3.8)
project(capra_joy_controls)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(THIS_PROJECT_DEPEND
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  capra_control_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
foreach(dep IN ITEMS ${THIS_PROJECT_DEPEND})
  find_package(${dep} REQUIRED)
endforeach(dep)

include_directories(include)

add_library(joy_controls_lib
  src/parsable/Trigger.cpp
  src/parsable/Value.cpp
  src/parsable/Action.cpp
  src/parsable/ControlScheme.cpp
  src/parsable/SchemeMap.cpp
)
ament_target_dependencies(joy_controls_lib
  ${THIS_PROJECT_DEPEND}
)
target_compile_features(joy_controls_lib PRIVATE cxx_std_20)
target_link_libraries(joy_controls_lib yaml-cpp)
target_compile_options(joy_controls_lib PRIVATE -g -fexceptions)


include_directories(${YAML_CPP_INCLUDE_DIR})
add_executable(joy_controls src/joy_controls_node.cpp)
ament_target_dependencies(joy_controls
  ${THIS_PROJECT_DEPEND}
)
target_compile_features(joy_controls PRIVATE cxx_std_20)
target_link_libraries(joy_controls joy_controls_lib)
target_compile_options(joy_controls PRIVATE -g -fexceptions)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_yaml_test test/yaml_test.cpp)
  target_include_directories(${PROJECT_NAME}_yaml_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_compile_features(${PROJECT_NAME}_yaml_test PRIVATE cxx_std_20)
  ament_target_dependencies(${PROJECT_NAME}_yaml_test
    ${THIS_PROJECT_DEPEND}
  )
  target_link_libraries(${PROJECT_NAME}_yaml_test joy_controls_lib)
  target_compile_options(${PROJECT_NAME}_yaml_test PRIVATE -g -fexceptions)
endif()
install(TARGETS
  joy_controls
  joy_controls_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config include
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
