cmake_minimum_required(VERSION 3.8)
project(buggy_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

# Planner library (abstraction and mock implementation)
add_library(buggy_planner_lib
  src/mock_global_planner.cpp
)
ament_target_dependencies(buggy_planner_lib
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

# Global Planner Node
add_executable(global_planner_node
  src/global_planner_node.cpp
)
target_link_libraries(global_planner_node
  buggy_planner_lib
)
ament_target_dependencies(global_planner_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS
  buggy_planner_lib
  global_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_planner test/test_planner.cpp)
  target_link_libraries(test_planner buggy_planner_lib)
  ament_target_dependencies(test_planner
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
  )
endif()

ament_package()
