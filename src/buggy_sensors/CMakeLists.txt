cmake_minimum_required(VERSION 3.8)
project(buggy_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)

# Mock GPS Node
add_executable(mock_gps_node src/mock_gps_node.cpp)
ament_target_dependencies(mock_gps_node
  rclcpp
  sensor_msgs
)

# Mock BMS Node
add_executable(mock_bms_node src/mock_bms_node.cpp)
ament_target_dependencies(mock_bms_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  mock_gps_node
  mock_bms_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
