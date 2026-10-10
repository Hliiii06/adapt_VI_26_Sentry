cmake_minimum_required(VERSION 3.8)
project(sentry_scan_adapter_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(scan_planner_msgs REQUIRED)

add_executable(rm_input_adapter src/rm_input_adapter_node.cpp)
target_include_directories(rm_input_adapter PRIVATE ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(rm_input_adapter ${PCL_LIBRARIES})
ament_target_dependencies(rm_input_adapter
  rclcpp nav_msgs sensor_msgs geometry_msgs diagnostic_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs pcl_conversions)

add_executable(cmd_gate src/cmd_gate_node.cpp)
ament_target_dependencies(cmd_gate
  rclcpp geometry_msgs std_msgs scan_planner_msgs)

add_executable(task_adapter src/task_adapter_node.cpp)
ament_target_dependencies(task_adapter
  rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(shadow_guard src/shadow_guard_node.cpp)
ament_target_dependencies(shadow_guard
  rclcpp geometry_msgs nav_msgs std_msgs scan_planner_msgs)

add_executable(input_pause_gate src/input_pause_gate_node.cpp)
ament_target_dependencies(input_pause_gate
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)

install(TARGETS rm_input_adapter cmd_gate task_adapter shadow_guard input_pause_gate
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 测试文件自身 #include 节点实现（CMD_GATE_UNIT_TEST 去掉其 main），不要再单独编译节点源文件
  ament_add_gtest(test_cmd_gate_logic test/test_cmd_gate_logic.cpp)
  target_include_directories(test_cmd_gate_logic PRIVATE src)
  ament_target_dependencies(test_cmd_gate_logic
    rclcpp geometry_msgs std_msgs scan_planner_msgs)
endif()

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
