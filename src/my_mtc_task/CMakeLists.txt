cmake_minimum_required(VERSION 3.8)
project(my_mtc_task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(simple_mtc src/simple_mtc.cpp)
ament_target_dependencies(simple_mtc
  rclcpp
  moveit_task_constructor_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

add_executable(hard_code src/hard_code.cpp)
ament_target_dependencies(hard_code
  rclcpp
  moveit_task_constructor_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

add_executable(auto_painter src/auto_painter.cpp)
ament_target_dependencies(auto_painter
  rclcpp
  moveit_task_constructor_core
  rclcpp_action
  geometry_msgs
)

add_executable(pick_and_place src/pick_and_place.cpp)
ament_target_dependencies(pick_and_place
  rclcpp
  moveit_task_constructor_core
  rclcpp_action
  geometry_msgs
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  simple_mtc
  hard_code
  auto_painter
  pick_and_place
  DESTINATION lib/${PROJECT_NAME})

ament_package()
