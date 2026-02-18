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
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS simple_mtc
  DESTINATION lib/${PROJECT_NAME})

ament_package()
