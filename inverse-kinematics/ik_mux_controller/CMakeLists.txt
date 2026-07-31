cmake_minimum_required(VERSION 3.22)
project(ik_mux_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(Eigen3 REQUIRED)


add_executable(ik_mux src/ik_mux.cpp)
ament_target_dependencies(ik_mux
  rclcpp
  control_msgs
  moveit_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
)

add_executable(ik_nxt src/ik_mux_nxt.cpp)
ament_target_dependencies(ik_nxt
  rclcpp
  control_msgs
  moveit_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
)

add_executable(ik_mux_vkb src/ik_mux_vkb.cpp)
ament_target_dependencies(ik_mux_vkb
  rclcpp
  rclcpp_action
  control_msgs
  geometry_msgs
  moveit_msgs
  sensor_msgs
  std_msgs
  std_srvs
  controller_manager_msgs
  tf2_ros
  tf2_geometry_msgs
)

add_executable(singularity_advisor src/singularity_advisor.cpp)
ament_target_dependencies(singularity_advisor
  rclcpp
  moveit_msgs
  sensor_msgs
  std_msgs
  moveit_core
  moveit_ros_planning
)
target_link_libraries(singularity_advisor Eigen3::Eigen)

install(TARGETS

  ik_mux
  ik_nxt
  ik_mux_vkb
  singularity_advisor
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
