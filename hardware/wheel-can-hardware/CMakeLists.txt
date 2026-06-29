cmake_minimum_required(VERSION 3.22)
project(wheel_can_hardware)

set(CMAKE_CXX_STANDARD 23)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(DEPENDENCIES
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  can_util
  ros2_fmt_logger
)

set(ODOM_DEPENDENCIES
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

set(MUX_DEPENDENCIES
  rclcpp
  geometry_msgs
  std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(dep IN LISTS DEPENDENCIES)
  find_package(${dep} REQUIRED)
endforeach()
foreach(dep IN LISTS ODOM_DEPENDENCIES)
  find_package(${dep} REQUIRED)
endforeach()
foreach(dep IN LISTS MUX_DEPENDENCIES)
  find_package(${dep} REQUIRED)
endforeach()

add_library(wheel_can_interface SHARED
  src/wheel_can_interface.cpp
)

target_compile_features(wheel_can_interface PUBLIC cxx_std_23)

target_include_directories(wheel_can_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(wheel_can_interface PUBLIC ${DEPENDENCIES})

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
try_compile(HAS_JAZZY_HARDWARE_INTERFACE ${CMAKE_BINARY_DIR}/is_jazzy
  SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/cmake/has_jazzy_hardware_interface.cpp
  CXX_STANDARD 20 LINK_LIBRARIES hardware_interface::hardware_interface
  OUTPUT_VARIABLE COMPILE_LOG
)

if(HAS_JAZZY_HARDWARE_INTERFACE)
  target_compile_definitions(wheel_can_interface PUBLIC HARDWARE_INTERFACE_IS_JAZZY=1)
else()
  target_compile_definitions(wheel_can_interface PUBLIC HARDWARE_INTERFACE_IS_JAZZY=0)
endif()

pluginlib_export_plugin_description_file(hardware_interface wheel_can_hardware.xml)

add_executable(wheel_odometry_node
  src/wheel_odometry_node.cpp
)
target_include_directories(wheel_odometry_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(wheel_odometry_node ${ODOM_DEPENDENCIES})

add_executable(wheel_bench_node
  src/wheel_bench_node.cpp
)
target_include_directories(wheel_bench_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(wheel_bench_node rclcpp geometry_msgs can_util ros2_fmt_logger)

add_executable(skid_steer_mux
  src/skid_steer_mux.cpp
)
ament_target_dependencies(skid_steer_mux ${MUX_DEPENDENCIES})

install(
  TARGETS wheel_can_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS wheel_odometry_node wheel_bench_node skid_steer_mux
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
