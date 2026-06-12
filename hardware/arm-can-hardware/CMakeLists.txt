cmake_minimum_required(VERSION 3.22)
project(arm_can_hardware)

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

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(dep IN LISTS DEPENDENCIES)
  find_package(${dep} REQUIRED)
endforeach()

add_library(arm_can_interface SHARED
  src/arm_can_interface.cpp
)

target_compile_features(arm_can_interface PUBLIC cxx_std_23)

target_include_directories(arm_can_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arm_can_interface PUBLIC ${DEPENDENCIES})

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
try_compile(HAS_JAZZY_HARDWARE_INTERFACE ${CMAKE_BINARY_DIR}/is_jazzy
  SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/cmake/has_jazzy_hardware_interface.cpp
  CXX_STANDARD 20 LINK_LIBRARIES hardware_interface::hardware_interface
  OUTPUT_VARIABLE COMPILE_LOG
)

if(HAS_JAZZY_HARDWARE_INTERFACE)
  target_compile_definitions(arm_can_interface PUBLIC HARDWARE_INTERFACE_IS_JAZZY=1)
else()
  target_compile_definitions(arm_can_interface PUBLIC HARDWARE_INTERFACE_IS_JAZZY=0)
endif()

pluginlib_export_plugin_description_file(hardware_interface arm_can_hardware.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(
  TARGETS arm_can_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
