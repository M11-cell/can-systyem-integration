cmake_minimum_required(VERSION 3.22)
project(encoder_boards)

set(CMAKE_CXX_STANDARD 23)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
    rclcpp
    can_util
    ros2_fmt_logger
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(backward_ros REQUIRED)

foreach(dependency IN ITEMS ${DEPENDENCIES})
    find_package(${dependency} REQUIRED)
endforeach()

add_library(encoder_boards SHARED
    src/arm_encoder_feedback.cpp
)

target_include_directories(encoder_boards PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(encoder_boards PUBLIC ${DEPENDENCIES})

install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

install(TARGETS encoder_boards
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

add_executable(encoder_telemetry_node src/encoder_telemetry_node.cpp)
target_link_libraries(encoder_telemetry_node PUBLIC encoder_boards)
ament_target_dependencies(encoder_telemetry_node PUBLIC ${DEPENDENCIES})

install(TARGETS encoder_telemetry_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(encoder_boards)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})
ament_package()
