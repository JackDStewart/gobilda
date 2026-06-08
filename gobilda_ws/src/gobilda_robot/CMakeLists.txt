cmake_minimum_required(VERSION 3.10)
project(gobilda_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)         # added this one also in rectangle_motion
find_package(geometry_msgs REQUIRED)    # added this for script to move in a rectangle
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


# Build cobs_c as a library
add_library(cobs_c STATIC hardware/libs/cobs-c/cobs.c)
target_include_directories(cobs_c PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/libs/cobs-c>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
add_library(${PROJECT_NAME}
  SHARED
  hardware/src/gobilda_system.cpp
  hardware/src/motor.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/gobilda_robot>
)

target_link_libraries(${PROJECT_NAME}
  cobs_c
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# target_compile_definitions(${PROJECT_NAME} PRIVATE "GOBILDA_ROBOT_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface gobilda_robot.xml)

install(
  DIRECTORY hardware/include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  cobs_c
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  hardware/libs/cobs-c/cobs.h
  DESTINATION include/${PROJECT_NAME}
)

# for moving in a rectangle
add_executable(rectangle_motion src/rectangle_motion.cpp)
target_compile_features(rectangle_motion PUBLIC cxx_std_20)
ament_target_dependencies(rectangle_motion rclcpp geometry_msgs nav_msgs)

install(TARGETS rectangle_motion
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
