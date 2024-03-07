cmake_minimum_required(VERSION 3.8)
project(ur_cb2_driver_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# some needed stuff
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ur_cb2_driver REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

include_directories(include)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  controller_manager
  ur_cb2_driver
)

# UR_CB2_DRIVER:
get_target_property(ur_cb2_driver_shared_INCLUDE_DIRS ${ur_cb2_driver_LIBRARIES} INTERFACE_INCLUDE_DIRECTORIES)
include_directories(${ur_cb2_driver_INCLUDE_DIRS})
set(UR_CB2_DRIVER_LIBS ${UR_CB2_DRIVER_LIBS} ${ur_cb2_driver_LIBRARIES})

# Hardware interface things:
add_library(ur_cb2_driver_ros_plugin
  SHARED
  src/hardware_interface.cpp
)
target_link_libraries(
  ur_cb2_driver_ros_plugin
)
target_include_directories(
  ur_cb2_driver_ros_plugin
  PRIVATE
  include
)
ament_target_dependencies(
  ur_cb2_driver_ros_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# prevent pluginlib from using boost
target_compile_definitions(ur_cb2_driver_ros_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# ur_ros2_control_node:
add_executable(ur_ros2_control_node
  ./src/ur_ros2_control_node.cpp
)
ament_target_dependencies(ur_ros2_control_node
  rclcpp
  controller_manager
)

# test_node:
add_executable(test_node
  ./src/test_node.cpp
)
ament_target_dependencies(test_node
  rclcpp
  ur_cb2_driver
)

# INSTALL:
install(TARGETS
  test_node ur_ros2_control_node
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS ur_cb2_driver_ros_plugin
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

# EXPORTS:
ament_export_include_directories(
  include
)
ament_export_libraries(
  ur_cb2_driver_ros_plugin
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
