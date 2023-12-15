cmake_minimum_required(VERSION 3.8)
project(ioc_ur_cb2_driver_ros)

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
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ioc_ur_cb2_driver REQUIRED)

get_target_property(ioc_ur_cb2_driver_INCLUDE_DIRS ${ioc_ur_cb2_driver_LIBRARIES} INTERFACE_INCLUDE_DIRECTORIES)
include_directories(${ioc_ur_cb2_driver_INCLUDE_DIRS})
set(IOC_UR_CB2_DRIVER_LIBS ${IOC_UR_CB2_DRIVER_LIBS} ${ioc_ur_cb2_driver_LIBRARIES})


add_executable(test_node ./src/test_node.cpp)
ament_target_dependencies(test_node rclcpp ioc_ur_cb2_driver)
# target_link_libraries(test_node ${IOC_UR_CB2_DRIVER_LIBS})

install(TARGETS
  test_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
