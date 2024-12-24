cmake_minimum_required(VERSION 3.8)
project(waterwitch_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eer_messages REQUIRED) 
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(
  ${eer_messages_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add the custom interfaces (msg, srv, action)
rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/WaterwitchThrustValues.msg"
)
ament_export_dependencies(rosidl_default_runtime)

# Ensure that we can use the custom interfaces created in the same ros package
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# This file contains some constants used throughout the waterwitch backend
# It is a static library so that it can be linked to other executables prior to runtime
add_library(waterwitch_constants STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/waterwitch_constants.cpp)

add_executable(pilot_listener ${CMAKE_CURRENT_SOURCE_DIR}/src/pilot_listener.cpp)
add_executable(simulation_bot_control ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation_bot_control.cpp)

ament_target_dependencies(simulation_bot_control rclcpp std_msgs eer_messages)
ament_target_dependencies(pilot_listener rclcpp std_msgs eer_messages)

target_link_libraries(pilot_listener "${cpp_typesupport_target}" waterwitch_constants)
target_link_libraries(simulation_bot_control "${cpp_typesupport_target}" waterwitch_constants)

install(TARGETS
  pilot_listener
  simulation_bot_control
  waterwitch_constants
  DESTINATION lib/${PROJECT_NAME}
)

# Add the launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
