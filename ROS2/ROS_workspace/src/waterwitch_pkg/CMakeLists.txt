cmake_minimum_required(VERSION 3.8)
project(waterwitch_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eer_messages REQUIRED) 

add_executable(pilot_listener src/pilot_listener.cpp)
ament_target_dependencies(pilot_listener rclcpp std_msgs eer_messages)

install(TARGETS
  pilot_listener
  DESTINATION lib/${PROJECT_NAME})

# Add the launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
