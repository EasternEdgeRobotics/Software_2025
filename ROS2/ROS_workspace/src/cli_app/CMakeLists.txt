cmake_minimum_required(VERSION 3.8)
project(cli_app)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(glfw3 REQUIRED)
find_package(eer_messages REQUIRED) 

# ImGui configuration
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/imgui)
set(IMGUI_SOURCES
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_demo.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# Source files
set(SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/gui.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cli.cpp
  ${IMGUI_SOURCES}
)

# Include directories
include_directories(
  ${IMGUI_DIR}
  ${IMGUI_DIR}/backends
  ${OpenCV_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${glfw3_INCLUDE_DIRS}
  ${eer_messages_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add executable
add_executable(${PROJECT_NAME}_node ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBRARIES}
  glfw
  GL
)

# Link ROS 2 dependencies
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  eer_messages 
  rosidl_default_runtime
)

# Install executable
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()