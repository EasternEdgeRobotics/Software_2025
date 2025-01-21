cmake_minimum_required(VERSION 3.8)
project(cpp_frontend)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(eer_interfaces REQUIRED) 

# Add the glfw and json libraries
add_subdirectory(include/glfw)
add_subdirectory(include/json)

# Import imgui, and glfw
include_directories(
    ${OPENGL_INCLUDE_DIRS}
    include/glfw/include
    include/imgui
    include/imgui/backends
)

# ImGui configuration
set(IMGUI_SOURCES
    include/imgui/imgui.cpp
    include/imgui/imgui_draw.cpp
    include/imgui/imgui_widgets.cpp
    include/imgui/imgui_tables.cpp
    include/imgui/imgui_demo.cpp
    include/imgui/backends/imgui_impl_glfw.cpp
    include/imgui/backends/imgui_impl_opengl3.cpp
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
  ${eer_interfaces_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add executable
add_executable(${PROJECT_NAME}_node ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBRARIES}
  glfw
  GL
  nlohmann_json::nlohmann_json
)

# Link ROS 2 dependencies
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  eer_interfaces 
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