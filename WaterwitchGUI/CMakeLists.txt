cmake_minimum_required(VERSION 3.25)
project(WaterwitchGUI)

find_package(OpenGL REQUIRED)

add_subdirectory(include/glfw)
add_subdirectory(include/json)

include_directories(
    ${OPENGL_INCLUDE_DIRS}
    include/glfw/include
    include/imgui
    include/imgui/backends
)

set(IMGUI_SOURCES
    include/imgui/imgui.cpp
    include/imgui/imgui_draw.cpp
    include/imgui/imgui_widgets.cpp
    include/imgui/imgui_tables.cpp
    include/imgui/imgui_demo.cpp
    include/imgui/backends/imgui_impl_glfw.cpp
    include/imgui/backends/imgui_impl_opengl3.cpp
)

add_executable(WaterwitchGUI src/main.cpp ${IMGUI_SOURCES})

target_link_libraries(WaterwitchGUI 
    ${OPENGL_LIBRARIES} 
    glfw
    nlohmann_json::nlohmann_json
)

set_target_properties(WaterwitchGUI PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)