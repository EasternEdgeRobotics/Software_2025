# CMake file adapted from https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/plugin/hello_world

cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

# Find the Gazebo CMake package, which allows for using custom CMake functions provided by Gazebo
# See https://github.com/gazebosim/gz-cmake
find_package(gz-cmake3 REQUIRED)

# Set the project name
project(EER_SIMULAITON_PLUGINS)

# Find the gz-plugin2, ensuring that it contains the plugin registering functionality
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

# Find the gz-sim8 package, which includes the Gazebo C++ libraries and headers  
gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

# Add the plugin libraries
add_library(Buoyancy SHARED src/Buoyancy.cc src/Buoyancy.hh)

# Set the C++ standard to C++17
set_property(TARGET Buoyancy PROPERTY CXX_STANDARD 17)

# Link the plugin library to Gazebo libraries 
target_link_libraries(Buoyancy
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

# Tell CMake to put the plugin library in the plugins directory
set_target_properties(Buoyancy PROPERTIES 
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/plugins
    )

# ----------------------------------------------
# ------=== Alternative CMakeLists.txt ===------
# ----------------------------------------------

# Below is working CMakeLists.txt for loading only the Buoyancy plugin
# It does not require the gz-cmake3 package

# cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

# # Set the project name
# project(EER_SIMULAITON_PLUGINS)

# # Find the Gazebo Package, which includes the libraries and headers for C++   
# find_package(gz-sim8 REQUIRED)

# # Add the plugin libraries
# add_library(Buoyancy SHARED src/Buoyancy.cc src/Buoyancy.hh)

# # Link the plugin library to Gazebo libraries (from the gz-sim8 package)
# target_link_libraries(Buoyancy PUBLIC ${gz-sim8_LIBRARIES})

# # Tell CMake to put the plugin library in the plugins directory
# set_target_properties(Buoyancy PROPERTIES 
#     LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/plugins
#     )

