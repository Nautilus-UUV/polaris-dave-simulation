cmake_minimum_required(VERSION 3.5)
project(sliding_mass_velocity_slider_plugin)

# Find Gazebo and Qt5 packages
find_package(gazebo REQUIRED)
find_package(Qt5 REQUIRED Widgets)

# Set plugin output directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Add the plugin source
add_library(sliding_mass_velocity_slider_plugin SHARED src/SlidingMassVelocitySliderPlugin.cc)

# Link Gazebo and Qt5 to the plugin
target_link_libraries(sliding_mass_velocity_slider_plugin
  ${GAZEBO_LIBRARIES}
  Qt5::Widgets
)

# Specify include directories
include_directories(${GAZEBO_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS})

