cmake_minimum_required(VERSION 3.8)
project(coppeliasim_plugin_velodyne)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include)

# Add library
add_library(v_repExtRosVelodyne SHARED
            src/v_repLib.cpp
            src/ros_server_velodyne.cpp
            src/velodyneROSModel.cpp
            src/velodyneROSModelCont.cpp
            src/v_repExtVelodyneROS.cpp
            src/scriptFunctionDataItem.cpp
            src/scriptFunctionData.cpp
            include/v_repLib.h
            include/scriptFunctionData.h
            include/scriptFunctionDataItem.h
            include/coppeliasim_plugin_velodyne/ros_server_velodyne.h
            include/coppeliasim_plugin_velodyne/velodyneROSModel.h
            include/coppeliasim_plugin_velodyne/velodyneROSModelCont.h
            include/coppeliasim_plugin_velodyne/v_repExtVelodyneROS.h)

# Link dependencies
ament_target_dependencies(v_repExtRosVelodyne
  rclcpp
  std_msgs
  sensor_msgs
)

target_link_libraries(v_repExtRosVelodyne
    pthread
    dl
)

# Install
install(TARGETS v_repExtRosVelodyne
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
