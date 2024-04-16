cmake_minimum_required(VERSION 3.5)
project(global_elevation_mapping)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  # geometry_msgs
  # nav_msgs
  # nav2_core
  # tf2
  # tf2_ros
  grid_map_ros
  # mppi_controller
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES snap_vislam
 CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###########
# Library #
###########

add_library(global_elevation_mapping
  src/global_elevation_mapping.cpp
)

add_dependencies(global_elevation_mapping
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(global_elevation_mapping
  ${catkin_LIBRARIES}
)

##############
# Executable #
##############

add_executable(global_elevation_mapping_node src/global_elevation_mapping_node.cpp)

add_dependencies(global_elevation_mapping_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(global_elevation_mapping_node
  global_elevation_mapping
  ${catkin_LIBRARIES}
)

###########
# Install #
###########

