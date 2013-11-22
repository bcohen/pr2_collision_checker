cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#find_package(catkin REQUIRED COMPONENTS sbpl_geometry_utils)
#include_directories(${catkin_INCLUDE_DIRS})
#message("HELLO ANDREW")
#message("${catkin_INCLUDE_DIRS}")
#message("BYE BYE ANDREW")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(pr2_collision_checker src/sbpl_arm_model.cpp
                                           src/pr2_collision_space.cpp
                                           src/pr2_collision_space_interface.cpp
                                           src/pr2_collision_space_monitor.cpp)
target_link_libraries(pr2_collision_checker leatherman sbpl_manipulation_components pviz ${catkin_LIBRARIES})

rosbuild_add_executable(testVoxelizer src/test_voxelizer.cpp)
target_link_libraries(testVoxelizer pr2_collision_checker leatherman)

