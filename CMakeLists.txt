cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

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
target_link_libraries(pr2_collision_checker leatherman sbpl_manipulation_components sbpl_geometry_utils)

rosbuild_add_executable(testPR2CollisionChecker src/test_pr2_collision_checker.cpp)
target_link_libraries(testPR2CollisionChecker pr2_collision_checker)

