cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init(test_pr2_2dnav_gazebo)

# Tests are disabled because of spurious failures, pending move to new
# machine dedicated to running Gazebo tests.
#
## For now, the tests fail on 64-bit for some reason.
## TODO: more thoroughly test and wrap up this 32-bit check
#include(CMakeDetermineSystem)
#if(CMAKE_SYSTEM_PROCESSOR MATCHES "i686" OR
#   CMAKE_SYSTEM_PROCESSOR MATCHES "i386" OR
#   CMAKE_SYSTEM_PROCESSOR MATCHES "unknown")
#  set(_is32bit TRUE)
#else(CMAKE_SYSTEM_PROCESSOR MATCHES "i686" OR
#     CMAKE_SYSTEM_PROCESSOR MATCHES "i386" OR
#     CMAKE_SYSTEM_PROCESSOR MATCHES "unknown")
#  set(_is32bit FALSE)
#endif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686" OR
#      CMAKE_SYSTEM_PROCESSOR MATCHES "i386" OR
#      CMAKE_SYSTEM_PROCESSOR MATCHES "unknown")
#
#if(CMAKE_SYSTEM_NAME MATCHES "Linux" AND _is32bit)
#
rospack_add_rostest(test_pr2_2dnav_wg.launch)
rospack_add_rostest(test_pr2_2dnav_empty_rotation.launch)
rospack_add_rostest(test_pr2_2dnav_empty_odom.launch)
rospack_add_rostest(test_pr2_2dnav_empty_diagonal.launch)
rospack_add_rostest(test_pr2_2dnav_empty_axis.launch)
rospack_add_rostest(test_pr2_2dnav_empty_amcl_axis.launch)
#
#endif(CMAKE_SYSTEM_NAME MATCHES "Linux" AND _is32bit)
