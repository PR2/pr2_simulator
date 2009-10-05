cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(test_pr2_mechanism_controllers_gazebo)

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
if("$ENV{ROS_BUILD_TEST_LABEL}" STREQUAL "" OR "$ENV{ROS_BUILD_TEST_LABEL}" STREQUAL "gazebo")
  rospack_add_rostest(test_base.launch)
  rospack_add_rostest(test_arm.launch)
  rospack_add_rostest(hztest_pr2_mechanism.launch)
  rospack_add_rostest(hztest_rostime.launch)
  rospack_add_rostest(hztest_pr2_odom.launch)
endif("$ENV{ROS_BUILD_TEST_LABEL}" STREQUAL "" OR "$ENV{ROS_BUILD_TEST_LABEL}" STREQUAL "gazebo")
#
#endif(CMAKE_SYSTEM_NAME MATCHES "Linux" AND _is32bit)
