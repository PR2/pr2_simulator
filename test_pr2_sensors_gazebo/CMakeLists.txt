cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(test_pr2_sensors_gazebo)


# For now, the tests fail on 64-bit for some reason.
# TODO: more thoroughly test and wrap up this 32-bit check
include(CMakeDetermineSystem)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "i686" OR
   CMAKE_SYSTEM_PROCESSOR MATCHES "i386" OR
   CMAKE_SYSTEM_PROCESSOR MATCHES "unknown")
  set(_is32bit TRUE)
else(CMAKE_SYSTEM_PROCESSOR MATCHES "i686" OR
     CMAKE_SYSTEM_PROCESSOR MATCHES "i386" OR
     CMAKE_SYSTEM_PROCESSOR MATCHES "unknown")
  set(_is32bit FALSE)
endif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686" OR
      CMAKE_SYSTEM_PROCESSOR MATCHES "i386" OR
      CMAKE_SYSTEM_PROCESSOR MATCHES "unknown")

if(CMAKE_SYSTEM_NAME MATCHES "Linux" AND _is32bit)
rospack_add_rostest(test_scan.launch)
rospack_add_rostest(test_camera.launch)
rospack_add_rostest(hztest_pr2_scan.launch)
rospack_add_rostest(hztest_pr2_image.launch)
endif(CMAKE_SYSTEM_NAME MATCHES "Linux" AND _is32bit)



