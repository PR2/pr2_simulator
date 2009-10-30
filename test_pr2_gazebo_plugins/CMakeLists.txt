cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init(test_pr2_gazebo_plugins)

rosbuild_add_rostest_labeled(test_pendulum.launch)

