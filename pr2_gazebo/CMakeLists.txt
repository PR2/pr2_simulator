cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init(pr2_gazebo)

rosbuild_add_rostest_labeled(gazebo test/collisions/test_slide.launch)
