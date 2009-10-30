cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init(test_pr2_mechanism_controllers_gazebo)

rosbuild_add_rostest_labeled(gazebo test_base.launch)
rosbuild_add_rostest_labeled(gazebo test_arm.launch)
rosbuild_add_rostest_labeled(gazebo hztest_pr2_mechanism.launch)
rosbuild_add_rostest_labeled(gazebo hztest_rostime.launch)
rosbuild_add_rostest_labeled(gazebo hztest_pr2_odom.launch)
