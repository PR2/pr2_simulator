cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init(test_pr2_sensors_gazebo)

rosbuild_add_rostest_labeled(gazebo test_scan.launch)
rosbuild_add_rostest_labeled(gazebo test_camera.launch)
rosbuild_add_rostest_labeled(gazebo hztest_pr2_scan.launch)
rosbuild_add_rostest_labeled(gazebo hztest_pr2_image.launch)
#rosbuild_add_rostest_labeled(gazebo hztest_pr2_stereo.launch)



