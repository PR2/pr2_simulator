cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()

rosbuild_add_boost_directories()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# test torso and gripper
#  find_package(Qt4 REQUIRED)
#  include(${QT_USE_FILE})
#  include_directories(${CMAKE_BINARY_DIR})
#  include_directories(${QT_INCLUDE_DIR})
#  set(qt_ros_slider_headers include/pr2_gazebo/qt_ros_slider.h)
#  qt4_wrap_cpp(qt_ros_slider_moc_sources ${qt_ros_slider_headers})
#  rosbuild_add_library(qt_ros_slider src/qt_ros_slider.cpp ${qt_ros_slider_moc_sources})
#  target_link_libraries(qt_ros_slider ${QT_LIBRARIES})
#  rosbuild_add_executable(torso_gripper_sliders src/torso_gripper_sliders.cpp)
#  target_link_libraries(torso_gripper_sliders qt_ros_slider ${QT_LIBRARIES})
#  rosbuild_link_boost(torso_gripper_sliders thread)

# move pr2 test
rosbuild_add_executable(move_pr2 src/move_pr2.cpp)
rosbuild_link_boost(move_pr2 thread)


#tests
rosbuild_add_rostest_labeled(gazebo test/collisions/test_slide.launch)
rosbuild_add_rostest_labeled(gazebo test/mechanism_controllers/test_base.launch)
rosbuild_add_rostest_labeled(gazebo test/mechanism_controllers/test_arm.launch)
rosbuild_add_rostest_labeled(gazebo test/mechanism_controllers/hztest_pr2_mechanism.launch)
rosbuild_add_rostest_labeled(gazebo test/mechanism_controllers/hztest_rostime.launch)
rosbuild_add_rostest_labeled(gazebo test/mechanism_controllers/hztest_pr2_odom.launch)

rosbuild_add_rostest_labeled(gazebo test/sensors/test_scan.launch)
rosbuild_add_rostest_labeled(gazebo test/sensors/test_camera.launch)
rosbuild_add_rostest_labeled(gazebo test/sensors/hztest_pr2_scan.launch)
rosbuild_add_rostest_labeled(gazebo test/sensors/hztest_pr2_image.launch)
rosbuild_add_rostest_labeled(gazebo test/sensors/hztest_pr2_stereo.launch)
rosbuild_add_rostest_labeled(gazebo test/spawn_test/test_pr2_spawn_with_offset.launch)
