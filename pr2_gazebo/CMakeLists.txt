cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init(pr2_gazebo)

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

rosbuild_add_pyunit(test/urdf2model/test_urdf2model_parser.py)

rosbuild_add_rostest_labeled(gazebo test/spawn_test/test_pr2_spawn_with_offset.launch)

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
include_directories(${CMAKE_BINARY_DIR})
include_directories(${QT_INCLUDE_DIR})

set(lcdrange_headers include/pr2_gazebo/lcdrange.h)
qt4_wrap_cpp(lcdrange_moc_sources ${lcdrange_headers})
rosbuild_add_library(lcdrange src/lcdrange.cpp ${lcdrange_moc_sources})
target_link_libraries(lcdrange ${QT_LIBRARIES})

rosbuild_add_executable(qt_slider_commander src/qt_slider_commander.cpp)
target_link_libraries(qt_slider_commander lcdrange ${QT_LIBRARIES})
rosbuild_link_boost(qt_slider_commander thread)

