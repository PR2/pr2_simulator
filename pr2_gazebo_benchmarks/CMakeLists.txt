cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_library(gazebo_physics_controllers
    src/joint_gravity_controller.cpp
    src/joint_pendulum_controller.cpp
)

rosbuild_remove_compile_flags(gazebo_physics_controllers -W)
target_link_libraries(gazebo_physics_controllers ltdl)

#rosbuild_add_boost_directories()
#rosbuild_add_library(gazebo_ros_benchmarks src/gazebo_ros_benchmarks.cpp)
#rosbuild_link_boost(gazebo_ros_benchmarks thread)

