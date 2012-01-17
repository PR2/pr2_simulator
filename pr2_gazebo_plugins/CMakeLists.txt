cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()

rosbuild_genmsg()
rosbuild_gensrv()

add_definitions(-fPIC)


# 
# if screw joint exists, remove hack for damping variable
#
execute_process(COMMAND rosrun opende ode-config --version OUTPUT_VARIABLE ode_version)
if (${ode_version} VERSION_LESS "0.11.1.71")
  MESSAGE(STATUS "OUTPUT_VARIABLE: ${ode_version} no screw joint")
else (${ode_version} VERSION_LESS "0.11.1.71")
  MESSAGE(STATUS "OUTPUT_VARIABLE: ${ode_version} screw joint exists, removing hack")
  add_definitions(-DODE_SCREW_JOINT)
endif (${ode_version} VERSION_LESS "0.11.1.71")




# dynamic reconfigure for gazebo ros camera nodes
# does not exist until simulator_gazebo 1.2.9
execute_process(COMMAND rosversion simulator_gazebo OUTPUT_VARIABLE simulator_gazebo_version)
if (${simulator_gazebo_version} VERSION_GREATER "1.2.8")
  MESSAGE(STATUS "OUTPUT_VARIABLE: ${simulator_gazebo_version}Enabling gazebo ros camera dynamic reconfigure")
  add_definitions(-DSIMULATOR_GAZEBO_GAZEBO_ROS_CAMERA_DYNAMIC_RECONFIGURE)
else (${simulator_gazebo_version} VERSION_GREATER "1.2.8")
  MESSAGE(STATUS "OUTPUT_VARIABLE: ${simulator_gazebo_version}Disabling gazebo ros camera dynamic reconfigure")
endif (${simulator_gazebo_version} VERSION_GREATER "1.2.8")

rosbuild_add_boost_directories()

rosbuild_add_library(gazebo_ros_controller_manager src/gazebo_ros_controller_manager.cpp)
rosbuild_link_boost(gazebo_ros_controller_manager thread)
rosbuild_add_library(gazebo_ros_power_monitor src/gazebo_ros_power_monitor.cpp)
rosbuild_link_boost(gazebo_ros_power_monitor thread)
rosbuild_add_library(gazebo_ros_prosilica src/gazebo_ros_prosilica.cpp)
rosbuild_link_boost(gazebo_ros_prosilica thread)
target_link_libraries(gazebo_ros_prosilica gazebo_ros_camera_utils DepthCameraPlugin)

exec_program("glxinfo" OUTPUT_VARIABLE glx_out RETURN_VALUE glx_val)
if(${glx_val} EQUAL 0)
  message(STATUS "run test here")
else(${glx_val} EQUAL 0)
  message(STATUS "no test here")
endif(${glx_val} EQUAL 0)

