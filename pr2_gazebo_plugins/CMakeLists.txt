cmake_minimum_required(VERSION 2.8.12)
project(pr2_gazebo_plugins)

add_definitions(-fPIC)

# system dependencies
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(XML libxml-2.0)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

# Depend on system install of Gazebo and SDFormat
#find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(orocos_kdl)

# catkin dependencies
find_package(catkin
  REQUIRED COMPONENTS 
    angles
    cv_bridge
    diagnostic_msgs
    diagnostic_updater
    gazebo_dev
    gazebo_msgs
    gazebo_plugins
    geometry_msgs
    image_transport
    message_generation 
    nav_msgs
    polled_camera
    pr2_controller_manager
    pr2_hardware_interface
    pr2_mechanism_model
    pr2_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    std_srvs 
    urdf
  #  pcl_ros
  #  pcl_conversions
)

# Generate msgs and srvs
add_message_files(
  DIRECTORY msg
  FILES
    PlugCommand.msg
    ModelJointsState.msg
)

add_service_files(
  DIRECTORY srv FILES
    SetModelsJointsStates.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    nav_msgs
    sensor_msgs
    pr2_msgs
    geometry_msgs
    diagnostic_msgs
)

# declare catkin_package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    message_runtime
    roscpp 
    rospy 
    angles 
    std_msgs
    nav_msgs
    sensor_msgs
    pr2_msgs
    geometry_msgs
    diagnostic_msgs
    gazebo_msgs
    urdf 
    tf 
    image_transport 
  DEPENDS 
    orocos_kdl
  LIBRARIES
    gazebo_ros_controller_manager
    gazebo_ros_power_monitor
)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${orocos_kdl_LIBRARY_DIRS}
)

# Gazebo ogre path setup
execute_process(COMMAND
  pkg-config --variable=plugindir OGRE
  OUTPUT_VARIABLE OGRE_PLUGIN_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# build libraries
add_library(gazebo_ros_controller_manager src/gazebo_ros_controller_manager.cpp)
add_dependencies(gazebo_ros_controller_manager
  ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(gazebo_ros_controller_manager ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_power_monitor src/gazebo_ros_power_monitor.cpp)
add_dependencies(gazebo_ros_power_monitor
  ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(gazebo_ros_power_monitor ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# install libraries
install(
  TARGETS 
    gazebo_ros_controller_manager
    gazebo_ros_power_monitor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# test for glx presence for testing
exec_program("glxinfo" OUTPUT_VARIABLE glx_out RETURN_VALUE glx_val)
if(${glx_val} EQUAL 0)
  message(STATUS "run test here")
else(${glx_val} EQUAL 0)
  message(STATUS "no test here")
endif(${glx_val} EQUAL 0)
