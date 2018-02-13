cmake_minimum_required(VERSION 2.8.3)
project(pr2_controller_configuration_gazebo)

find_package(gazebo REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDF sdformat REQUIRED)

find_package(catkin REQUIRED)

catkin_package(
  DEPENDS 
    GAZEBO
    SDF
)

# Install Gazebo launch files
install(
  DIRECTORY
    config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(
  DIRECTORY
    launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
