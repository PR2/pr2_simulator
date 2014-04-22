cmake_minimum_required(VERSION 2.8.3)
project(pr2_gazebo)

find_package(catkin REQUIRED)

catkin_package(
  DEPENDS 
    gazebo 
    SDF
)

# Install Gazebo Scripts
install(
  PROGRAMS
    scripts/pr2_fingertip_pressure_contact_translator.py
    scripts/pr2_simulate_torso_spring.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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

# Install some gazebo test models
install(
  DIRECTORY
    urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
